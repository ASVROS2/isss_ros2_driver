"""A stand-in for the ISSS on the network, for bench work and tests without hardware.

``SimulatedSonar`` listens on TCP like the real device, records the register
writes it receives, and streams packets: either replayed from a recorded
``.bin`` log, or synthetic pings whose size follows the ``0x11`` register.

What it does *not* know about the real device: whether streaming starts on
connect or only after configuration, how the device paces pings, and how it
reacts to unknown registers. Those belong in hardware bench tests.
"""

from __future__ import annotations

import math
import socket
import struct
import threading
from pathlib import Path
from typing import Sequence

from isss.log import find_packet_offsets
from isss.protocol import (
    DEFAULT_SAMPLES,
    FREQ_ID_DEFAULT,
    HEADER_SIZE,
    PACKET_TYPE_ISSS,
    SOP1,
    SOP2,
    counts_from_depression,
    expected_ping_interval,
    packet_size,
    range_from_decimation,
)

REG_DECIMATION = 0x10
REG_NUM_SAMPLES = 0x11
REG_NUM_PHASES = 0x12

# Header bytes 2-3 read 6007 in every recorded log, including 2000-sample ones,
# so the field is not the packet size. Mimic the device rather than the docs.
OBSERVED_SIZE_FIELD = 6007


def packets_from_log(path: str | Path, *, num_samples: int = DEFAULT_SAMPLES) -> list[bytes]:
    """Split a recorded .bin log into raw packets."""
    data = Path(path).read_bytes()
    size = packet_size(num_samples)
    return [
        data[offset : offset + size]
        for offset in find_packet_offsets(data, num_samples=num_samples)
        if offset + size <= len(data)
    ]


def synthetic_packet(
    *,
    num_samples: int = DEFAULT_SAMPLES,
    ping_time: int = 0,
    decimation: int = 111,
    bottom_sample: int | None = None,
    counts_per_unit_sine: float = 15218.0,
    boresight_depression_deg: float = 44.5,
) -> bytes:
    """Build one ping over a flat bottom: water column, then the seabed.

    Range is measured in samples, so a bin ``i`` beyond the first bottom return
    ``b`` sees the seabed at depression ``asin(b / i)``. Phase follows the
    interferometer model of ``isss.protocol``, so decoding a synthetic ping
    recovers that geometry. Both sides see the same seabed.
    """
    bottom = bottom_sample if bottom_sample is not None else num_samples // 5
    beta0 = math.radians(boresight_depression_deg)
    header = bytearray(HEADER_SIZE)
    header[0], header[1] = SOP1, SOP2
    struct.pack_into(">H", header, 2, OBSERVED_SIZE_FIELD)
    header[4] = PACKET_TYPE_ISSS
    struct.pack_into(">I", header, 5, ping_time & 0xFFFFFFFF)
    header[9] = FREQ_ID_DEFAULT
    struct.pack_into(">H", header, 10, decimation & 0xFFFF)

    payload = bytearray(num_samples * 6)
    for i in range(num_samples):
        if i < bottom:
            rss, counts = 5, 0
        else:
            rss = int(200 * math.exp(-(i - bottom) / (num_samples / 4))) + 10
            depression = math.asin(bottom / max(i, 1))
            counts = round(counts_from_depression(depression, counts_per_unit_sine, beta0))
        phase = counts & 0xFFFF
        payload[i * 6 : i * 6 + 6] = bytes(
            [phase >> 8, phase & 0xFF, rss, phase >> 8, phase & 0xFF, rss]
        )
    return bytes(header) + bytes(payload)


class SimulatedSonar:
    """Serve one client at a time on ``host:port`` (port 0 picks a free port).

    With ``rate_hz=None`` pings are paced like the device: one per two-way
    travel time for the range implied by the decimation and sample registers.
    """

    def __init__(
        self,
        packets: Sequence[bytes] | None = None,
        *,
        host: str = "127.0.0.1",
        port: int = 0,
        rate_hz: float | None = None,
    ) -> None:
        self.packets = list(packets) if packets else None
        self.host = host
        self.rate_hz = rate_hz
        self.registers: dict[int, int] = {
            REG_DECIMATION: 111,
            REG_NUM_SAMPLES: DEFAULT_SAMPLES,
            REG_NUM_PHASES: 2,
        }
        self.register_writes: list[tuple[int, int]] = []
        self.pings_sent = 0

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self.port = self._server.getsockname()[1]

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._client: socket.socket | None = None
        self._thread: threading.Thread | None = None

    # Lifecycle

    def start(self) -> SimulatedSonar:
        self._server.listen(1)
        self._server.settimeout(0.1)
        self._thread = threading.Thread(target=self._accept_loop, name="isss-sim", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self.drop_client()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._server.close()

    def __enter__(self) -> SimulatedSonar:
        return self.start()

    def __exit__(self, *_exc: object) -> None:
        self.stop()

    def drop_client(self) -> None:
        """Close the current connection, as a cable pull or device reboot would."""
        with self._lock:
            client, self._client = self._client, None
        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()

    # Serving

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                client, _addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self._lock:
                self._client = client
            reader = threading.Thread(target=self._read_registers, args=(client,), daemon=True)
            reader.start()
            self._stream(client)
            with self._lock:
                if self._client is client:
                    self._client = None
            client.close()
            reader.join(timeout=1.0)

    def _read_registers(self, client: socket.socket) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = client.recv(64)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while len(buf) >= 4:
                if buf[0] != SOP1:
                    buf = buf[1:]
                    continue
                reg, value = buf[1], (buf[2] << 8) | buf[3]
                buf = buf[4:]
                with self._lock:
                    self.registers[reg] = value
                    self.register_writes.append((reg, value))

    def _period(self, registers: dict[int, int]) -> float:
        if self.rate_hz is not None:
            return 1.0 / self.rate_hz
        return expected_ping_interval(
            range_from_decimation(registers[REG_DECIMATION], registers[REG_NUM_SAMPLES])
        )

    def _stream(self, client: socket.socket) -> None:
        index = 0
        ping_time = 0
        while not self._stop.is_set():
            with self._lock:
                if self._client is not client:
                    return
                registers = dict(self.registers)
            period = self._period(registers)
            if self.packets is not None:
                packet = self.packets[index % len(self.packets)]
            else:
                packet = synthetic_packet(
                    num_samples=registers[REG_NUM_SAMPLES],
                    ping_time=ping_time,
                    decimation=registers[REG_DECIMATION],
                )
            try:
                client.sendall(packet)
            except OSError:
                return
            with self._lock:
                self.pings_sent += 1
            index += 1
            ping_time += int(period * 1e4)
            self._stop.wait(period)


def main(argv: list[str] | None = None) -> None:
    """``isss-sim``: serve a log (or synthetic pings) as if it were the sonar."""
    import argparse

    parser = argparse.ArgumentParser(description="Simulate the ISSS on TCP for bench testing")
    parser.add_argument("log", nargs="?", type=Path, help="Replay this .bin log (default: synthetic)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=57349)
    parser.add_argument("--rate", type=float, default=None, help="Pings per second (default: paced by range, like the device)")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES, help="Samples per channel in the log")
    args = parser.parse_args(argv)

    packets = packets_from_log(args.log, num_samples=args.samples) if args.log else None
    if args.log and not packets:
        parser.error(f"No packets found in {args.log}")

    sim = SimulatedSonar(packets, host=args.host, port=args.port, rate_hz=args.rate).start()
    source = f"{len(packets)} packets from {args.log}" if packets else "synthetic pings"
    pacing = f"{args.rate} Hz" if args.rate else "range-paced"
    print(f"Simulated ISSS on {args.host}:{sim.port} serving {source}, {pacing} (Ctrl-C to stop)", flush=True)
    seen = 0
    try:
        while True:
            threading.Event().wait(0.5)
            with sim._lock:
                writes = sim.register_writes[seen:]
            for reg, value in writes:
                print(f"register 0x{reg:02X} <- {value}", flush=True)
            seen += len(writes)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
