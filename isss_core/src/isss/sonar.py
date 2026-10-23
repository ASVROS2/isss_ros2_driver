"""TCP interface to the DeepVision ISSS."""

from __future__ import annotations

import socket
from collections import deque
from typing import Iterator

from isss.config import SonarConfig
from isss.protocol import (
    HEADER_SIZE,
    SampleData,
    compute_decimation,
    data_size,
    is_packet_sync_byte,
    packet_size,
    parse_header,
    parse_payload,
)


class ISSSonar:
    """Connect to the ISSS over TCP, configure registers, and read pings."""

    def __init__(self, config: SonarConfig | None = None) -> None:
        self.config = config or SonarConfig()
        self._sock: socket.socket | None = None

    def connect(self) -> None:
        """Open a TCP connection to the sonar."""
        if self.is_connected:
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.config.recv_timeout_sec)
        try:
            sock.connect((self.config.ip, self.config.port))
        except OSError:
            sock.close()
            raise
        self._sock = sock

    def disconnect(self) -> None:
        """Close the TCP connection."""
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)  # unblocks a recv() in another thread
            except OSError:
                pass
            sock.close()

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def setup_sonar(self) -> list[tuple[int, int]]:
        """Write decimation, sample count, and phase registers; returns the writes made."""
        if not self.is_connected:
            raise RuntimeError("Not connected")

        writes = [
            (0x10, compute_decimation(self.config.range_m, self.config.num_samples)),
            (0x11, self.config.num_samples),
            (0x12, self.config.num_phases),
        ]
        for reg, value in writes:
            self.set_register(reg, value)
        return writes

    def configure(
        self,
        *,
        range_m: float | None = None,
        num_samples: int | None = None,
        num_phases: int | None = None,
    ) -> None:
        """Update configuration and push new register values to the device."""
        if range_m is not None:
            self.config.range_m = range_m
        if num_samples is not None:
            self.config.num_samples = num_samples
        if num_phases is not None:
            self.config.num_phases = num_phases
        self.setup_sonar()

    def read_packet(self) -> bytes:
        """Read one raw ISSS packet from the TCP stream."""
        sock = self._sock  # local ref: disconnect() may run from another thread
        if sock is None:
            raise RuntimeError("Not connected")

        header_window: deque[int] = deque(maxlen=HEADER_SIZE)

        while True:
            byte = sock.recv(1)
            if not byte:
                raise ConnectionError("Connection closed while searching for packet header")
            header_window.append(byte[0])
            if len(header_window) == HEADER_SIZE and is_packet_sync_byte(bytes(header_window)):
                header = bytes(header_window)
                break

        payload_len = data_size(self.config.num_samples)
        payload = self._recv_exact(sock, payload_len)
        return header + payload

    def read_ping(self) -> tuple[SampleData, bytes]:
        """Read and decode one ping; returns (sample data, raw packet bytes)."""
        packet = self.read_packet()
        header = parse_header(packet[:HEADER_SIZE])
        samples = parse_payload(
            packet[HEADER_SIZE:],
            self.config.num_samples,
            self.config,
        )
        return samples, packet

    def stream_pings(self) -> Iterator[tuple[SampleData, bytes]]:
        """Yield decoded pings until the connection fails."""
        while True:
            yield self.read_ping()

    def set_register(self, reg: int, value: int) -> None:
        """Write one 16-bit register: ``[0xFE, reg, value_hi, value_lo]``."""
        sock = self._sock
        if sock is None:
            raise RuntimeError("Not connected")
        tx = bytes([0xFE, reg & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
        sock.sendall(tx)

    def recv_raw(self, max_bytes: int = 65536) -> bytes:
        """Whatever bytes are available (up to ``max_bytes``), without framing; b"" on close."""
        sock = self._sock
        if sock is None:
            raise RuntimeError("Not connected")
        return sock.recv(max_bytes)

    @staticmethod
    def _recv_exact(sock: socket.socket, nbytes: int) -> bytes:
        chunks: list[bytes] = []
        remaining = nbytes
        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                raise ConnectionError("Connection closed while reading packet payload")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def __enter__(self) -> ISSSonar:
        self.connect()
        self.setup_sonar()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.disconnect()
