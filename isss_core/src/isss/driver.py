"""Run the sonar continuously: read loop, reconnect, raw logging and statistics.

This is everything a driver needs except the middleware. The ROS 2 node in
``isss_ros2_driver`` only translates parameters, messages and services to and
from this class, so all behaviour here can be tested on a bench machine.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from isss.config import SonarConfig
from isss.log import BinLogWriter
from isss.protocol import Ping, compute_decimation, parse_packet
from isss.sonar import ISSSonar

PingCallback = Callable[[Ping], None]


class NotConnectedError(RuntimeError):
    """Raised when an operation needs a live connection to the sonar."""


@dataclass
class DriverStatus:
    """Snapshot of the driver state, published as status/diagnostics by wrappers."""

    connected: bool
    ping_rate_hz: float
    last_error: str
    range_m: float
    num_samples: int
    num_phases: int
    decimation: int
    logging: bool
    log_path: Optional[str]


class SonarDriver:
    """Keep a connection to the ISSS alive and hand every decoded ping to ``on_ping``.

    ``on_ping`` is called from the driver's read thread.
    ``logger`` may be a ``logging.Logger`` or a ROS logger; only ``info``,
    ``warning`` and ``error`` with a single string are used.
    """

    def __init__(
        self,
        config: SonarConfig,
        on_ping: PingCallback | None = None,
        *,
        log_dir: str | Path | None = None,
        logger: Any = None,
    ) -> None:
        self.config = config
        self.on_ping = on_ping
        self.log = logger or logging.getLogger(__name__)
        self._sonar = ISSSonar(config)
        self._log_writer = BinLogWriter(log_dir) if log_dir else None

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._connected = False
        self._last_error = ""
        self._ping_count_window = 0
        self._ping_window_start = time.monotonic()
        self._ping_rate_hz = 0.0
        self._last_register_writes: list[tuple[int, int]] = []

    # Lifecycle

    def start(self) -> None:
        """Connect (or schedule reconnects) and start the read thread."""
        if self._thread is not None:
            return
        self._stop.clear()
        if not self._connect_and_setup():
            self.log.warning("Initial sonar connection failed; read loop will attempt reconnect.")
        if self._log_writer is not None:
            self.start_logging()
        self._thread = threading.Thread(target=self._run, name="isss-driver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the read thread, close the connection and any log file."""
        self._stop.set()
        self._sonar.disconnect()
        if self._thread is not None:
            self._thread.join(timeout=self.config.recv_timeout_sec + 1.0)
            self._thread = None
        if self._log_writer is not None:
            self._log_writer.close()
        self._set_connected(False)

    def __enter__(self) -> SonarDriver:
        self.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()

    # Commands

    def configure(
        self,
        *,
        range_m: float | None = None,
        num_samples: int | None = None,
        num_phases: int | None = None,
    ) -> None:
        """Validate and apply a new configuration.

        The configuration is stored even when the sonar is disconnected, so it
        is applied on the next reconnect; ``NotConnectedError`` is raised in
        that case to tell the caller it has not reached the device yet.
        """
        new_range = self.config.range_m if range_m is None else range_m
        new_samples = self.config.num_samples if num_samples is None else num_samples
        new_phases = self.config.num_phases if num_phases is None else num_phases
        if new_range <= 0 or new_samples <= 0 or new_phases < 0:
            raise ValueError("Invalid configuration values")

        with self._lock:
            self.config.range_m = new_range
            self.config.num_samples = new_samples
            self.config.num_phases = new_phases

        if not self.is_connected:
            raise NotConnectedError("Sonar is not connected")
        try:
            self._note_setup(self._sonar.setup_sonar())
        except OSError as exc:
            self._set_error(f"Failed to write sonar configuration registers: {exc}")
            raise
        self.log.info(
            f"Sonar reconfigured: range={new_range:.1f}, "
            f"num_samples={new_samples}, num_phases={new_phases}"
        )

    def start_logging(self) -> Path:
        """Start a new raw .bin log; requires ``log_dir``."""
        if self._log_writer is None:
            raise RuntimeError("No log directory configured")
        path = self._log_writer.open(self.config, self._last_register_writes)
        self.log.info(f"Saving sonar data to: {path}")
        return path

    def stop_logging(self) -> None:
        if self._log_writer is not None:
            self._log_writer.close()

    # State

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def status(self) -> DriverStatus:
        with self._lock:
            return DriverStatus(
                connected=self._connected,
                ping_rate_hz=self._ping_rate_hz,
                last_error=self._last_error,
                range_m=self.config.range_m,
                num_samples=self.config.num_samples,
                num_phases=self.config.num_phases,
                decimation=compute_decimation(self.config.range_m, self.config.num_samples),
                logging=self._log_writer is not None and self._log_writer.is_open,
                log_path=(
                    str(self._log_writer.path)
                    if self._log_writer is not None and self._log_writer.is_open
                    else None
                ),
            )

    # Internals

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.is_connected and not self._reconnect():
                continue
            try:
                packet = self._sonar.read_packet()
            except (OSError, ConnectionError, RuntimeError) as exc:
                if self._stop.is_set():
                    break
                self._set_error(f"Receive error or timeout from sonar: {exc}")
                self._sonar.disconnect()
                self._set_connected(False)
                continue

            self._update_ping_rate()
            if self._log_writer is not None:
                self._log_writer.write(packet)
            if self.on_ping is not None:
                try:
                    self.on_ping(parse_packet(packet, self.config))
                except Exception as exc:  # a failing consumer must not kill the read loop
                    self.log.error(f"on_ping callback failed: {exc!r}")

    def _reconnect(self) -> bool:
        self.log.warning(
            f"Sonar disconnected; retrying in {self.config.reconnect_interval_sec:.1f} s"
        )
        if self._stop.wait(self.config.reconnect_interval_sec):
            return False
        return self._connect_and_setup()

    def _connect_and_setup(self) -> bool:
        try:
            self._sonar.connect()
        except OSError as exc:
            self._set_error(
                f"Failed to connect to sonar at {self.config.ip}:{self.config.port}: {exc}"
            )
            self._set_connected(False)
            return False
        try:
            self._note_setup(self._sonar.setup_sonar())
        except OSError as exc:
            self._set_error(f"Failed to configure sonar registers: {exc}")
            self._sonar.disconnect()
            self._set_connected(False)
            return False

        self._set_connected(True)
        self._set_error("")
        self.log.info(f"Connected to sonar at {self.config.ip}:{self.config.port}")
        return True

    def _note_setup(self, writes: list[tuple[int, int]]) -> None:
        self._last_register_writes = writes
        if self._log_writer is not None:
            self._log_writer.note_setup(self.config, writes)

    def _update_ping_rate(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._ping_count_window += 1
            elapsed = now - self._ping_window_start
            if elapsed >= 1.0:
                self._ping_rate_hz = self._ping_count_window / elapsed
                self._ping_count_window = 0
                self._ping_window_start = now

    def _set_connected(self, value: bool) -> None:
        with self._lock:
            self._connected = value

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message
