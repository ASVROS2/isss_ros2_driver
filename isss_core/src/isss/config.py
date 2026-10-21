"""Sonar configuration."""

from dataclasses import dataclass


@dataclass
class SonarConfig:
    """Connection and interferometry parameters for the ISSS."""

    ip: str = "192.168.2.62"
    port: int = 57349
    range_m: float = 20.0
    num_samples: int = 1000
    num_phases: int = 2
    sound_velocity: float = 1440.9
    carrier_frequency: float = 12.5e6
    baseline: float = 3.2e-3
    recv_timeout_sec: float = 2.0
    reconnect_interval_sec: float = 5.0
