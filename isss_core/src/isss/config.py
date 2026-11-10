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
    # Interferometer calibration: counts = K * sin(beta0 - depression).
    # K fitted against the Beckholmen dock mesh (sonar3d_reference, Oct 2026).
    # beta0 is the nominal 45 deg mount; that fit gave 44.5 for the stick mount.
    counts_per_unit_sine: float = 15218.0
    boresight_depression_deg: float = 45.0
    # Per side, counts = sign * K * sin(beta0 - depression). -1 for a transducer
    # mounted upside down (its two receivers swapped): returns mirror about the boresight.
    port_phase_sign: int = 1
    starboard_phase_sign: int = 1
    recv_timeout_sec: float = 2.0
    reconnect_interval_sec: float = 5.0
