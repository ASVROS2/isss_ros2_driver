#!/usr/bin/env python3
"""Example: connect to the ISSS and print ping metadata."""

from isss import ISSSonar, SonarConfig

config = SonarConfig(
    ip="192.168.2.62",
    range_m=20.0,
    num_samples=1000,
)

with ISSSonar(config) as sonar:
    for i in range(5):
        samples, packet = sonar.read_ping()
        print(
            f"ping {i}: {len(packet)} bytes, "
            f"port max={max(samples.port_rss)}, stbd max={max(samples.starboard_rss)}"
        )
