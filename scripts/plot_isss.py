#!/usr/bin/env python3
"""
Plot phase difference and SSS intensity from recorded ISSS binary logs.

Usage:
  ros2 run isss_ros2_driver plot_isss.py <filename.bin> [num_pings]
  python3 plot_isss.py <path/to/file.bin> [num_pings]
"""

import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np

BYTES_HEADER = 12
BYTES_PER_SAMPLE = 6


def parse_args():
    parser = argparse.ArgumentParser(description="Plot ISSS binary log data")
    parser.add_argument("file", help="Path to .bin log file")
    parser.add_argument(
        "num_pings",
        nargs="?",
        type=int,
        default=1000,
        help="Number of pings to plot (default: 1000)")
    return parser.parse_args()


def find_packet_indices(data, bytes_per_ping, max_index):
    indices = []
    for i in range(max_index):
        if (
            data[i] == 0xFE and
            data[i + 1] == 0x05 and
            data[i + 4] == 0xE1
        ):
            indices.append(i)
    return indices


def main():
    args = parse_args()

    with open(args.file, mode="rb") as log_file:
        data = log_file.read()

    num_samples = 1000
    bytes_per_ping = BYTES_HEADER + BYTES_PER_SAMPLE * num_samples
    total_nbr_pings = len(data) // bytes_per_ping
    nbr_pings = min(args.num_pings, total_nbr_pings)

    print(f"total_nbr_pings: {total_nbr_pings}")
    print(f"plotting: {nbr_pings} pings")

    index_list = find_packet_indices(data, bytes_per_ping, bytes_per_ping * nbr_pings)
    if not index_list:
        print("No valid ISSS packets found", file=sys.stderr)
        sys.exit(1)

    print(f"found packets: {len(index_list)}")

    sss_port = []
    sss_stbd = []
    sss_phase_port = []
    sss_phase_stbd = []
    sss_phase_port_hi = []
    sss_phase_stbd_hi = []

    for i in index_list[:nbr_pings]:
        sonar_data = data[i + BYTES_HEADER:i + BYTES_HEADER + BYTES_PER_SAMPLE * num_samples]
        sss_port.append(list(sonar_data[2::6]))
        sss_phase_port.append(list(sonar_data[1::6]))
        sss_phase_port_hi.append(list(sonar_data[0::6]))
        sss_stbd.append(list(sonar_data[5::6]))
        sss_phase_stbd.append(list(sonar_data[4::6]))
        sss_phase_stbd_hi.append(list(sonar_data[3::6]))

    sss_port = np.array(sss_port)
    sss_stbd = np.array(sss_stbd)
    sss_phase_port_hi = np.array(sss_phase_port_hi)
    sss_phase_stbd_hi = np.array(sss_phase_stbd_hi)
    sss_phase_port = np.array(sss_phase_port)
    sss_phase_stbd = np.array(sss_phase_stbd)

    phase_port = 256 * sss_phase_port_hi + sss_phase_port
    phase_stbd = 256 * sss_phase_stbd_hi + sss_phase_stbd

    idx = min(10, len(sss_port) - 1)

    fig, (int_port, int_stbd) = plt.subplots(1, 2)
    int_port.imshow(sss_port, cmap="gray", vmax=sss_port.max())
    int_port.set_title("Port intensity")
    int_stbd.imshow(sss_stbd, cmap="gray", vmax=sss_stbd.max())
    int_stbd.set_title("Starboard intensity")

    fig, (ax_phase_port, ax_phase_stbd) = plt.subplots(1, 2, sharey=True)
    ax_phase_port.scatter(np.arange(num_samples), phase_port[idx], s=1)
    ax_phase_port.set_title("Port phase")
    ax_phase_stbd.scatter(np.arange(num_samples), phase_stbd[idx], s=1)
    ax_phase_stbd.set_title("Starboard phase")

    plt.show()


if __name__ == "__main__":
    main()
