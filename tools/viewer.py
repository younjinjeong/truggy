#!/usr/bin/env python3
"""
TruggyAD Telemetry Viewer

Reads binary telemetry log files produced by `truggy --log <file>`
and displays trajectory, control history, and state plots.

Usage:
    python3 tools/viewer.py <telemetry.bin>
    python3 tools/viewer.py <telemetry.bin> --live  # real-time tail mode
"""

import sys
import struct
import numpy as np

# Log entry format: see src/common/logger.h
# uint64_t timestamp_us
# 7 floats: x, y, yaw, roll, u_x, u_y, yaw_rate
# 2 floats: steering, throttle
# 3 floats: perception_ms, planning_ms, state_ms
HEADER_SIZE = 12  # magic(4) + version(4) + entry_size(4)
ENTRY_FORMAT = '<Q10f3f'  # little-endian: uint64 + 13 floats... wait, let me recalculate
# Actually: uint64_t(8) + 7*float(28) + 2*float(8) + 3*float(12) = 56 bytes
ENTRY_FORMAT = '<Q7f2f3f'
ENTRY_SIZE = struct.calcsize(ENTRY_FORMAT)

FIELD_NAMES = [
    'timestamp_us',
    'x', 'y', 'yaw', 'roll', 'u_x', 'u_y', 'yaw_rate',
    'steering', 'throttle',
    'perception_ms', 'planning_ms', 'state_ms'
]


def read_log(path):
    """Read binary telemetry log, return list of dicts."""
    entries = []
    with open(path, 'rb') as f:
        # Read header
        header = f.read(HEADER_SIZE)
        if len(header) < HEADER_SIZE:
            print(f"Error: file too small ({len(header)} bytes)")
            return entries

        magic, version, entry_size = struct.unpack('<III', header)
        if magic != 0x54524731:
            print(f"Error: bad magic 0x{magic:08X} (expected 0x54524731)")
            return entries
        if entry_size != ENTRY_SIZE:
            print(f"Warning: entry size mismatch (file={entry_size}, expected={ENTRY_SIZE})")

        # Read entries
        while True:
            data = f.read(ENTRY_SIZE)
            if len(data) < ENTRY_SIZE:
                break
            values = struct.unpack(ENTRY_FORMAT, data)
            entry = dict(zip(FIELD_NAMES, values))
            entries.append(entry)

    return entries


def plot_telemetry(entries):
    """Plot trajectory, controls, and state."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed. Install with: pip3 install matplotlib")
        sys.exit(1)

    if not entries:
        print("No entries to plot")
        return

    # Convert to numpy arrays
    t = np.array([(e['timestamp_us'] - entries[0]['timestamp_us']) / 1e6
                  for e in entries])
    x = np.array([e['x'] for e in entries])
    y = np.array([e['y'] for e in entries])
    yaw = np.array([e['yaw'] for e in entries])
    u_x = np.array([e['u_x'] for e in entries])
    u_y = np.array([e['u_y'] for e in entries])
    yaw_rate = np.array([e['yaw_rate'] for e in entries])
    steering = np.array([e['steering'] for e in entries])
    throttle = np.array([e['throttle'] for e in entries])

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'TruggyAD Telemetry ({len(entries)} samples, {t[-1]:.1f}s)')

    # 1. Trajectory (bird's eye view)
    ax = axes[0, 0]
    ax.plot(x, y, 'b-', linewidth=0.8)
    ax.plot(x[0], y[0], 'go', markersize=10, label='Start')
    ax.plot(x[-1], y[-1], 'r^', markersize=10, label='End')
    # Draw heading arrows every N points
    step = max(1, len(x) // 30)
    for i in range(0, len(x), step):
        dx = 0.3 * np.cos(yaw[i])
        dy = 0.3 * np.sin(yaw[i])
        ax.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.05,
                 fc='gray', ec='gray', alpha=0.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory (BEV)')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. Control commands
    ax = axes[0, 1]
    ax.plot(t, steering, 'b-', linewidth=0.8, label='Steering')
    ax.plot(t, throttle, 'r-', linewidth=0.8, label='Throttle')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Command [-1, 1]')
    ax.set_title('Control Commands')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-1.1, 1.1)

    # 3. Velocity
    ax = axes[1, 0]
    speed = np.sqrt(u_x**2 + u_y**2)
    ax.plot(t, u_x, 'b-', linewidth=0.8, label='u_x (forward)')
    ax.plot(t, u_y, 'r-', linewidth=0.8, label='u_y (lateral)')
    ax.plot(t, speed, 'k--', linewidth=0.8, label='|speed|')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Body-Frame Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Yaw and yaw rate
    ax = axes[1, 1]
    ax.plot(t, np.degrees(yaw), 'b-', linewidth=0.8, label='Yaw (deg)')
    ax2 = ax.twinx()
    ax2.plot(t, np.degrees(yaw_rate), 'r-', linewidth=0.8, alpha=0.7,
             label='Yaw rate (deg/s)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (deg)', color='b')
    ax2.set_ylabel('Yaw rate (deg/s)', color='r')
    ax.set_title('Heading')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def print_summary(entries):
    """Print quick summary stats."""
    if not entries:
        print("No entries")
        return

    duration = (entries[-1]['timestamp_us'] - entries[0]['timestamp_us']) / 1e6
    speeds = [np.sqrt(e['u_x']**2 + e['u_y']**2) for e in entries]
    print(f"  Entries:   {len(entries)}")
    print(f"  Duration:  {duration:.1f} s")
    print(f"  Rate:      {len(entries)/duration:.1f} Hz")
    print(f"  Speed:     avg={np.mean(speeds):.2f} max={np.max(speeds):.2f} m/s")
    print(f"  Position:  start=({entries[0]['x']:.2f}, {entries[0]['y']:.2f}) "
          f"end=({entries[-1]['x']:.2f}, {entries[-1]['y']:.2f})")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    path = sys.argv[1]
    print(f"Reading {path}...")
    entries = read_log(path)

    if entries:
        print_summary(entries)
        plot_telemetry(entries)
    else:
        print("No valid entries found")
