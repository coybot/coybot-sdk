#!/usr/bin/env python3
"""
Motor test example.

Spins each motor (1-4) one at a time at low throttle to verify motor order
and ESC connections.

SAFETY: Always remove props before running this.

Usage:
    python motor_test.py [--throttle 20] [--duration 2]
"""

import argparse

from coybot_sdk.motor_test import run_motor_test


def main():
    parser = argparse.ArgumentParser(description="Test drone motors via MAVLink")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--throttle", type=int, default=20, help="Throttle percentage (1-100)")
    parser.add_argument("--duration", type=int, default=2, help="Duration per motor in seconds")
    args = parser.parse_args()

    run_motor_test(
        port=args.port,
        baud=args.baud,
        throttle=args.throttle,
        duration=args.duration,
    )


if __name__ == "__main__":
    main()
