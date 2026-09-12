#!/usr/bin/env python3
"""
Standalone motor test utility for ArduPilot drones via MAVLink.

Spins each motor (1-4) one at a time at low throttle to verify motor order
and ESC connections. Use this BEFORE flying a new build.

SAFETY: Always remove props before running this.

See https://coy.bot/docs for details.

Usage:
    python -m coybot_sdk.motor_test [--throttle 20] [--duration 2]
"""

import argparse
import time

from pymavlink import mavutil


def run_motor_test(port="/dev/ttyACM0", baud=115200, throttle=20, duration=2):
    """Run motor test on all 4 motors sequentially."""

    print(f"Connecting to {port} at {baud} baud...")
    master = mavutil.mavlink_connection(port, baud=baud, source_system=255)

    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print(f"Connected! System {master.target_system}, Component {master.target_component}")

    print()
    print("MOTOR TEST IN 3 SECONDS - STAND CLEAR!")
    print(f"    Throttle: {throttle}%")
    print(f"    Duration: {duration} seconds per motor")
    print()
    time.sleep(3)

    for motor in range(1, 5):
        print(f">>> Motor {motor} <<<", end=" ", flush=True)

        # MAV_CMD_DO_MOTOR_TEST = 209
        master.mav.command_long_send(
            1,           # target_system
            1,           # target_component
            209,         # command (DO_MOTOR_TEST)
            0,           # confirmation
            motor,       # param1: motor instance (1-indexed)
            0,           # param2: throttle type (0=percent)
            throttle,    # param3: throttle value
            duration,    # param4: timeout in seconds
            1,           # param5: motor count
            0,           # param6: test order
            0,           # param7: unused
        )

        start = time.time()
        result = "sent"
        while time.time() - start < 2:
            msg = master.recv_msg()
            if msg:
                t = msg.get_type()
                if t == "COMMAND_ACK" and msg.command == 209:
                    if msg.result == 0:
                        result = "spinning"
                    else:
                        result = f"failed ({msg.result})"
                    break
                elif t == "STATUSTEXT" and "Motor" in str(msg.text):
                    result = f"{msg.text}"
                    break
            time.sleep(0.01)

        print(result)
        time.sleep(duration + 1)

    print()
    print("Motor test complete.")


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
