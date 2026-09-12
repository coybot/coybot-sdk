#!/usr/bin/env python3
"""
Standalone arm/disarm utility for ArduPilot drones via MAVLink.

This is a simple CLI script that connects directly to the flight controller
and arms or disarms the drone, without going through the full SDK.

See https://coy.bot/docs for details.

Usage:
    python -m coybot_sdk.arm_disarm arm
    python -m coybot_sdk.arm_disarm disarm
    python -m coybot_sdk.arm_disarm status
"""

import argparse
import time

from pymavlink import mavutil


def get_status(master):
    """Get current armed status and flight mode."""
    master.mav.request_data_stream_send(
        1, 1,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
    )

    start = time.time()
    while time.time() - start < 3:
        msg = master.recv_msg()
        if msg and msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == 1:
            armed = bool(msg.base_mode & 128)
            mode = msg.custom_mode

            # ArduCopter flight modes
            modes = {
                0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
                4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
                9: "LAND", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
                19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
            }
            mode_name = modes.get(mode, f"MODE_{mode}")

            return armed, mode_name
        time.sleep(0.05)

    return None, None


def arm(master, force=True):
    """Arm the drone."""
    print("Sending ARM command...")

    # MAV_CMD_COMPONENT_ARM_DISARM = 400
    # param1: 1 = arm; param2: 0 = normal, 21196 = force
    force_param = 21196 if force else 0

    master.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        force_param,
        0, 0, 0, 0, 0,
    )

    start = time.time()
    while time.time() - start < 5:
        msg = master.recv_msg()
        if msg:
            t = msg.get_type()
            if t == "COMMAND_ACK" and msg.command == 400:
                if msg.result == 0:
                    print("ARM command accepted!")
                    return True
                else:
                    results = {
                        1: "TEMPORARILY_REJECTED",
                        2: "DENIED",
                        3: "UNSUPPORTED",
                        4: "FAILED",
                    }
                    print(f"ARM failed: {results.get(msg.result, msg.result)}")
                    return False
            elif t == "STATUSTEXT":
                print(f"   Status: {msg.text}")
        time.sleep(0.02)

    print("No ACK received")
    return False


def disarm(master, force=True):
    """Disarm the drone."""
    print("Sending DISARM command...")

    force_param = 21196 if force else 0

    master.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        force_param,
        0, 0, 0, 0, 0,
    )

    start = time.time()
    while time.time() - start < 5:
        msg = master.recv_msg()
        if msg:
            t = msg.get_type()
            if t == "COMMAND_ACK" and msg.command == 400:
                if msg.result == 0:
                    print("DISARM command accepted!")
                    return True
                else:
                    print(f"DISARM failed: result={msg.result}")
                    return False
            elif t == "STATUSTEXT":
                print(f"   Status: {msg.text}")
        time.sleep(0.02)

    print("No ACK received")
    return False


def main():
    parser = argparse.ArgumentParser(description="Arm or disarm drone via MAVLink")
    parser.add_argument("action", choices=["arm", "disarm", "status"],
                        help="Action to perform")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--force", action="store_true",
                        help="Force arm/disarm (bypass pre-arm checks)")

    args = parser.parse_args()

    print(f"Connecting to {args.port}...")
    master = mavutil.mavlink_connection(args.port, baud=args.baud, source_system=255)

    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print(f"Connected to System {master.target_system}")
    print()

    armed, mode = get_status(master)
    if armed is not None:
        status = "ARMED" if armed else "DISARMED"
        print(f"Current status: {status}")
        print(f"Flight mode: {mode}")
        print()

    if args.action == "status":
        return

    if args.action == "arm":
        if armed:
            print("Already armed!")
            return
        print("ARMING IN 3 SECONDS - STAND CLEAR!")
        time.sleep(3)
        success = arm(master, force=args.force)

        if success:
            time.sleep(1)
            armed, _ = get_status(master)
            if armed:
                print()
                print("DRONE IS NOW ARMED")
                print("   Motors will spin if throttle is raised.")

    elif args.action == "disarm":
        if not armed:
            print("Already disarmed!")
            return
        success = disarm(master, force=args.force)

        if success:
            time.sleep(1)
            armed, _ = get_status(master)
            if not armed:
                print()
                print("DRONE IS NOW DISARMED")


if __name__ == "__main__":
    main()
