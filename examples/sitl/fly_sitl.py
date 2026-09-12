#!/usr/bin/env python3
"""
Minimal arm/takeoff/hover/land against ArduPilot SITL.

Prereq: SITL running and listening on tcp:127.0.0.1:5760, e.g.

    sim_vehicle.py -v ArduCopter --console --map

Then in another shell:

    export COYBOT_SDK_SERIAL_PORT=tcp:127.0.0.1:5760
    python fly_sitl.py

The SDK calls below are identical to what you'd run on real hardware.
The only thing that changes is the connection URL.
"""

import os
import time

import coybot_sdk as drone


def main():
    if "COYBOT_SDK_SERIAL_PORT" not in os.environ:
        print(
            "Set COYBOT_SDK_SERIAL_PORT to your SITL endpoint, e.g. "
            "export COYBOT_SDK_SERIAL_PORT=tcp:127.0.0.1:5760"
        )
        return

    print("Connecting and taking off in SITL...")
    if not drone.takeoff(2.0):
        print("Takeoff failed.")
        return

    print("Hovering for 5 seconds...")
    time.sleep(5)

    print("Landing...")
    drone.land()

    print("Done.")
    drone.disconnect()


if __name__ == "__main__":
    main()
