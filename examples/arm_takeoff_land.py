#!/usr/bin/env python3
"""
Minimal arm/takeoff/hover/land example.

This script connects to the flight controller, takes off to a low altitude,
hovers briefly, and lands.

SAFETY: Run this somewhere with plenty of clearance, GPS lock, and a clear
plan to manually take control if anything goes wrong. The SDK clamps
altitude to MIN_ALTITUDE..MAX_ALTITUDE, but it cannot save you from a bad
takeoff site.
"""

import time

import coybot_sdk as drone


def main():
    print("Connecting and taking off...")
    if not drone.takeoff(2.0):  # 2 meters AGL
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
