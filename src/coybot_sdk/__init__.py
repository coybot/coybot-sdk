"""
coybot-sdk: Open Python SDK for ArduPilot drones.

This package provides MAVLink primitives, camera drivers, and utilities
for building drone applications. See https://coy.bot/docs for details.
"""

from coybot_sdk.drone import (
    # Connection
    set_config_path,
    disconnect,
    # Flight commands
    arm,
    disarm,
    takeoff,
    land,
    goto,
    set_velocity,
    set_yaw,
    wait,
    # Telemetry
    get_position,
    get_attitude,
    get_battery,
    get_flight_mode,
    get_telemetry,
    is_armed,
    # Setup
    motor_test,
    configure_battery_monitoring,
    configure_failsafes,
    setup_drone,
    # Camera
    capture_photo,
    release_camera,
    # Safety constants
    MAX_VELOCITY,
    MAX_ALTITUDE,
    MIN_ALTITUDE,
    MAX_YAW_RATE,
)

__version__ = "0.1.0"

__all__ = [
    "set_config_path",
    "disconnect",
    "arm",
    "disarm",
    "takeoff",
    "land",
    "goto",
    "set_velocity",
    "set_yaw",
    "wait",
    "get_position",
    "get_attitude",
    "get_battery",
    "get_flight_mode",
    "get_telemetry",
    "is_armed",
    "motor_test",
    "configure_battery_monitoring",
    "configure_failsafes",
    "setup_drone",
    "capture_photo",
    "release_camera",
    "MAX_VELOCITY",
    "MAX_ALTITUDE",
    "MIN_ALTITUDE",
    "MAX_YAW_RATE",
    "__version__",
]
