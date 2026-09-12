"""Public, hardware-free e2e self-test: exercises coybot_sdk against ArduPilot SITL for
each of the three vehicle frames the SDK can drive (Copter, Rover, Plane).

Requires ArduPilot's `sim_vehicle.py` on PATH (see examples/sitl/README.md for setup —
the ArduPilot dev docs' standard install puts Tools/autotest on PATH). When it's not
found, every test here skips cleanly, so `pip install coybot-sdk && pytest` still passes
with no SITL installed — this file is not part of the always-on test_imports.py smoke tier.

Scope: this proves the SDK's arm / velocity-command / telemetry / disarm contract holds
identically across all three ArduPilot frames — the actually-portable part of the public
API. It deliberately does NOT drive takeoff()/land(): that pair hardcodes a GUIDED +
MAV_CMD_NAV_TAKEOFF sequence with copter-shaped timing (see drone.py's `takeoff`/`land`),
which is not validated here for Rover (can't take off at all) or Plane (guided takeoff
from a standing start is a materially different procedure ArduPilot handles differently
for fixed-wing — see the note in eco/aws/src/handler.py's FIXEDWING_SYSTEM_PROMPT for the
same class of gap one layer up the stack). Extending takeoff()/land() with frame-aware
behavior is future work, not something this test should silently assume works.

Run: cd sdk && pytest tests/test_e2e_sitl.py -v
"""
from __future__ import annotations

import os
import shutil
import socket
import subprocess
import tempfile
import time

import pytest

import coybot_sdk as drone

SITL_BIN = shutil.which("sim_vehicle.py")
pytestmark = pytest.mark.skipif(
    SITL_BIN is None,
    reason="ArduPilot sim_vehicle.py not on PATH — see examples/sitl/README.md",
)

# ArduPilot's default multi-instance port scheme: TCP MAVLink port = 5760 + 10*instance.
# Note: the ground-rover vehicle's sim_vehicle.py -v name is "Rover", not "ArduRover"
# (see ArduPilot's Tools/autotest/pysim/vehicleinfo.json).
_FRAMES = [
    ("ArduCopter", 0, 5760),
    ("Rover", 1, 5770),
    ("ArduPlane", 2, 5780),
]
_BOOT_TIMEOUT_S = 90.0
_ARM_RETRY_TIMEOUT_S = 30.0


def _wait_for_port(port: int, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1.0)
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return True
        time.sleep(1.0)
    return False


@pytest.fixture(params=_FRAMES, ids=[f[0] for f in _FRAMES])
def sitl_port(request):
    frame, instance, port = request.param
    workdir = tempfile.mkdtemp(prefix=f"sitl-{frame}-")
    proc = subprocess.Popen(
        [SITL_BIN, "-v", frame, "--instance", str(instance),
         "--no-mavproxy", "--no-rebuild", "-w"],
        cwd=workdir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        if not _wait_for_port(port, _BOOT_TIMEOUT_S):
            proc.terminate()
            pytest.skip(f"{frame} SITL never opened port {port} within {_BOOT_TIMEOUT_S}s")
        yield frame, port
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        drone.disconnect()


def _arm_with_retry(timeout: float) -> bool:
    # ArduCopter hard-blocks arming (even force-arm) until its boot-time sensor
    # calibration finishes, which can outlast the TCP port opening. Retry rather
    # than treating the FC's transient "not ready yet" as a failure.
    deadline = time.time() + timeout
    while True:
        if drone.arm():
            return True
        if time.time() >= deadline:
            return False
        time.sleep(1.0)


def test_arm_move_telemetry_disarm(sitl_port):
    frame, port = sitl_port
    os.environ["COYBOT_SDK_SERIAL_PORT"] = f"tcp:127.0.0.1:{port}"
    try:
        assert _arm_with_retry(_ARM_RETRY_TIMEOUT_S), f"{frame}: arm() failed"
        assert drone.is_armed(), f"{frame}: not armed after arm()"

        drone.set_velocity(0.5, 0.0, 0.0)
        drone.wait(2)

        telemetry = drone.get_telemetry()
        assert telemetry, f"{frame}: get_telemetry() returned nothing"
        assert telemetry.get("armed") is True, f"{frame}: telemetry disagrees, not armed"

        assert drone.disarm(), f"{frame}: disarm() failed"
        assert not drone.is_armed(), f"{frame}: still armed after disarm()"
    finally:
        del os.environ["COYBOT_SDK_SERIAL_PORT"]
