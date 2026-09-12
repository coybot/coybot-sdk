"""
Auto-detection and factory for camera implementations.

See https://coy.bot/docs for details.
"""

from typing import Optional

from coybot_sdk.camera.base import Camera


def _check_oakd_available() -> bool:
    """Check if OAK-D camera is available."""
    try:
        import depthai as dai
        devices = dai.Device.getAllAvailableDevices()
        return len(devices) > 0
    except Exception:
        return False


def _check_realsense_available() -> bool:
    """Check if RealSense camera is available."""
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        devices = ctx.query_devices()
        return len(devices) > 0
    except Exception:
        return False


def list_available_cameras() -> list[dict]:
    """
    List all available cameras.

    Returns:
        List of dicts with camera info: {"type": str, "name": str, "available": bool}
    """
    cameras = []

    # Check OAK-D
    try:
        import depthai as dai
        for device_info in dai.Device.getAllAvailableDevices():
            cameras.append({
                "type": "oakd",
                "name": f"OAK-D ({device_info.name})",
                "device_id": device_info.getDeviceId(),
                "available": True,
            })
    except Exception:
        pass

    # Check RealSense
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        for device in ctx.query_devices():
            cameras.append({
                "type": "realsense",
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "available": True,
            })
    except Exception:
        pass

    return cameras


def get_camera(
    preferred_type: Optional[str] = None,
    rgb_fps: int = 30,
    enable_depth: bool = True,
    rgb_resolution: tuple[int, int] = (1280, 720),
) -> Optional[Camera]:
    """
    Get a camera instance, auto-detecting the available hardware.

    Args:
        preferred_type: Preferred camera type ("oakd", "realsense"). If None, auto-detect.
        rgb_fps: Target frame rate for RGB camera.
        enable_depth: Whether to enable depth output.
        rgb_resolution: Desired RGB resolution as (width, height).

    Returns:
        Camera instance if available, None otherwise.
    """
    if preferred_type == "oakd":
        camera = _try_oakd(rgb_fps, enable_depth, rgb_resolution)
        if camera:
            return camera
    elif preferred_type == "realsense":
        camera = _try_realsense(rgb_fps, enable_depth, rgb_resolution)
        if camera:
            return camera

    # Auto-detect: try OAK-D first, then RealSense
    camera = _try_oakd(rgb_fps, enable_depth, rgb_resolution)
    if camera:
        return camera

    camera = _try_realsense(rgb_fps, enable_depth, rgb_resolution)
    if camera:
        return camera

    return None


def _try_oakd(rgb_fps: int, enable_depth: bool, rgb_resolution: tuple[int, int]) -> Optional[Camera]:
    """Try to create an OAK-D camera instance."""
    if not _check_oakd_available():
        return None

    try:
        from coybot_sdk.camera.oakdlite import OakDLiteCamera
        return OakDLiteCamera(
            rgb_fps=rgb_fps,
            enable_depth=enable_depth,
            rgb_resolution=rgb_resolution,
        )
    except Exception as e:
        print(f"Failed to initialize OAK-D camera: {e}")
        return None


def _try_realsense(rgb_fps: int, enable_depth: bool, rgb_resolution: tuple[int, int]) -> Optional[Camera]:
    """Try to create a RealSense camera instance."""
    if not _check_realsense_available():
        return None

    try:
        from coybot_sdk.camera.intel_d435i import RealSenseCamera
        # Cap fps at 30 for stability
        return RealSenseCamera(
            rgb_fps=min(rgb_fps, 30),
            enable_depth=enable_depth,
            rgb_resolution=rgb_resolution,
        )
    except Exception as e:
        print(f"Failed to initialize RealSense camera: {e}")
        return None
