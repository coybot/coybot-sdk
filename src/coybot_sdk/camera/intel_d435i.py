"""
Intel RealSense D435i camera implementation.

The D435i has:
- 1920x1080 RGB camera
- Stereo depth from two IR cameras (up to 1280x720)
- IMU (accelerometer + gyroscope)
- USB 3.0 connection

Install: ``pip install coybot-sdk[camera-realsense]``

See https://coy.bot/docs for details.
"""

from typing import Optional

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    raise ImportError(
        "pyrealsense2 library required. "
        "Install with: pip install coybot-sdk[camera-realsense]"
    )

from coybot_sdk.camera.base import Camera, CameraFrame


class RealSenseCamera(Camera):
    """Intel RealSense D435i camera implementation."""

    CAMERA_TYPE = "realsense"

    # Common resolutions supported by D435i
    SUPPORTED_RESOLUTIONS = [
        (1920, 1080),
        (1280, 720),
        (848, 480),
        (640, 480),
        (640, 360),
        (424, 240),
    ]

    def __init__(
        self,
        rgb_fps: int = 30,
        enable_depth: bool = True,
        rgb_resolution: tuple[int, int] = (1280, 720),
        depth_resolution: tuple[int, int] = (640, 480),
    ):
        """
        Initialize RealSense D435i camera.

        Args:
            rgb_fps: Frame rate for RGB camera.
            enable_depth: Whether to enable depth output.
            rgb_resolution: RGB resolution as (width, height). Defaults to 1280x720
                            for a good balance of quality and performance.
            depth_resolution: Depth resolution as (width, height).
        """
        self._rgb_fps = rgb_fps
        self._enable_depth = enable_depth
        self._rgb_resolution = rgb_resolution
        self._depth_resolution = depth_resolution

        self._pipeline: Optional[rs.pipeline] = None
        self._config: Optional[rs.config] = None
        self._align: Optional[rs.align] = None
        self._sequence_num = 0

    def start(self) -> None:
        """Start the camera pipeline."""
        if self._pipeline is not None:
            return

        pipeline = rs.pipeline()
        config = rs.config()

        rgb_started = False
        resolutions_to_try = [self._rgb_resolution] + [
            r for r in self.SUPPORTED_RESOLUTIONS if r != self._rgb_resolution
        ]

        for resolution in resolutions_to_try:
            try:
                config = rs.config()
                config.enable_stream(
                    rs.stream.color,
                    resolution[0],
                    resolution[1],
                    rs.format.rgb8,
                    self._rgb_fps,
                )

                align = None
                if self._enable_depth:
                    depth_res = self._depth_resolution
                    if depth_res[0] > resolution[0]:
                        depth_res = (resolution[0], resolution[1])

                    config.enable_stream(
                        rs.stream.depth,
                        depth_res[0],
                        depth_res[1],
                        rs.format.z16,
                        self._rgb_fps,
                    )
                    align = rs.align(rs.stream.color)

                pipeline.start(config)

                self._pipeline = pipeline
                self._config = config
                self._align = align
                self._rgb_resolution = resolution
                rgb_started = True
                print(f"RealSense started at {resolution[0]}x{resolution[1]}")
                break

            except RuntimeError as e:
                print(f"Failed at {resolution}: {e}")
                continue

        if not rgb_started:
            try:
                pipeline.stop()
            except Exception:
                pass
            raise RuntimeError("Could not start RealSense camera at any supported resolution")

        # Let auto-exposure settle
        for _ in range(30):
            self._pipeline.wait_for_frames()

    def stop(self) -> None:
        """Stop the camera pipeline and release resources."""
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        self._config = None
        self._align = None

    def get_frame(self, timeout_ms: int = 1000) -> Optional[CameraFrame]:
        """Get the next frame from the camera."""
        if self._pipeline is None:
            try:
                self.start()
            except Exception as e:
                print(f"Error starting camera pipeline: {e}")
                return None

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms)

            if not frames:
                return None

            if self._enable_depth and self._align:
                frames = self._align.process(frames)

            frame = CameraFrame()

            color_frame = frames.get_color_frame()
            if color_frame:
                frame.rgb = np.asanyarray(color_frame.get_data())
                frame.timestamp_ms = color_frame.get_timestamp()

            if self._enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    frame.depth = np.asanyarray(depth_frame.get_data())

            self._sequence_num += 1
            frame.sequence_num = self._sequence_num

            return frame

        except Exception as e:
            msg = str(e)
            print(f"Error getting frame: {msg}")
            if "before start()" in msg or "not started" in msg.lower():
                try:
                    self._pipeline = None
                    self._config = None
                    self._align = None

                    import time
                    time.sleep(0.5)

                    self.start()

                    if self._pipeline is None:
                        print("Failed to restart camera pipeline")
                        return None

                    frames = self._pipeline.wait_for_frames(timeout_ms)
                    if not frames:
                        return None
                    if self._enable_depth and self._align:
                        frames = self._align.process(frames)
                    frame = CameraFrame()
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        frame.rgb = np.asanyarray(color_frame.get_data())
                        frame.timestamp_ms = color_frame.get_timestamp()
                    if self._enable_depth:
                        depth_frame = frames.get_depth_frame()
                        if depth_frame:
                            frame.depth = np.asanyarray(depth_frame.get_data())
                    self._sequence_num += 1
                    frame.sequence_num = self._sequence_num
                    return frame
                except Exception as retry_err:
                    print(f"Error retrying frame capture: {retry_err}")
                    self._pipeline = None
                    self._config = None
                    self._align = None
            return None

    def is_connected(self) -> bool:
        """Check if camera is connected and operational."""
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            return len(devices) > 0
        except Exception:
            return False

    @property
    def resolution(self) -> tuple[int, int]:
        """Return (width, height) of the RGB camera."""
        return self._rgb_resolution

    @staticmethod
    def list_devices() -> list[dict]:
        """List all connected RealSense devices."""
        devices = []
        try:
            ctx = rs.context()
            for device in ctx.query_devices():
                devices.append({
                    "name": device.get_info(rs.camera_info.name),
                    "serial": device.get_info(rs.camera_info.serial_number),
                    "firmware": device.get_info(rs.camera_info.firmware_version),
                    "usb_type": device.get_info(rs.camera_info.usb_type_descriptor),
                })
        except Exception as e:
            print(f"Error listing devices: {e}")
        return devices
