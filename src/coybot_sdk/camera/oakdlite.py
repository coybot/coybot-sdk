"""
OAK-D Lite camera implementation using DepthAI 3.x.

The OAK-D Lite has:
- 4K RGB camera (IMX214)
- Stereo depth from two OV7251 mono cameras
- Intel Movidius Myriad X VPU for on-device AI

Install: ``pip install coybot-sdk[camera-oak]``

See https://coy.bot/docs for details.
"""

from typing import Optional

try:
    import depthai as dai
except ImportError:
    raise ImportError(
        "depthai library required. "
        "Install with: pip install coybot-sdk[camera-oak]"
    )

from coybot_sdk.camera.base import Camera, CameraFrame


class OakDLiteCamera(Camera):
    """OAK-D Lite camera implementation for DepthAI 3.x."""

    CAMERA_TYPE = "oakd"

    def __init__(
        self,
        rgb_fps: int = 30,
        enable_depth: bool = True,
        rgb_resolution: tuple[int, int] = (1920, 1080),
        depth_resolution: tuple[int, int] = (640, 400),
    ):
        """
        Initialize OAK-D Lite camera.

        Args:
            rgb_fps: Frame rate for RGB camera.
            enable_depth: Whether to enable depth output.
            rgb_resolution: RGB resolution as (width, height).
            depth_resolution: Depth resolution as (width, height).
        """
        self._rgb_fps = rgb_fps
        self._enable_depth = enable_depth
        self._rgb_resolution = rgb_resolution
        self._depth_resolution = depth_resolution

        self._pipeline: Optional[dai.Pipeline] = None
        self._rgb_queue = None
        self._depth_queue = None
        self._sequence_num = 0

    def start(self) -> None:
        """Start the camera pipeline."""
        if self._pipeline is not None:
            return

        self._pipeline = dai.Pipeline()

        cam_rgb = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        rgb_out = cam_rgb.requestOutput(self._rgb_resolution)
        self._rgb_queue = rgb_out.createOutputQueue()

        if self._enable_depth:
            stereo = self._pipeline.create(dai.node.StereoDepth).build(
                autoCreateCameras=True,
                size=self._depth_resolution,
            )
            self._depth_queue = stereo.depth.createOutputQueue()

        self._pipeline.start()

    def stop(self) -> None:
        """Stop the camera pipeline and release resources."""
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        self._rgb_queue = None
        self._depth_queue = None

    def get_frame(self, timeout_ms: int = 1000) -> Optional[CameraFrame]:
        """Get the next frame from the camera."""
        if self._pipeline is None:
            return None

        try:
            frame = CameraFrame()

            if self._rgb_queue:
                rgb_msg = self._rgb_queue.get()
                if rgb_msg:
                    frame.rgb = rgb_msg.getCvFrame()
                    ts = rgb_msg.getTimestamp()
                    frame.timestamp_ms = ts.total_seconds() * 1000

            if self._enable_depth and self._depth_queue:
                depth_msg = self._depth_queue.tryGet()
                if depth_msg:
                    frame.depth = depth_msg.getCvFrame()

            self._sequence_num += 1
            frame.sequence_num = self._sequence_num

            return frame

        except Exception as e:
            print(f"Error getting frame: {e}")
            return None

    def is_connected(self) -> bool:
        """Check if camera is connected and operational."""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0
        except Exception:
            return False

    @property
    def resolution(self) -> tuple[int, int]:
        """Return (width, height) of the RGB camera."""
        return self._rgb_resolution

    @staticmethod
    def list_devices() -> list[dict]:
        """List all connected OAK devices."""
        devices = []
        for device_info in dai.Device.getAllAvailableDevices():
            devices.append({
                "name": device_info.name,
                "device_id": device_info.getDeviceId(),
                "state": str(device_info.state),
            })
        return devices
