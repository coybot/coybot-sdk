"""
Abstract base class and frame container for camera implementations.

See https://coy.bot/docs for details.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class CameraFrame:
    """Container for camera frame data."""

    rgb: Optional[np.ndarray] = None         # RGB image (H, W, 3)
    depth: Optional[np.ndarray] = None       # Depth map in mm (H, W)
    left_mono: Optional[np.ndarray] = None   # Left mono camera (H, W)
    right_mono: Optional[np.ndarray] = None  # Right mono camera (H, W)
    timestamp_ms: float = 0.0
    sequence_num: int = 0


class Camera(ABC):
    """Abstract base class for all camera implementations."""

    # Camera type identifier (set by subclasses)
    CAMERA_TYPE: str = "unknown"

    @abstractmethod
    def start(self) -> None:
        """Start the camera pipeline."""

    @abstractmethod
    def stop(self) -> None:
        """Stop the camera pipeline and release resources."""

    @abstractmethod
    def get_frame(self, timeout_ms: int = 1000) -> Optional[CameraFrame]:
        """
        Get the next frame from the camera.

        Args:
            timeout_ms: Maximum time to wait for a frame, in milliseconds.

        Returns:
            CameraFrame if successful, None if timeout or error.
        """

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if camera is connected and operational."""

    @property
    @abstractmethod
    def resolution(self) -> tuple[int, int]:
        """Return (width, height) of the RGB camera."""

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False
