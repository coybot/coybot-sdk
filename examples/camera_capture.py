#!/usr/bin/env python3
"""
Capture a single frame from an attached camera (OAK-D Lite or RealSense
D435i) and save it to disk.

Install the camera dependencies first:
    pip install coybot-sdk[all]
"""

import argparse

import cv2

from coybot_sdk.camera import get_camera, list_available_cameras


def main():
    parser = argparse.ArgumentParser(description="Capture a frame from the drone camera")
    parser.add_argument("--output", default="capture.jpg", help="Output JPEG path")
    parser.add_argument("--no-depth", action="store_true", help="Disable depth stream")
    args = parser.parse_args()

    available = list_available_cameras()
    if not available:
        print("No supported cameras detected. Check USB connection.")
        return
    print(f"Available cameras: {[c['name'] for c in available]}")

    camera = get_camera(rgb_fps=15, enable_depth=not args.no_depth)
    if camera is None:
        print("Failed to initialize camera.")
        return

    with camera:
        # Warm up
        for _ in range(5):
            camera.get_frame(timeout_ms=500)

        frame = camera.get_frame(timeout_ms=2000)
        if frame is None or frame.rgb is None:
            print("Failed to capture frame.")
            return

        bgr = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(args.output, bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(f"Saved {args.output} ({frame.rgb.shape[1]}x{frame.rgb.shape[0]})")

        if frame.depth is not None:
            print(f"Depth frame: {frame.depth.shape}")


if __name__ == "__main__":
    main()
