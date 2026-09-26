import Foundation
import CoreImage
import CoreVideo
import ImageIO
#if canImport(UIKit)
import UIKit
#endif

/// Downscales and JPEG-encodes a camera frame for sending to a cloud vision brain. Kept
/// small and lossy on purpose — `CloudBrain` sends this on every think-tick, not every
/// video frame, but bandwidth/latency still matter for a live mission.
public enum FrameEncoder {
    ///
    /// `orientation` rotates the raw buffer upright before encoding, so the brain sees the
    /// scene the right way up and the image points it returns are in the same upright frame
    /// `Detector` boxes and `ARSessionManager.unproject` use.
    public static func jpeg(_ pixelBuffer: CVPixelBuffer?, orientation: CGImagePropertyOrientation = .up,
                            maxDimension: CGFloat = 512, quality: CGFloat = 0.6) -> Data? {
        guard let pixelBuffer else { return nil }
        let image = CIImage(cvPixelBuffer: pixelBuffer).oriented(orientation)
        let longestSide = max(image.extent.width, image.extent.height)
        let scale = longestSide > maxDimension ? maxDimension / longestSide : 1
        let scaled = scale < 1 ? image.transformed(by: CGAffineTransform(scaleX: scale, y: scale)) : image

        guard let cgImage = CIContext().createCGImage(scaled, from: scaled.extent) else { return nil }
        #if canImport(UIKit)
        return UIImage(cgImage: cgImage).jpegData(compressionQuality: quality)
        #else
        return nil
        #endif
    }
}
