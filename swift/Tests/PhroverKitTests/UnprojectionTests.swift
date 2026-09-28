import XCTest
import UIKit
import CoreImage
import CoreVideo
import simd
import RoverNav
@testable import PhroverKit

/// `ARCamera`/`ARDepthData` have no public initializers, so ARKit can never hand a test a
/// real one — these tests exercise the pure-math helpers `ARSessionManager` factors its
/// `unproject(normalizedPoint:)` through (`sensorPixel`/`sampleDepth`/`unprojectPoint`)
/// against synthetic intrinsics/transforms/depth instead. Runs on the iOS Simulator.
@MainActor
final class UnprojectionTests: XCTestCase {
    func testSensorPixelHandlesCorners() {
        let imageSize = CGSize(width: 1920, height: 1440) // raw (landscape) sensor space

        let bottomLeft = ARSessionManager.sensorPixel(forVisionNormalizedPoint: .zero, imageSize: imageSize)
        XCTAssertEqual(bottomLeft.x, 1920, accuracy: 0.01)
        XCTAssertEqual(bottomLeft.y, 1440, accuracy: 0.01)

        let topRight = ARSessionManager.sensorPixel(forVisionNormalizedPoint: CGPoint(x: 1, y: 1), imageSize: imageSize)
        XCTAssertEqual(topRight.x, 0, accuracy: 0.01)
        XCTAssertEqual(topRight.y, 0, accuracy: 0.01)
    }

    func testSampleDepthReadsConstantValue() {
        let imageSize = CGSize(width: 100, height: 100)
        let depthMap = Self.makeDepthBuffer(width: 20, height: 20, constantDepth: 2.5)

        let depth = ARSessionManager.sampleDepth(depthMap, atVisionNormalizedPoint: CGPoint(x: 0.5, y: 0.5), imageSize: imageSize)

        XCTAssertEqual(depth, 2.5)
    }

    func testSampleDepthRejectsInvalidReadings() {
        let imageSize = CGSize(width: 100, height: 100)
        let depthMap = Self.makeDepthBuffer(width: 20, height: 20, constantDepth: 0)

        XCTAssertNil(ARSessionManager.sampleDepth(depthMap, atVisionNormalizedPoint: CGPoint(x: 0.5, y: 0.5), imageSize: imageSize))
    }

    func testForwardClearanceCatchesOffCenterObstacleInDrivingCorridor() {
        let depthMap = Self.makeDepthBuffer(width: 20, height: 20, constantDepth: 3.5)
        for y in 9...11 {
            for x in 12...14 {
                Self.setDepth(0.35, x: x, y: y, in: depthMap)
            }
        }

        let clearance = ARSessionManager.forwardClearance(fromDepthMap: depthMap)

        XCTAssertEqual(clearance, 0.35, accuracy: 0.001)
    }

    func testForwardClearanceIgnoresSingleNearOutlierInDrivingCorridor() {
        let depthMap = Self.makeDepthBuffer(width: 20, height: 20, constantDepth: 3.5)
        Self.setDepth(0.35, x: 10, y: 10, in: depthMap)

        let clearance = ARSessionManager.forwardClearance(fromDepthMap: depthMap)

        XCTAssertEqual(clearance, 3.5, accuracy: 0.001)
    }

    func testUnprojectPointAtIdentityTransform() {
        // fx = fy = 500, principal point (cx, cy) = (400, 300), raw sensor 600x800.
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(500, 0, 0),
            SIMD3<Float>(0, 500, 0),
            SIMD3<Float>(400, 300, 1)
        ))
        let identity = simd_float4x4(1) // camera at world origin, no rotation, looking down -Z
        let imageSize = CGSize(width: 600, height: 800)
        // The normalized point whose sensor-space position is exactly the principal point.
        let normalized = CGPoint(x: 1 - 300.0 / 800.0, y: 1 - 400.0 / 600.0)

        let goal = ARSessionManager.unprojectPoint(normalized, imageSize: imageSize,
                                                    intrinsics: intrinsics, cameraTransform: identity, depth: 3.0)

        // A ray through the principal point has no lateral offset: straight down -Z at
        // depth 3 from the origin flattens to nav-plane (world x, world z) = (0, -3).
        XCTAssertEqual(goal.x, 0, accuracy: 0.01)
        XCTAssertEqual(goal.y, -3, accuracy: 0.01)
    }

    func testUnprojectPointTranslatesWithCameraPose() {
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(500, 0, 0),
            SIMD3<Float>(0, 500, 0),
            SIMD3<Float>(400, 300, 1)
        ))
        var transform = simd_float4x4(1)
        transform.columns.3 = SIMD4<Float>(2, 0, 5, 1) // camera translated to world (2, _, 5)
        let imageSize = CGSize(width: 600, height: 800)
        let normalized = CGPoint(x: 1 - 300.0 / 800.0, y: 1 - 400.0 / 600.0)

        let goal = ARSessionManager.unprojectPoint(normalized, imageSize: imageSize,
                                                    intrinsics: intrinsics, cameraTransform: transform, depth: 3.0)

        // Same ray, but the camera itself is offset by (2, 0, 5): world = (2,0,5) + (0,0,-3).
        XCTAssertEqual(goal.x, 2, accuracy: 0.01)
        XCTAssertEqual(goal.y, 2, accuracy: 0.01)
    }

    // MARK: - Orientation

    /// The invariant the whole mount-agnostic path rests on: a point picked in the image
    /// the model saw (`CIImage.oriented`, which is what `FrameEncoder` sends the brain and
    /// matches what Vision runs on) maps back to the raw pixel it came from. Checked by
    /// rendering a single marked pixel through Core Image rather than restating the
    /// rotation table, so a sign slip in `sensorPixel` can't hide behind a matching slip
    /// in the test.
    func testSensorPixelInvertsCoreImageRotationForEveryMount() throws {
        let width = 8, height = 4
        let marked = (x: 1, y: 0) // raw top-left-origin pixel; off every axis of symmetry
        let raw = Self.makeMarkedBGRABuffer(width: width, height: height, marked: marked)

        for orientation in [CGImagePropertyOrientation.up, .right, .left, .down] {
            let (upright, size) = try Self.render(CIImage(cvPixelBuffer: raw).oriented(orientation))
            let found = try XCTUnwrap(Self.brightestPixel(upright, width: size.w, height: size.h),
                                      "no marker for \(orientation.rawValue)")
            // Pixel centre in the upright image, as a Vision-normalized (y-up) point.
            let vision = CGPoint(x: (Double(found.x) + 0.5) / Double(size.w),
                                 y: 1 - (Double(found.y) + 0.5) / Double(size.h))

            let sensor = ARSessionManager.sensorPixel(forVisionNormalizedPoint: vision,
                                                      imageSize: CGSize(width: width, height: height),
                                                      orientation: orientation)

            XCTAssertEqual(sensor.x, Double(marked.x) + 0.5, accuracy: 0.01, "orientation \(orientation.rawValue)")
            XCTAssertEqual(sensor.y, Double(marked.y) + 0.5, accuracy: 0.01, "orientation \(orientation.rawValue)")
        }
    }

    func testDefaultOrientationKeepsPortraitMapping() {
        let imageSize = CGSize(width: 1920, height: 1440)
        let p = CGPoint(x: 0.2, y: 0.7)

        let implicit = ARSessionManager.sensorPixel(forVisionNormalizedPoint: p, imageSize: imageSize)
        let explicit = ARSessionManager.sensorPixel(forVisionNormalizedPoint: p, imageSize: imageSize, orientation: .right)

        XCTAssertEqual(implicit, explicit)
    }

    func testImageOrientationFollowsGravityForEachMount() {
        // Camera looking level, rolled about its optical axis. 0° = the usual portrait hold.
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: Self.levelCamera(rollDegrees: 0), previous: .up), .right)
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: Self.levelCamera(rollDegrees: 90), previous: .right), .up)
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: Self.levelCamera(rollDegrees: 180), previous: .right), .left)
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: Self.levelCamera(rollDegrees: 270), previous: .right), .down)
    }

    func testImageOrientationHoldsNearTheDiagonal() {
        // 50° is past the 45° midpoint but inside the hysteresis band: stay put.
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: Self.levelCamera(rollDegrees: 50), previous: .right), .right)
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: Self.levelCamera(rollDegrees: 40), previous: .up), .up)
        // Well past it: switch.
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: Self.levelCamera(rollDegrees: 65), previous: .right), .up)
    }

    func testImageOrientationHoldsWhenLookingStraightDown() {
        // Optical axis along world -Y: world-up has no component in the image plane.
        let down = simd_float4x4(columns: (
            SIMD4<Float>(1, 0, 0, 0),
            SIMD4<Float>(0, 0, -1, 0),
            SIMD4<Float>(0, 1, 0, 0),
            SIMD4<Float>(0, 0, 0, 1)
        ))
        XCTAssertEqual(ARSessionManager.imageOrientation(cameraTransform: down, previous: .left), .left)
    }

    /// A landscape mount must land a detection in the same world spot a portrait mount
    /// does when both are looking at the same raw pixel.
    func testUnprojectPointAgreesAcrossMountsForTheSameRawPixel() {
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(500, 0, 0),
            SIMD3<Float>(0, 500, 0),
            SIMD3<Float>(400, 300, 1)
        ))
        let imageSize = CGSize(width: 800, height: 600)
        let rawPixel = CGPoint(x: 600, y: 150) // right of and above the principal point
        let u = rawPixel.x / imageSize.width, v = rawPixel.y / imageSize.height
        let asPortrait = CGPoint(x: 1 - v, y: 1 - u)  // how that pixel reads in the .right-rotated image
        let asLandscape = CGPoint(x: u, y: 1 - v)     // and in the .up image

        let a = ARSessionManager.unprojectPoint(asPortrait, imageSize: imageSize, orientation: .right,
                                                intrinsics: intrinsics, cameraTransform: simd_float4x4(1), depth: 2)
        let b = ARSessionManager.unprojectPoint(asLandscape, imageSize: imageSize, orientation: .up,
                                                intrinsics: intrinsics, cameraTransform: simd_float4x4(1), depth: 2)

        XCTAssertEqual(a.x, b.x, accuracy: 1e-6)
        XCTAssertEqual(a.y, b.y, accuracy: 1e-6)
        XCTAssertEqual(a.x, 0.8, accuracy: 1e-6) // (600 - 400) / 500 * 2
    }

    func testFrameEncoderSendsTheUprightImage() throws {
        let raw = Self.makeMarkedBGRABuffer(width: 40, height: 20, marked: (x: 0, y: 0))

        let portrait = try XCTUnwrap(FrameEncoder.jpeg(raw, orientation: .right).flatMap(UIImage.init(data:)))
        let landscape = try XCTUnwrap(FrameEncoder.jpeg(raw, orientation: .up).flatMap(UIImage.init(data:)))

        XCTAssertEqual(portrait.size.width, 20)
        XCTAssertEqual(portrait.size.height, 40)
        XCTAssertEqual(landscape.size.width, 40)
        XCTAssertEqual(landscape.size.height, 20)
    }

    // MARK: - No LiDAR

    private static let intrinsics = simd_float3x3(columns: (
        SIMD3<Float>(1000, 0, 0), SIMD3<Float>(0, 1000, 0), SIMD3<Float>(960, 720, 1)
    ))
    private static let sensor = CGSize(width: 1920, height: 1440)

    func testCameraRayAgreesWithUnprojection() {
        let transform = Self.levelCamera(rollDegrees: 0)
        let point = CGPoint(x: 0.3, y: 0.2)
        let ray = ARSessionManager.cameraRay(through: point, imageSize: Self.sensor, orientation: .right,
                                             intrinsics: Self.intrinsics, cameraTransform: transform)
        let atDepth = ARSessionManager.unprojectPoint(point, imageSize: Self.sensor, orientation: .right,
                                                      intrinsics: Self.intrinsics, cameraTransform: transform,
                                                      depth: 3)
        // Same pixel, so the depth-3 point lies on the ray.
        let horizontal = simd_normalize(SIMD2(ray.direction.x, ray.direction.z))
        let toPoint = simd_normalize(SIMD2(Float(atDepth.x) - ray.origin.x, Float(atDepth.y) - ray.origin.z))
        XCTAssertEqual(simd_dot(horizontal, toPoint), 1, accuracy: 1e-4)
    }

    /// A phone 0.2 m up on the rover, looking level: the ray to feet 2.5 m ahead drops
    /// 0.2 m over 2.5 m, and must land on the floor there.
    func testFeetRayLandsOnTheFloorAtTheirDistance() {
        let origin = SIMD3<Float>(0, 0.2, 0)
        let direction = simd_normalize(SIMD3<Float>(0, -0.2, -2.5))

        let hit = ARSessionManager.floorIntersection(origin: origin, direction: direction, floorY: 0)

        XCTAssertEqual(hit?.z ?? 0, -2.5, accuracy: 1e-3)
        XCTAssertEqual(hit?.y ?? 1, 0, accuracy: 1e-4)
    }

    func testALevelOrRisingRayNeverHitsTheFloor() {
        let origin = SIMD3<Float>(0, 0.2, 0)
        XCTAssertNil(ARSessionManager.floorIntersection(origin: origin, direction: SIMD3(0, 0, -1), floorY: 0))
        XCTAssertNil(ARSessionManager.floorIntersection(origin: origin, direction: simd_normalize(SIMD3(0, 0.1, -1)), floorY: 0))
        // Nearly level: the "hit" is tens of metres out, which is noise, not a person.
        XCTAssertNil(ARSessionManager.floorIntersection(origin: origin, direction: simd_normalize(SIMD3(0, -0.005, -1)), floorY: 0))
    }

    func testRangeFromApparentHeight() {
        // 1.7 m tall at 2.5 m with a 1000 px focal length spans 680 px of the frame's
        // height. Portrait (.right): upright height is the sensor's 1920 px width.
        let fraction = 680.0 / 1920.0
        let box = CGRect(x: 0.4, y: 0.3, width: 0.2, height: fraction)

        let range = ARSessionManager.depthFromApparentHeight(box, imageSize: Self.sensor, orientation: .right,
                                                             intrinsics: Self.intrinsics)

        XCTAssertEqual(range ?? 0, 2.5, accuracy: 0.01)
    }

    func testACutOffPersonGivesNoRange() {
        let touchingBottom = CGRect(x: 0.4, y: 0.0, width: 0.2, height: 0.6)
        XCTAssertNil(ARSessionManager.depthFromApparentHeight(touchingBottom, imageSize: Self.sensor,
                                                              orientation: .right, intrinsics: Self.intrinsics))
    }

    // MARK: - Helpers

    /// Camera looking level along world -Z, rolled about its optical axis. ARKit's camera
    /// +X runs toward the phone's bottom edge, so at roll 0 it points at the ground.
    private static func levelCamera(rollDegrees: Double) -> simd_float4x4 {
        let r = rollDegrees * .pi / 180
        let x = SIMD3<Float>(Float(sin(r)), Float(-cos(r)), 0)
        let z = SIMD3<Float>(0, 0, 1)
        let y = simd_cross(z, x)
        return simd_float4x4(columns: (
            SIMD4<Float>(x, 0), SIMD4<Float>(y, 0), SIMD4<Float>(z, 0), SIMD4<Float>(0, 0, 0, 1)
        ))
    }

    private static func makeMarkedBGRABuffer(width: Int, height: Int, marked: (x: Int, y: Int)) -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        let attrs: [CFString: Any] = [
            kCVPixelBufferCGImageCompatibilityKey: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey: true
        ]
        CVPixelBufferCreate(kCFAllocatorDefault, width, height, kCVPixelFormatType_32BGRA,
                            attrs as CFDictionary, &buffer)
        let pixelBuffer = buffer!
        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }
        let rowBytes = CVPixelBufferGetBytesPerRow(pixelBuffer)
        let base = CVPixelBufferGetBaseAddress(pixelBuffer)!.assumingMemoryBound(to: UInt8.self)
        for y in 0..<height {
            for x in 0..<width {
                let px = base + y * rowBytes + x * 4
                let on: UInt8 = (x == marked.x && y == marked.y) ? 255 : 0
                px[0] = on; px[1] = on; px[2] = on; px[3] = 255
            }
        }
        return pixelBuffer
    }

    /// Renders to a top-left-origin RGBA8 bitmap.
    private static func render(_ image: CIImage) throws -> (pixels: [UInt8], size: (w: Int, h: Int)) {
        let extent = image.extent.integral
        let w = Int(extent.width), h = Int(extent.height)
        var pixels = [UInt8](repeating: 0, count: w * h * 4)
        let context = CIContext(options: [.workingColorSpace: NSNull(), .outputColorSpace: NSNull()])
        let cg = try XCTUnwrap(context.createCGImage(image, from: extent))
        let bitmap = try XCTUnwrap(CGContext(data: &pixels, width: w, height: h, bitsPerComponent: 8,
                                             bytesPerRow: w * 4, space: CGColorSpaceCreateDeviceRGB(),
                                             bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue))
        bitmap.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
        return (pixels, (w, h))
    }

    private static func brightestPixel(_ pixels: [UInt8], width: Int, height: Int) -> (x: Int, y: Int)? {
        var best: (x: Int, y: Int, v: Int)?
        for y in 0..<height {
            for x in 0..<width {
                let v = Int(pixels[(y * width + x) * 4])
                if v > 128, v > (best?.v ?? 0) { best = (x, y, v) }
            }
        }
        return best.map { ($0.x, $0.y) }
    }

    private static func makeDepthBuffer(width: Int, height: Int, constantDepth: Float) -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        let attrs: [CFString: Any] = [
            kCVPixelBufferCGImageCompatibilityKey: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey: true,
        ]
        CVPixelBufferCreate(kCFAllocatorDefault, width, height, kCVPixelFormatType_DepthFloat32,
                            attrs as CFDictionary, &pixelBuffer)
        guard let buffer = pixelBuffer else { fatalError("failed to create synthetic depth buffer") }

        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let stride = CVPixelBufferGetBytesPerRow(buffer) / MemoryLayout<Float32>.size
        let base = CVPixelBufferGetBaseAddress(buffer)!.assumingMemoryBound(to: Float32.self)
        for y in 0..<height {
            for x in 0..<width {
                base[y * stride + x] = constantDepth
            }
        }
        return buffer
    }

    private static func setDepth(_ depth: Float, x: Int, y: Int, in depthMap: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(depthMap, [])
        defer { CVPixelBufferUnlockBaseAddress(depthMap, []) }
        let stride = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
        let base = CVPixelBufferGetBaseAddress(depthMap)!.assumingMemoryBound(to: Float32.self)
        base[y * stride + x] = depth
    }
}
