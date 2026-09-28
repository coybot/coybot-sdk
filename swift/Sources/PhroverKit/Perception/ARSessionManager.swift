import Foundation
import ARKit
import ImageIO
import RoverNav

/// Owns the ARKit session and is the rover's **primary odometry + mapping** source
/// (the WAVE ROVER base has no wheel encoders). Provides:
///   • 6DoF pose flattened to the nav ground plane (`Pose2D`)
///   • LiDAR scene mesh anchors → obstacles for the costmap
///   • live LiDAR depth → reactive obstacle avoidance
///
/// Nav frame convention (matches RoverNav.Geometry): x = ARKit world X, y = ARKit world Z.
@Observable
@MainActor
public final class ARSessionManager: NSObject, @preconcurrency ARSessionDelegate {
    public let session = ARSession()

    public private(set) var pose: Pose2D?
    public private(set) var meshAnchors: [ARMeshAnchor] = []
    /// Nearest obstacle distance (m) in a forward cone from the latest depth frame.
    public private(set) var forwardClearance: Double = .infinity
    public private(set) var trackingState: ARCamera.TrackingState = .notAvailable

    /// Latest RGB frame, for `Detector` to run inference on.
    public private(set) var latestPixelBuffer: CVPixelBuffer?
    /// Latest camera (intrinsics + transform + raw sensor `imageResolution`), retained so
    /// `unproject(normalizedPoint:)` can back-project a detection into the world.
    public private(set) var latestCamera: ARCamera?
    /// Latest LiDAR depth map (meters, aligned to `latestCamera.imageResolution`'s aspect).
    public private(set) var latestDepthMap: CVPixelBuffer?
    /// Which way up the raw camera buffer is, derived from gravity each frame. Everything
    /// that hands the image to a model (`Detector`, `FrameEncoder`) rotates by this so the
    /// model sees the scene upright, and `unproject` inverts the same rotation — so the
    /// phone can be mounted portrait, landscape either way, or upside down.
    public private(set) var imageOrientation: CGImagePropertyOrientation = .right
    /// World height (ARKit y) of the floor, from the lowest horizontal plane ARKit has
    /// found below the camera. What a phone with no LiDAR places people and objects on.
    public private(set) var floorHeight: Float?
    /// Whether this device produces a LiDAR depth map at all (Pro iPhones and iPads).
    public var hasLiDAR: Bool { ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) }
    private var lastClearanceLogAt = Date.distantPast

    public override init() {
        super.init()
        session.delegate = self
    }

    public func start() {
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics.insert(.smoothedSceneDepth)
        }
        // Without LiDAR there is no depth map; the floor plane is what everything is
        // placed on instead (see `groundPoint(boundingBox:label:)`).
        if !hasLiDAR {
            config.planeDetection = [.horizontal]
        }
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    public func pause() { session.pause() }

    // MARK: - ARSessionDelegate

    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        trackingState = frame.camera.trackingState
        pose = Self.groundPose(from: frame.camera.transform)
        latestPixelBuffer = frame.capturedImage
        latestCamera = frame.camera
        let orientation = Self.imageOrientation(cameraTransform: frame.camera.transform, previous: imageOrientation)
        if orientation != imageOrientation {
            imageOrientation = orientation
            RuntimeFileLog.append("image_orientation", fields: ["exif": "\(orientation.rawValue)"])
        }
        if let depth = frame.smoothedSceneDepth ?? frame.sceneDepth {
            forwardClearance = Self.forwardClearance(from: depth)
            latestDepthMap = depth.depthMap
            logForwardClearanceIfNeeded()
        }
    }

    public func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        collectMesh(anchors)
        collectFloor(anchors)
    }
    public func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        collectMesh(anchors)
        collectFloor(anchors)
    }

    private func collectFloor(_ anchors: [ARAnchor]) {
        guard let cameraY = latestCamera?.transform.columns.3.y else { return }
        let heights = anchors
            .compactMap { $0 as? ARPlaneAnchor }
            .filter { $0.alignment == .horizontal }
            .map { $0.transform.columns.3.y }
            // A table top is a horizontal plane too; the floor is below the camera.
            .filter { $0 < cameraY - 0.05 }
        guard let lowest = heights.min() else { return }
        floorHeight = min(floorHeight ?? lowest, lowest)
    }

    private func collectMesh(_ anchors: [ARAnchor]) {
        let mesh = anchors.compactMap { $0 as? ARMeshAnchor }
        guard !mesh.isEmpty else { return }
        var map = Dictionary(meshAnchors.map { ($0.identifier, $0) }, uniquingKeysWith: { a, _ in a })
        for m in mesh { map[m.identifier] = m }
        meshAnchors = Array(map.values)
    }

    private func logForwardClearanceIfNeeded(now: Date = Date()) {
        guard now.timeIntervalSince(lastClearanceLogAt) >= 1 else { return }
        lastClearanceLogAt = now
        RuntimeFileLog.append("forward_clearance", fields: [
            "meters": forwardClearance.isFinite ? String(format: "%.2f", forwardClearance) : "inf",
            "tracking": trackingStateDescription(trackingState)
        ], now: now)
    }

    private func trackingStateDescription(_ state: ARCamera.TrackingState) -> String {
        switch state {
        case .normal: return "normal"
        case .limited: return "limited"
        case .notAvailable: return "notAvailable"
        @unknown default: return "unknown"
        }
    }

    // MARK: - Object grounding

    /// Back-projects a point in normalized Vision coordinates (bottom-left origin, y-up)
    /// of the *upright* image — the frame `Detector` boxes and the cloud brain's image
    /// points are both in, because both see the buffer rotated by `imageOrientation` — to
    /// a world point on the nav plane, by sampling the aligned LiDAR depth map and
    /// unprojecting through the camera intrinsics. Returns `nil` if there's no
    /// camera/depth yet or the sampled depth is invalid.
    ///
    /// `ARCamera` has no public initializer, so the math is split into the static helpers
    /// `sensorPixel`/`sampleDepth`/`unprojectPoint` so it can be exercised in tests against
    /// synthetic intrinsics/transforms/depth.
    public func unproject(normalizedPoint: CGPoint) -> Vec2? {
        guard let camera = latestCamera, let depthMap = latestDepthMap else { return nil }
        let imageSize = camera.imageResolution // raw (landscape) sensor pixel space, matches `intrinsics`
        guard let depth = Self.sampleDepth(depthMap, atVisionNormalizedPoint: normalizedPoint,
                                           imageSize: imageSize, orientation: imageOrientation) else {
            return nil
        }
        return Self.unprojectPoint(normalizedPoint, imageSize: imageSize, orientation: imageOrientation,
                                   intrinsics: camera.intrinsics, cameraTransform: camera.transform, depth: depth)
    }

    /// Where a detected object stands on the nav plane. With LiDAR, the depth under its
    /// centre (`unproject`). Without it, where the ray through the bottom-centre of its
    /// box — a person's feet — meets the floor ARKit has found; before a floor is found,
    /// and only for a person, from how tall they look against an assumed 1.7 m.
    public func groundPoint(boundingBox box: CGRect, label: String) -> Vec2? {
        if latestDepthMap != nil {
            return unproject(normalizedPoint: CGPoint(x: box.midX, y: box.midY))
        }
        guard let camera = latestCamera else { return nil }
        let imageSize = camera.imageResolution
        if let floorHeight {
            let ray = Self.cameraRay(through: CGPoint(x: box.midX, y: box.minY), imageSize: imageSize,
                                     orientation: imageOrientation, intrinsics: camera.intrinsics,
                                     cameraTransform: camera.transform)
            if let hit = Self.floorIntersection(origin: ray.origin, direction: ray.direction, floorY: floorHeight) {
                return Vec2(Double(hit.x), Double(hit.z))
            }
        }
        guard MissionAgent.visualQueryMatchScore(query: "person", label: label) > 0,
              let depth = Self.depthFromApparentHeight(box, imageSize: imageSize,
                                                        orientation: imageOrientation,
                                                        intrinsics: camera.intrinsics)
        else { return nil }
        return Self.unprojectPoint(CGPoint(x: box.midX, y: box.midY), imageSize: imageSize,
                                   orientation: imageOrientation, intrinsics: camera.intrinsics,
                                   cameraTransform: camera.transform, depth: Float(depth))
    }

    /// The world ray through a Vision-normalized image point.
    static func cameraRay(through point: CGPoint, imageSize: CGSize, orientation: CGImagePropertyOrientation,
                          intrinsics: simd_float3x3, cameraTransform: simd_float4x4)
        -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let sensor = sensorPixel(forVisionNormalizedPoint: point, imageSize: imageSize, orientation: orientation)
        let fx = Double(intrinsics[0][0]), fy = Double(intrinsics[1][1])
        let cx = Double(intrinsics[2][0]), cy = Double(intrinsics[2][1])
        // Camera space at depth 1: +X right, +Y up, looking down -Z (as in unprojectPoint).
        let local = SIMD4<Float>(Float((sensor.x - cx) / fx), Float(-(sensor.y - cy) / fy), -1, 0)
        let world = cameraTransform * local
        let origin = cameraTransform.columns.3
        return (SIMD3(origin.x, origin.y, origin.z), simd_normalize(SIMD3(world.x, world.y, world.z)))
    }

    /// Where a ray meets the horizontal plane y = `floorY`, if it points down at it and
    /// lands within `maxRange` metres (a nearly level ray "hits" the floor absurdly far
    /// away, which is worse than no answer).
    static func floorIntersection(origin: SIMD3<Float>, direction: SIMD3<Float>, floorY: Float,
                                  maxRange: Float = 12) -> SIMD3<Float>? {
        guard direction.y < -1e-3, origin.y > floorY else { return nil }
        let t = (floorY - origin.y) / direction.y
        let hit = origin + t * direction
        let horizontal = simd_length(SIMD2(hit.x - origin.x, hit.z - origin.z))
        return horizontal <= maxRange ? hit : nil
    }

    /// Range (m, along the optical axis) to a person from how tall their box is, assuming
    /// `assumedHeight`. Rough — a child or someone sitting reads far — so it is only the
    /// fallback until ARKit has a floor. `nil` when the box touches the top or bottom of
    /// the frame, since a cut-off person looks shorter and so farther than they are.
    static func depthFromApparentHeight(_ box: CGRect, imageSize: CGSize, orientation: CGImagePropertyOrientation,
                                        intrinsics: simd_float3x3, assumedHeight: Double = 1.7) -> Double? {
        guard box.minY > 0.02, box.maxY < 0.98 else { return nil }
        let top = sensorPixel(forVisionNormalizedPoint: CGPoint(x: box.midX, y: box.maxY),
                              imageSize: imageSize, orientation: orientation)
        let bottom = sensorPixel(forVisionNormalizedPoint: CGPoint(x: box.midX, y: box.minY),
                                 imageSize: imageSize, orientation: orientation)
        let pixels = hypot(top.x - bottom.x, top.y - bottom.y)
        guard pixels > 1 else { return nil }
        let focal = Double(intrinsics[0][0] + intrinsics[1][1]) / 2
        return focal * assumedHeight / pixels
    }

    /// Undoes the rotation a Vision request (or `CIImage.oriented`) applied for
    /// `orientation`, landing back in the raw sensor pixel space (top-left origin)
    /// `intrinsics`/depth are calibrated against.
    static func sensorPixel(forVisionNormalizedPoint p: CGPoint, imageSize: CGSize,
                            orientation: CGImagePropertyOrientation = .right) -> CGPoint {
        // Upright image, top-left origin, normalized.
        let u = p.x, v = 1 - p.y
        let raw: (u: Double, v: Double)
        switch orientation {
        case .up: raw = (u, v)
        case .left: raw = (1 - v, u)        // displayed after a 90° counter-clockwise turn
        case .down: raw = (1 - u, 1 - v)    // 180°
        default: raw = (v, 1 - u)           // .right, 90° clockwise; mirrored never occurs
        }
        return CGPoint(x: raw.u * imageSize.width, y: raw.v * imageSize.height)
    }

    /// Minimum lead the gravity component on a new image axis needs over the current one
    /// before the orientation flips. About 12° either side of 45°, so a mount that sits
    /// near the diagonal, or a chassis rocking over a threshold, doesn't flicker.
    static let orientationHysteresis = 0.2

    /// Which EXIF orientation makes the raw buffer upright, from where world-up falls in
    /// the camera's image plane. ARKit's camera +X runs along the raw image toward its right
    /// edge and +Y toward its top, so world-up's components on those axes say which raw
    /// edge is up: left edge up is the ordinary portrait hold (`.right`), top edge up is
    /// landscape with the raw buffer already upright (`.up`). Holds `previous` when the
    /// camera looks nearly straight up or down, where the roll is undefined.
    static func imageOrientation(cameraTransform t: simd_float4x4,
                                 previous: CGImagePropertyOrientation) -> CGImagePropertyOrientation {
        let upX = Double(t.columns.0.y), upY = Double(t.columns.1.y)
        func lift(_ o: CGImagePropertyOrientation) -> Double {
            switch o {
            case .up: return upY
            case .left: return upX
            case .down: return -upY
            default: return -upX
            }
        }
        let best = [CGImagePropertyOrientation.right, .up, .left, .down].max { lift($0) < lift($1) }!
        guard best != previous, lift(best) > 0.3,
              lift(best) > lift(previous) + orientationHysteresis else { return previous }
        return best
    }

    /// Fraction of the depth map's smaller side sampled either side of the point. Small
    /// enough to stay on the object, large enough to span the gaps a single pixel falls
    /// through (see `sampleDepth`).
    static let depthSampleWindowFraction = 0.03

    /// Samples the LiDAR depth map (meters) at the raw-sensor-space point corresponding to
    /// a Vision-normalized point. The depth map is lower-res than the color camera but
    /// aligned to the same field of view, so the fractional position carries over directly.
    ///
    /// Reads a small window and takes its 25th percentile rather than the single pixel
    /// under the point. A detection's centre is not guaranteed to land on the object: on a
    /// standing person it routinely falls between torso and arm and reads the wall metres
    /// behind them, which puts the unprojected world point well past the target. Preferring
    /// the nearer quartile of a local window biases toward the surface facing the camera —
    /// the same reasoning as `forwardClearance(fromDepthMap:)`'s low percentile, over a
    /// local window instead of the whole driving corridor. Erring near also errs safe: a
    /// goal short of the object is a stop, one behind it is a collision.
    static func sampleDepth(_ depthMap: CVPixelBuffer, atVisionNormalizedPoint p: CGPoint, imageSize: CGSize,
                            orientation: CGImagePropertyOrientation = .right) -> Float? {
        guard imageSize.width > 0, imageSize.height > 0 else { return nil }
        let sensor = sensorPixel(forVisionNormalizedPoint: p, imageSize: imageSize, orientation: orientation)

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        let dw = CVPixelBufferGetWidth(depthMap), dh = CVPixelBufferGetHeight(depthMap)
        guard let base = CVPixelBufferGetBaseAddress(depthMap), dw > 0, dh > 0 else { return nil }
        let stride = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
        let ptr = base.assumingMemoryBound(to: Float32.self)
        let dx = min(dw - 1, max(0, Int((sensor.x / imageSize.width) * Double(dw))))
        let dy = min(dh - 1, max(0, Int((sensor.y / imageSize.height) * Double(dh))))

        let half = max(1, Int(Double(min(dw, dh)) * depthSampleWindowFraction))
        var samples: [Float] = []
        samples.reserveCapacity((2 * half + 1) * (2 * half + 1))
        for y in max(0, dy - half)...min(dh - 1, dy + half) {
            for x in max(0, dx - half)...min(dw - 1, dx + half) {
                let d = ptr[y * stride + x]
                if d > 0.05 && d.isFinite { samples.append(d) }
            }
        }
        guard !samples.isEmpty else { return nil }
        samples.sort()
        let index = min(samples.count - 1, max(0, Int(Double(samples.count - 1) * 0.25)))
        return samples[index]
    }

    /// Back-projects a raw-sensor-space point at a known depth through the camera
    /// intrinsics and pose into a world-plane point (matches `groundPose`'s x/world-x,
    /// y/world-z convention). Pure math — testable with synthetic intrinsics/transform.
    static func unprojectPoint(_ visionNormalizedPoint: CGPoint, imageSize: CGSize,
                               orientation: CGImagePropertyOrientation = .right,
                               intrinsics: simd_float3x3, cameraTransform: simd_float4x4, depth: Float) -> Vec2 {
        let sensor = sensorPixel(forVisionNormalizedPoint: visionNormalizedPoint, imageSize: imageSize,
                                 orientation: orientation)
        let fx = Double(intrinsics[0][0]), fy = Double(intrinsics[1][1])
        let cx = Double(intrinsics[2][0]), cy = Double(intrinsics[2][1])
        let d = Double(depth)

        // Camera space: +X right, +Y up, camera looks down -Z (same convention as groundPose).
        let xCam = (sensor.x - cx) / fx * d
        let yCam = -(sensor.y - cy) / fy * d
        let zCam = -d

        let world = cameraTransform * SIMD4<Float>(Float(xCam), Float(yCam), Float(zCam), 1)
        return Vec2(Double(world.x), Double(world.z))
    }

    // MARK: - Geometry helpers

    /// Flatten an ARKit camera transform to a ground-plane pose. Camera looks down -Z.
    static func groundPose(from t: simd_float4x4) -> Pose2D {
        let p = t.columns.3
        // Device forward in world = -(third basis column).
        let fwd = -SIMD3<Float>(t.columns.2.x, t.columns.2.y, t.columns.2.z)
        let yaw = atan2(Double(fwd.z), Double(fwd.x))
        return Pose2D(position: Vec2(Double(p.x), Double(p.z)), yaw: yaw)
    }

    /// Robust near depth (m) sampled from the center region of the LiDAR depth map.
    static func forwardClearance(from depth: ARDepthData) -> Double {
        forwardClearance(fromDepthMap: depth.depthMap)
    }

    /// Robust near depth (m) sampled from the driving corridor of the LiDAR depth map.
    static func forwardClearance(fromDepthMap map: CVPixelBuffer) -> Double {
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        let w = CVPixelBufferGetWidth(map), h = CVPixelBufferGetHeight(map)
        guard let base = CVPixelBufferGetBaseAddress(map), w > 0, h > 0 else { return .infinity }
        let rowBytes = CVPixelBufferGetBytesPerRow(map)
        let ptr = base.assumingMemoryBound(to: Float32.self)
        let stride = rowBytes / MemoryLayout<Float32>.size

        var depths: [Float] = []
        depths.reserveCapacity((h / 3) * (w / 2))
        // Sample a wider central driving corridor. A wall slightly off-center in the
        // mounted phone's view still needs to stop the rover before contact.
        for y in (h / 3)..<(h * 2 / 3) {
            for x in (w / 4)..<(w * 3 / 4) {
                let d = ptr[y * stride + x]
                if d > 0.05 && d.isFinite { depths.append(d) }
            }
        }
        guard !depths.isEmpty else { return .infinity }
        depths.sort()
        let index = min(depths.count - 1, max(0, Int(Double(depths.count - 1) * 0.10)))
        return Double(depths[index])
    }
}
