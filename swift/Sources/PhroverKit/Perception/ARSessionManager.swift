import Foundation
import ARKit
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
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    public func pause() { session.pause() }

    // MARK: - ARSessionDelegate

    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        trackingState = frame.camera.trackingState
        pose = Self.groundPose(from: frame.camera.transform)
        latestPixelBuffer = frame.capturedImage
        latestCamera = frame.camera
        if let depth = frame.smoothedSceneDepth ?? frame.sceneDepth {
            forwardClearance = Self.forwardClearance(from: depth)
            latestDepthMap = depth.depthMap
            logForwardClearanceIfNeeded()
        }
    }

    public func session(_ session: ARSession, didAdd anchors: [ARAnchor]) { collectMesh(anchors) }
    public func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) { collectMesh(anchors) }

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

    /// Back-projects a point in `Detector`'s normalized Vision coordinates (bottom-left
    /// origin, y-up, in the upright/portrait frame Vision produces because `Detector` hands
    /// it the buffer with `orientation: .right`) to a world point on the nav plane, by
    /// sampling the aligned LiDAR depth map and unprojecting through the camera intrinsics.
    /// Returns `nil` if there's no camera/depth yet or the sampled depth is invalid.
    ///
    /// The `.right`-rotation inverse assumes the phone is held in the same portrait
    /// orientation `Detector` was tuned for; this is the one piece of the perception path
    /// that can only be validated on a real LiDAR device (see rover/README.md status notes)
    /// — `ARCamera` has no public initializer, so the math below is split into the static
    /// helpers `sensorPixel`/`sampleDepth`/`unprojectPoint` specifically so it can still be
    /// exercised in tests against synthetic intrinsics/transforms/depth.
    public func unproject(normalizedPoint: CGPoint) -> Vec2? {
        guard let camera = latestCamera, let depthMap = latestDepthMap else { return nil }
        let imageSize = camera.imageResolution // raw (landscape) sensor pixel space, matches `intrinsics`
        guard let depth = Self.sampleDepth(depthMap, atVisionNormalizedPoint: normalizedPoint, imageSize: imageSize) else {
            return nil
        }
        return Self.unprojectPoint(normalizedPoint, imageSize: imageSize,
                                   intrinsics: camera.intrinsics, cameraTransform: camera.transform, depth: depth)
    }

    /// Undoes the `.right` (90° clockwise) rotation `Detector`'s Vision request handler
    /// applied, landing back in the raw sensor pixel space `intrinsics`/depth are
    /// calibrated against.
    static func sensorPixel(forVisionNormalizedPoint p: CGPoint, imageSize: CGSize) -> CGPoint {
        CGPoint(x: (1 - p.y) * imageSize.width, y: (1 - p.x) * imageSize.height)
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
    static func sampleDepth(_ depthMap: CVPixelBuffer, atVisionNormalizedPoint p: CGPoint, imageSize: CGSize) -> Float? {
        guard imageSize.width > 0, imageSize.height > 0 else { return nil }
        let sensor = sensorPixel(forVisionNormalizedPoint: p, imageSize: imageSize)

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
                               intrinsics: simd_float3x3, cameraTransform: simd_float4x4, depth: Float) -> Vec2 {
        let sensor = sensorPixel(forVisionNormalizedPoint: visionNormalizedPoint, imageSize: imageSize)
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
