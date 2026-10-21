import Foundation
import CoreVideo
import simd
import RoverNav

/// Spatial, same-frame feet projection. Never changes generic object grounding.
public enum FollowPersonProjection {
    public static func project(box: CGRect, in snapshot: ARFrameSnapshot) -> Vec2? {
        evaluate(box: box, detectorConfidence: 0, rawPersonID: 0, in: snapshot).position
    }

    public static func evaluate(box: CGRect, detectorConfidence: Float, rawPersonID: Int,
                                in snapshot: ARFrameSnapshot, bodyVerified: Bool = false,
                                verifiedDepthAnchor: CGPoint? = nil) -> FollowProjectionEvidence {
        var facts = FollowProjectionEvidence(rawPersonID: rawPersonID, box: box,
            detectorConfidence: detectorConfidence, snapshot: snapshot)
        func reject(_ reason: FollowProjectionRejection) -> FollowProjectionEvidence {
            var rejected = facts
            rejected.rejection = reason
            return rejected
        }
        if let map = snapshot.depthMap {
            facts.depthSize = CGSize(width: CVPixelBufferGetWidth(map), height: CVPixelBufferGetHeight(map))
            facts.depthBytesPerRow = CVPixelBufferGetBytesPerRow(map)
            facts.depthPixelFormat = CVPixelBufferGetPixelFormatType(map)
        }
        if let confidence = snapshot.depthConfidenceMap {
            facts.confidenceSize = CGSize(width: CVPixelBufferGetWidth(confidence), height: CVPixelBufferGetHeight(confidence))
            facts.confidenceBytesPerRow = CVPixelBufferGetBytesPerRow(confidence)
            facts.confidencePixelFormat = CVPixelBufferGetPixelFormatType(confidence)
        }
        guard [box.origin.x, box.origin.y, box.size.width, box.size.height].allSatisfy(\.isFinite),
              box.size.width > 0, box.size.height > 0 else { return reject(.invalidBox) }
        facts.clippedLeft = box.minX <= 0
        facts.clippedBottom = box.minY <= 0
        facts.clippedRight = box.maxX >= 1
        facts.clippedTop = box.maxY >= 1
        let feet = CGPoint(x: box.midX, y: box.minY)
        facts.feet = feet
        let visible = bodyVerified
            ? box.minX >= 0 && box.minY > 0 && box.maxX <= 1 && box.maxY <= 1
            : box.minX > 0 && box.minY > 0 && box.maxX < 1 && box.maxY < 1
        guard visible else { return reject(.clippedBox) }
        guard let map = snapshot.depthMap else { return reject(.depthUnavailable) }
        guard CVPixelBufferGetPixelFormatType(map) == kCVPixelFormatType_DepthFloat32,
              !CVPixelBufferIsPlanar(map), CVPixelBufferGetWidth(map) > 0, CVPixelBufferGetHeight(map) > 0,
              CVPixelBufferGetBytesPerRow(map) >= CVPixelBufferGetWidth(map) * MemoryLayout<Float>.stride,
              CVPixelBufferGetBytesPerRow(map) % MemoryLayout<Float>.alignment == 0 else { return reject(.invalidDepthLayout) }
        guard snapshot.imageResolution.width.isFinite, snapshot.imageResolution.height.isFinite,
              snapshot.imageResolution.width > 0, snapshot.imageResolution.height > 0 else { return reject(.invalidCalibration) }
        let anchor = bodyVerified ? (verifiedDepthAnchor ?? feet) : feet
        guard anchor.x.isFinite, anchor.y.isFinite, anchor.x > 0, anchor.x < 1, anchor.y > 0, anchor.y < 1,
              box.insetBy(dx: -0.02, dy: -0.02).contains(anchor) else { return reject(.invalidCalibration) }
        facts.depthAnchor = anchor
        facts.depthAnchorKind = bodyVerified && verifiedDepthAnchor != nil ? "verified_torso" : "box_feet"
        let sensor = ARSessionManager.sensorPixel(forVisionNormalizedPoint: anchor,
            imageSize: snapshot.imageResolution, orientation: snapshot.imageOrientation)
        let width = CVPixelBufferGetWidth(map), height = CVPixelBufferGetHeight(map)
        facts.sensorPixel = sensor
        let depthPixel = CGPoint(x: sensor.x / snapshot.imageResolution.width * CGFloat(width),
                                 y: sensor.y / snapshot.imageResolution.height * CGFloat(height))
        facts.depthPixel = depthPixel
        let x = Int(floor(depthPixel.x)), y = Int(floor(depthPixel.y))
        facts.depthCenter = CGPoint(x: x, y: y)
        guard x >= 2, y >= 2, x + 2 < width, y + 2 < height else { return reject(.clippedDepthWindow) }
        guard CVPixelBufferLockBaseAddress(map, .readOnly) == kCVReturnSuccess else { return reject(.invalidDepthLayout) }
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(map) else { return reject(.invalidDepthLayout) }
        let confidence = snapshot.depthConfidenceMap
        if let confidence {
            guard CVPixelBufferGetPixelFormatType(confidence) == kCVPixelFormatType_OneComponent8,
                  !CVPixelBufferIsPlanar(confidence), CVPixelBufferGetWidth(confidence) == width,
                  CVPixelBufferGetHeight(confidence) == height,
                  CVPixelBufferGetBytesPerRow(confidence) >= width else {
                facts.confidenceAvailability = .invalid
                return reject(.invalidConfidenceMap)
            }
            guard CVPixelBufferLockBaseAddress(confidence, .readOnly) == kCVReturnSuccess else {
                facts.confidenceAvailability = .invalid
                return reject(.invalidConfidenceMap)
            }
        }
        defer { if let confidence { CVPixelBufferUnlockBaseAddress(confidence, .readOnly) } }
        let confidenceBase = confidence.flatMap { CVPixelBufferGetBaseAddress($0) }
        if confidence != nil, confidenceBase == nil {
            facts.confidenceAvailability = .invalid
            return reject(.invalidConfidenceMap)
        }
        var values: [Float] = []
        var invalidDepth = 0, lowConfidence = 0
        for row in (y - 2)...(y + 2) {
            let pixels = base.advanced(by: row * CVPixelBufferGetBytesPerRow(map)).assumingMemoryBound(to: Float.self)
            for col in (x - 2)...(x + 2) {
                let depth = pixels[col]
                let level = confidenceBase.map {
                    $0.advanced(by: row * CVPixelBufferGetBytesPerRow(confidence!)).assumingMemoryBound(to: UInt8.self)[col]
                }
                if !depth.isFinite || depth <= 0.05 { invalidDepth += 1 }
                else if let level, level != 1 && level != 2 { lowConfidence += 1 }
                else { values.append(depth) }
            }
        }
        facts.validSampleCount = values.count
        facts.invalidDepthCount = invalidDepth
        facts.lowConfidenceCount = lowConfidence
        guard values.count >= 5 else { return reject(.insufficientValidDepth) }
        values.sort()
        let depth = median(values)
        let deviations = values.map { abs(Double($0) - depth) }.sorted()
        let middle = deviations.count / 2
        let mad = deviations.count.isMultiple(of: 2)
            ? (deviations[middle - 1] + deviations[middle]) / 2 : deviations[middle]
        facts.medianDepth = depth
        facts.medianAbsoluteDeviation = mad
        let inliers = deviations.filter { $0 <= 0.20 }.count
        let requiredInliers = Int(ceil(0.60 * Double(values.count)))
        facts.inlierCount = inliers
        facts.requiredInlierCount = requiredInliers
        guard mad <= 0.10, inliers >= requiredInliers else { return reject(.inconsistentDepth) }
        guard (0..<3).allSatisfy({ column in (0..<3).allSatisfy { snapshot.cameraIntrinsics[column][$0].isFinite } }),
              snapshot.cameraIntrinsics[0][0] > 0, snapshot.cameraIntrinsics[1][1] > 0,
              (0..<4).allSatisfy({ column in (0..<4).allSatisfy { snapshot.cameraTransform[column][$0].isFinite } }) else { return reject(.invalidCalibration) }
        let calibration = snapshot.cameraIntrinsics
        facts.cameraRay = SIMD3<Double>((sensor.x - Double(calibration[2][0])) / Double(calibration[0][0]),
                                       -(sensor.y - Double(calibration[2][1])) / Double(calibration[1][1]), -1)
        let camera = SIMD4<Float>(
            Float((sensor.x - Double(calibration[2][0])) / Double(calibration[0][0]) * depth),
            Float(-(sensor.y - Double(calibration[2][1])) / Double(calibration[1][1]) * depth),
            -Float(depth), 1)
        let world = snapshot.cameraTransform * camera
        guard (0..<4).allSatisfy({ world[$0].isFinite }) else { return reject(.nonfiniteProjection) }
        facts.worldPoint = SIMD3<Float>(world.x, world.y, world.z)
        let position = Vec2(Double(world.x), Double(world.z))
        facts.position = position
        facts.pairedPose = snapshot.pose
        let dx = position.x - snapshot.pose.position.x, dz = position.y - snapshot.pose.position.y
        facts.groundRange = hypot(dx, dz)
        facts.headingError = RotationDiagnosticMeasurement.normalize(atan2(dz, dx) - snapshot.pose.yaw)
        return facts
    }

    /// Nav-plane azimuth (`atan2(dy, dx)`) of the camera ray through the centre of `box`, or nil
    /// when the box or calibration is unusable. Needs no depth, so it stays valid when the depth
    /// under a partly hidden person would read the occluder.
    public static func centerBearing(box: CGRect, in snapshot: ARFrameSnapshot) -> Double? {
        let visible = box.intersection(CGRect(x: 0, y: 0, width: 1, height: 1))
        guard !visible.isNull, visible.width > 0, visible.height > 0,
              snapshot.imageResolution.width.isFinite, snapshot.imageResolution.width > 0,
              snapshot.imageResolution.height.isFinite, snapshot.imageResolution.height > 0 else { return nil }
        let k = snapshot.cameraIntrinsics
        guard (0..<3).allSatisfy({ column in (0..<3).allSatisfy { k[column][$0].isFinite } }),
              k[0][0] > 0, k[1][1] > 0,
              (0..<4).allSatisfy({ column in (0..<4).allSatisfy { snapshot.cameraTransform[column][$0].isFinite } })
        else { return nil }
        let sensor = ARSessionManager.sensorPixel(forVisionNormalizedPoint: CGPoint(x: visible.midX, y: visible.midY),
            imageSize: snapshot.imageResolution, orientation: snapshot.imageOrientation)
        let direction = SIMD4<Float>(Float((sensor.x - Double(k[2][0])) / Double(k[0][0])),
                                     Float(-(sensor.y - Double(k[2][1])) / Double(k[1][1])), -1, 0)
        let world = snapshot.cameraTransform * direction
        guard world.x.isFinite, world.z.isFinite, world.x != 0 || world.z != 0 else { return nil }
        return atan2(Double(world.z), Double(world.x))
    }

    private static func median(_ sorted: [Float]) -> Double {
        let middle = sorted.count / 2
        return sorted.count.isMultiple(of: 2)
            ? (Double(sorted[middle - 1]) + Double(sorted[middle])) / 2
            : Double(sorted[middle])
    }
}
