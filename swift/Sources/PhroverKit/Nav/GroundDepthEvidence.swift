import Foundation
import RoverNav
import simd

/// Immutable raw-depth evidence paired with a planning capture. Only affirmative
/// floor observations can refute lower mesh; absence/occlusion never clears it.
struct GroundDepthEvidence: Sendable {
    enum Verdict { case floorObserved, lowerSurfaceObserved, unresolved }
    let capturedAt: Double
    let newestTimestamp: Double
    private let frames: [DepthSafetyProjection]

    init?(snapshot: DepthSafetySnapshot, at time: Double) {
        guard time.isFinite, snapshot.timestamp.isFinite, time >= snapshot.timestamp,
              time - snapshot.timestamp <= DepthSafetyEvaluator.maximumRawDepthAge,
              snapshot.failureReason == nil, let live = snapshot.projection else { return nil }
        let older = snapshot.history.filter {
            $0.timestamp.isFinite && snapshot.timestamp - $0.timestamp >= 0.05 &&
                time >= $0.timestamp && time - $0.timestamp <= DepthSafetyEvaluator.maximumRawDepthAge
        }.max { $0.timestamp < $1.timestamp }
        guard let older else { return nil }
        let projections = [live, older.projection]
        guard projections.allSatisfy(Self.valid) else { return nil }
        capturedAt = time; newestTimestamp = snapshot.timestamp; frames = projections
    }

    private static func valid(_ p: DepthSafetyProjection) -> Bool {
        guard p.width > 0, p.height > 0, p.height <= Int.max / p.width,
              p.depthValues.count == p.width * p.height,
              p.fx.isFinite, p.fy.isFinite, p.fx > 0, p.fy > 0, p.cx.isFinite, p.cy.isFinite,
              (0..<4).allSatisfy({ column in (0..<4).allSatisfy { p.cameraFromWorld[column][$0].isFinite } }) else { return false }
        let determinant = simd_determinant(p.cameraFromWorld)
        guard determinant.isFinite, abs(determinant - 1) < 0.001,
              abs(p.cameraFromWorld.columns.3.w - 1) < 1e-6 else { return false }
        let axes = (0..<3).map { column -> SIMD3<Float> in
            let v = p.cameraFromWorld[column]
            return SIMD3(v.x, v.y, v.z)
        }
        return (0..<3).allSatisfy { i in
            abs(p.cameraFromWorld[i].w) < 1e-6 && abs(simd_length_squared(axes[i]) - 1) < 0.001 &&
                (0..<3).allSatisfy { j in i == j || abs(simd_dot(axes[i], axes[j])) < 0.001 }
        }
    }

    func assess(cellX: Int, cellZ: Int, map: Costmap, plane: LocalGroundPlane) -> Verdict {
        let center = map.cellCenter(cellX, cellZ)
        let offset = map.resolution * 0.4
        let points = [center, center + Vec2(-offset, -offset), center + Vec2(offset, -offset),
                      center + Vec2(-offset, offset), center + Vec2(offset, offset)]
        var verdicts: [Verdict] = []
        for p in frames {
            let worldFromCamera = simd_inverse(p.cameraFromWorld)
            var pixels = Set<Int>()
            var floorCount = 0, lowerCount = 0
            for point in points {
                guard let floor = plane.height(at: point) else { return .unresolved }
                let camera = p.cameraFromWorld * SIMD4<Float>(Float(point.x), floor, Float(point.y), 1)
                let depth = -Double(camera.z)
                guard depth.isFinite, depth > 0.05 else { return .unresolved }
                let x = (p.fx * Double(camera.x) / depth + p.cx).rounded()
                let y = (p.cy - p.fy * Double(camera.y) / depth).rounded()
                guard x.isFinite, y.isFinite, x >= 0, y >= 0, x < Double(p.width), y < Double(p.height) else { return .unresolved }
                let index = Int(y) * p.width + Int(x)
                pixels.insert(index)
                let measured = Double(p.depthValues[index])
                guard measured.isFinite, measured > 0.05 else { return .unresolved }
                let hit = worldFromCamera * SIMD4<Float>(Float((x - p.cx) / p.fx * measured),
                    Float((p.cy - y) / p.fy * measured), Float(-measured), 1)
                let hitPoint = Vec2(Double(hit.x), Double(hit.z))
                guard hitPoint.distance(to: point) <= map.resolution * 0.4,
                      let expectedHeight = plane.height(at: hitPoint), hit.y.isFinite else { return .unresolved }
                let difference = Double(hit.y - expectedHeight)
                if abs(difference) <= 0.02 { floorCount += 1 }
                if difference < -0.05 { lowerCount += 1 }
            }
            guard pixels.count >= 3 else { return .unresolved }
            if floorCount == points.count { verdicts.append(.floorObserved) }
            else if lowerCount == points.count { verdicts.append(.lowerSurfaceObserved) }
            else { return .unresolved }
        }
        if verdicts.allSatisfy({ $0 == .floorObserved }) { return .floorObserved }
        if verdicts.allSatisfy({ $0 == .lowerSurfaceObserved }) { return .lowerSurfaceObserved }
        return .unresolved
    }
}
