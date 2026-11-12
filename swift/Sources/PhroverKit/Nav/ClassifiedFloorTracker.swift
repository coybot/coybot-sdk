import Foundation
import RoverNav
import simd

/// Independent floor evidence from classified AR planes, never camera height.
/// Three distinct geometry updates must agree beneath the rover before use.
struct ClassifiedFloorTracker {
    struct Plane {
        let xSlope: Double
        let zSlope: Double
        let intercept: Double
        let boundary: [Vec2]

        init?(origin: SIMD3<Double>, normal: SIMD3<Double>, boundary: [Vec2]) {
            guard (0..<3).allSatisfy({ origin[$0].isFinite && normal[$0].isFinite }),
                  abs(normal.y) > 1e-6, (3...128).contains(boundary.count),
                  boundary.allSatisfy({ $0.x.isFinite && $0.y.isFinite }) else { return nil }
            let a = -normal.x / normal.y, b = -normal.z / normal.y
            guard hypot(a, b) <= 0.08 else { return nil }
            xSlope = a; zSlope = b
            intercept = origin.y - a * origin.x - b * origin.z
            self.boundary = boundary
        }

        func height(at point: Vec2) -> Double? {
            guard point.x.isFinite, point.y.isFinite else { return nil }
            var inside = false
            for i in boundary.indices {
                let a = boundary[i], b = boundary[(i + 1) % boundary.count]
                let dx = b.x - a.x, dz = b.y - a.y
                let lengthSquared = dx * dx + dz * dz
                guard lengthSquared.isFinite, lengthSquared > 1e-12 else { return nil }
                let t = max(0, min(1, ((point.x - a.x) * dx + (point.y - a.y) * dz) / lengthSquared))
                // Require observed floor around the center, not just a distant
                // horizontal plane at a convenient height or its bounding box.
                guard hypot(point.x - a.x - t * dx, point.y - a.y - t * dz) >= 0.25 else { return nil }
                if (a.y > point.y) != (b.y > point.y),
                   point.x < a.x + (point.y - a.y) * dx / dz { inside.toggle() }
            }
            let value = intercept + xSlope * point.x + zSlope * point.y
            return inside && value.isFinite ? value : nil
        }
    }

    private struct Sample { let time: Double; let plane: Plane }
    private var samples: [UUID: [Sample]] = [:]
    private var highWater: Double?
    static let maximumAge = 2.0

    mutating func clear() { samples.removeAll(); highWater = nil }
    mutating func remove(_ ids: Set<UUID>) { for id in ids { samples.removeValue(forKey: id) } }

    mutating func record(id: UUID, timestamp: Double, plane: Plane?) {
        guard timestamp.isFinite else { clear(); return }
        if let highWater, timestamp < highWater { clear(); return }
        highWater = timestamp
        samples = samples.filter { timestamp - ($0.value.last?.time ?? -.infinity) <= Self.maximumAge }
        guard let plane else { samples.removeValue(forKey: id); return }
        var history = samples[id] ?? []
        if let last = history.last, timestamp - last.time < 0.075 {
            history[history.count - 1] = .init(time: last.time, plane: plane)
            samples[id] = history
            return
        }
        history.append(.init(time: timestamp, plane: plane))
        if history.count > 3 { history.removeFirst(history.count - 3) }
        if samples[id] == nil, samples.count >= 32 { return }
        samples[id] = history
    }

    func reference(at position: Vec2, now: Double) -> Float? {
        guard now.isFinite, let highWater, now >= highWater else { return nil }
        var heights: [Double] = []
        var stable: [Double] = []
        for history in samples.values {
            guard let latest = history.last, now - latest.time <= Self.maximumAge,
                  let currentHeight = latest.plane.height(at: position) else { continue }
            heights.append(currentHeight) // Even an immature conflicting level vetoes selection.
            guard history.count == 3, latest.time - history[0].time >= 0.15,
                  now - history[0].time <= Self.maximumAge else { continue }
            let values = history.compactMap { $0.plane.height(at: position) }.sorted()
            guard values.count == 3, values[2] - values[0] <= 0.02 else { continue }
            stable.append(values[1])
        }
        guard !stable.isEmpty, let low = heights.min(), let high = heights.max(), high - low <= 0.02 else { return nil }
        let value = stable.sorted()[stable.count / 2]
        let result = Float(value)
        return result.isFinite ? result : nil
    }
}
