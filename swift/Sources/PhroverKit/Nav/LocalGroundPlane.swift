import Foundation
import RoverNav

/// A local mesh plane corroborated by an independently supplied mounted-camera
/// floor reference. It never labels geometry as self-mesh or clears cells.
struct LocalGroundPlane: Sendable {
    static func mountedReference(captures: [(time: Double, height: Double)], cameraHeight: Double) -> Float? {
        guard cameraHeight.isFinite, cameraHeight > 0,
              captures.allSatisfy({ $0.time.isFinite && $0.height.isFinite }) else { return nil }
        let ordered = captures.sorted { $0.time < $1.time }
        var distinct: [(time: Double, height: Double)] = []
        for capture in ordered where distinct.last.map({ capture.time - $0.time >= 0.05 }) ?? true {
            distinct.append(capture)
        }
        guard distinct.count >= 3, distinct.last!.time - distinct.first!.time >= 0.15 else { return nil }
        let heights = distinct.map(\.height).sorted()
        guard heights.last! - heights.first! <= 0.03 else { return nil }
        return Float(heights[heights.count / 2] - cameraHeight)
    }

    let center: Vec2
    let xSlope: Double
    let zSlope: Double
    let height: Double
    let minX: Double
    let maxX: Double
    let minZ: Double
    let maxZ: Double

    func height(at point: Vec2) -> Float? {
        let x = point.x - center.x, z = point.y - center.y
        guard x >= minX, x <= maxX, z >= minZ, z <= maxZ, hypot(x, z) <= 1.2 else { return nil }
        return Float(height + xSlope * x + zSlope * z)
    }

    static func assess(vertices: [SIMD3<Float>], center: Vec2, expectedFloorY: Float?)
        -> (plane: Self?, diagnostics: [String: String]) {
        var fields = ["local_floor_status": "rejected", "mounted_floor_reference_y_m": expectedFloorY.map { String($0) } ?? "unavailable"]
        func reject(_ reason: String) -> (Self?, [String: String]) {
            fields["local_floor_rejection"] = reason
            return (nil, fields)
        }
        guard let expectedFloorY, expectedFloorY.isFinite else { return reject("no_valid_mount_reference") }
        struct Cell: Hashable { let x: Int; let z: Int }
        var cells: [Cell: SIMD3<Double>] = [:]
        for v in vertices {
            guard v.x.isFinite, v.y.isFinite, v.z.isFinite else { continue }
            let p = SIMD3<Double>(Double(v.x) - center.x, Double(v.y), Double(v.z) - center.y)
            let distance = hypot(p.x, p.z)
            // Exclude the chassis-sized central area from floor fitting.
            guard distance >= 0.25, distance <= 1.2 else { continue }
            let cell = Cell(x: Int(floor(p.x / 0.1)), z: Int(floor(p.z / 0.1)))
            if cells[cell].map({ p.y < $0.y }) ?? true { cells[cell] = p }
        }
        let all = cells.values.sorted { $0.x == $1.x ? $0.z < $1.z : $0.x < $1.x }
        let eligible = all.filter { abs($0.y - Double(expectedFloorY)) <= 0.06 }
        fields["local_floor_spatial_cells"] = String(all.count)
        guard eligible.count >= 24 else { return reject("insufficient_reference_consistent_support") }
        let median = eligible.map(\.y).sorted()[eligible.count / 2]
        let seed = eligible.filter { abs($0.y - median) <= 0.03 }
        guard let initial = fit(seed) else { return reject("degenerate_support") }
        let inliers = eligible.filter { abs($0.y - (initial.2 + initial.0 * $0.x + initial.1 * $0.z)) <= 0.015 }
        guard inliers.count >= 24, Double(inliers.count) >= Double(all.count) * 0.7,
              let (a, b, c) = fit(inliers) else { return reject("inconsistent_surface") }
        guard hypot(a, b) <= 0.08, abs(c - Double(expectedFloorY)) <= 0.04 else { return reject("slope_or_mount_mismatch") }
        let rms = sqrt(inliers.reduce(0.0) { $0 + pow($1.y - (c + a * $1.x + b * $1.z), 2) } / Double(inliers.count))
        guard rms <= 0.01 else { return reject("high_residual") }
        let minX = inliers.map(\.x).min()!, maxX = inliers.map(\.x).max()!
        let minZ = inliers.map(\.z).min()!, maxZ = inliers.map(\.z).max()!
        let quadrants = (0..<4).map { quadrant in
            inliers.filter { ($0.x < 0 ? 0 : 1) + ($0.z < 0 ? 0 : 2) == quadrant }.count
        }
        guard minX <= -0.25, maxX >= 0.25, minZ <= -0.25, maxZ >= 0.25,
              quadrants.allSatisfy({ $0 >= 3 }) else { return reject("support_does_not_surround_rover") }
        // A supported lower patch could be a step or the true floor beneath a
        // raised surface. Do not reinterpret that scene as a single clear plane.
        let lowerCells = all.filter { $0.y < c + a * $0.x + b * $0.z - 0.05 }.count
        fields["local_floor_conflicting_lower_cells"] = String(lowerCells)
        guard lowerCells < 3 else { return reject("multiple_floor_levels") }
        fields["local_floor_status"] = "accepted"
        fields["local_floor_y_at_start_m"] = String(c)
        fields["local_floor_x_slope"] = String(a)
        fields["local_floor_z_slope"] = String(b)
        fields["local_floor_inlier_cells"] = String(inliers.count)
        fields["local_floor_rms_m"] = String(rms)
        fields["local_floor_support_bounds_dx_dz_m"] = "\(minX),\(maxX),\(minZ),\(maxZ)"
        return (.init(center: center, xSlope: a, zSlope: b, height: c, minX: minX, maxX: maxX, minZ: minZ, maxZ: maxZ), fields)
    }

    private static func fit(_ points: [SIMD3<Double>]) -> (Double, Double, Double)? {
        guard points.count >= 3 else { return nil }
        let mean = points.reduce(SIMD3<Double>.zero, +) / Double(points.count)
        var xx = 0.0, zz = 0.0, xz = 0.0, xy = 0.0, zy = 0.0
        for point in points {
            let d = point - mean
            xx += d.x * d.x; zz += d.z * d.z; xz += d.x * d.z
            xy += d.x * d.y; zy += d.z * d.y
        }
        let det = xx * zz - xz * xz
        guard det.isFinite, det > 1e-8 else { return nil }
        let a = (xy * zz - zy * xz) / det, b = (zy * xx - xy * xz) / det
        let c = mean.y - a * mean.x - b * mean.z
        return a.isFinite && b.isFinite && c.isFinite ? (a, b, c) : nil
    }
}
