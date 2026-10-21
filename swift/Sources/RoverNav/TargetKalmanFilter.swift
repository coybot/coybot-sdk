import Foundation
import simd

/// Constant-velocity Kalman filter for a walking person on the nav plane.
///
/// State is `[x, y, vx, vy]` in metres and metres per second, in the same frame as `Pose2D`.
/// It exists so a follower can keep an estimate through a short detection dropout (a person
/// passing behind a pillar, a missed frame) instead of treating every miss as a lost target.
///
/// Pure and value-typed: no clock, no I/O. Callers own time and pass it explicitly, which
/// keeps it deterministic to unit-test and safe to copy for look-ahead gating.
public struct TargetKalmanFilter: Sendable, Equatable {
    /// Process noise: how hard the person may accelerate or turn between frames (m/s^2).
    public static let defaultAccelerationSigma = 0.5

    public struct Estimate: Sendable, Equatable {
        public let position: Vec2
        public let velocity: Vec2
        /// 1-sigma position uncertainty (m), the larger principal axis.
        public let positionSigma: Double
    }

    /// Time of the current state; measurements must not predate it by construction.
    public private(set) var time: TimeInterval
    public var accelerationSigma: Double

    private var x: SIMD4<Double>
    private var p: simd_double4x4

    public init(position: Vec2, at time: TimeInterval, positionSigma: Double = 0.10,
                velocitySigma: Double = 0.5, accelerationSigma: Double = defaultAccelerationSigma) {
        self.time = time
        self.accelerationSigma = accelerationSigma
        x = SIMD4(position.x, position.y, 0, 0)
        p = simd_double4x4(diagonal: SIMD4(positionSigma * positionSigma, positionSigma * positionSigma,
                                           velocitySigma * velocitySigma, velocitySigma * velocitySigma))
    }

    // MARK: Prediction

    /// Advance the state to `t`. Times at or before the current state are ignored.
    public mutating func predict(to t: TimeInterval) {
        let dt = t - time
        guard dt > 0, dt.isFinite else { return }
        let (f, q) = transition(dt)
        x = f * x
        p = f * p * f.transpose + q
        time = t
    }

    /// Where the person is expected to be at `t`, without changing the filter.
    public func estimate(at t: TimeInterval) -> Estimate {
        var copy = self
        copy.predict(to: t)
        return copy.currentEstimate
    }

    public var currentEstimate: Estimate {
        let sxx = p[0][0], syy = p[1][1], sxy = p[1][0]
        let mean = (sxx + syy) / 2
        let radius = (((sxx - syy) / 2) * ((sxx - syy) / 2) + sxy * sxy).squareRoot()
        return Estimate(position: Vec2(x[0], x[1]), velocity: Vec2(x[2], x[3]),
                        positionSigma: max(0, mean + radius).squareRoot())
    }

    // MARK: Position measurements

    /// Squared Mahalanobis distance of a position measurement. 9.21 is the 99% gate for 2 dof.
    public func gate(position z: Vec2, sigma: Double) -> Double {
        let s = innovationCovariance(sigma: sigma)
        let dx = z.x - x[0], dy = z.y - x[1]
        let det = s.a * s.d - s.b * s.b
        guard det > 1e-12 else { return .infinity }
        return (s.d * dx * dx - 2 * s.b * dx * dy + s.a * dy * dy) / det
    }

    public mutating func update(position z: Vec2, sigma: Double) {
        let s = innovationCovariance(sigma: sigma)
        let det = s.a * s.d - s.b * s.b
        guard det > 1e-12 else { return }
        // K = P Hᵀ S⁻¹, with H selecting x and y.
        let inv = (a: s.d / det, b: -s.b / det, d: s.a / det)
        var k = simd_double2x4()                 // 4 rows, 2 columns (simd naming is columns x rows)
        for row in 0..<4 {
            let px = p[0][row], py = p[1][row]   // p is symmetric: column == row
            k[0][row] = px * inv.a + py * inv.b
            k[1][row] = px * inv.b + py * inv.d
        }
        let innovation = SIMD2(z.x - x[0], z.y - x[1])
        x += k * innovation
        var h = simd_double4x2()                 // 2 rows, 4 columns, selecting x and y
        h[0][0] = 1; h[1][1] = 1
        p = (matrix_identity_double4x4 - k * h) * p
        symmetrize()
    }

    // MARK: Bearing-only measurements

    /// Absolute bearing (rad, `atan2(dy, dx)`) from `origin` to the filter's current position.
    public func predictedBearing(from origin: Vec2) -> Double? { bearingModel(from: origin)?.0 }

    /// Signed difference `measured - predicted`, wrapped to (-pi, pi].
    public func bearingInnovation(_ bearing: Double, from origin: Vec2) -> Double? {
        predictedBearing(from: origin).map { Self.wrap(bearing - $0) }
    }

    /// Squared Mahalanobis distance of an absolute bearing (rad, `atan2(dy, dx)` on the nav
    /// plane) taken from `origin`. 6.63 is the 99% gate for 1 dof.
    public func gate(bearing: Double, from origin: Vec2, sigma: Double) -> Double {
        guard let (h, jacobian) = bearingModel(from: origin) else { return .infinity }
        let s = simd_dot(jacobian, p * jacobian) + sigma * sigma
        guard s > 1e-12 else { return .infinity }
        let y = Self.wrap(bearing - h)
        return y * y / s
    }

    /// Correct the direction to the person without learning their range; the filter's range
    /// uncertainty stays what it was. This is what keeps a track alive when only part of the
    /// body is visible and a depth reading would land on whatever is in front of them.
    public mutating func update(bearing: Double, from origin: Vec2, sigma: Double) {
        guard let (h, jacobian) = bearingModel(from: origin) else { return }
        let s = simd_dot(jacobian, p * jacobian) + sigma * sigma
        guard s > 1e-12 else { return }
        let k = (p * jacobian) / s
        x += k * Self.wrap(bearing - h)
        let kh = simd_double4x4(rows: [k[0] * jacobian, k[1] * jacobian, k[2] * jacobian, k[3] * jacobian])
        p = (matrix_identity_double4x4 - kh) * p
        symmetrize()
    }

    // MARK: Internals

    private struct Symmetric2 { var a: Double, b: Double, d: Double }

    private func innovationCovariance(sigma: Double) -> Symmetric2 {
        Symmetric2(a: p[0][0] + sigma * sigma, b: p[1][0], d: p[1][1] + sigma * sigma)
    }

    private func transition(_ dt: Double) -> (simd_double4x4, simd_double4x4) {
        var f = matrix_identity_double4x4
        f[2][0] = dt   // x += vx * dt  (column 2, row 0)
        f[3][1] = dt   // y += vy * dt
        let q = accelerationSigma * accelerationSigma
        var m = simd_double4x4()
        for axis in 0..<2 {
            m[axis][axis] = q * pow(dt, 4) / 4
            m[axis + 2][axis] = q * pow(dt, 3) / 2
            m[axis][axis + 2] = q * pow(dt, 3) / 2
            m[axis + 2][axis + 2] = q * dt * dt
        }
        return (f, m)
    }

    private func bearingModel(from origin: Vec2) -> (Double, SIMD4<Double>)? {
        let dx = x[0] - origin.x, dy = x[1] - origin.y
        let r2 = dx * dx + dy * dy
        guard r2 > 1e-6 else { return nil }
        return (atan2(dy, dx), SIMD4(-dy / r2, dx / r2, 0, 0))
    }

    private mutating func symmetrize() {
        for i in 0..<4 {
            for j in (i + 1)..<4 {
                let mean = (p[j][i] + p[i][j]) / 2
                p[j][i] = mean
                p[i][j] = mean
            }
        }
    }

    static func wrap(_ angle: Double) -> Double {
        var a = angle.truncatingRemainder(dividingBy: 2 * .pi)
        if a > .pi { a -= 2 * .pi } else if a < -.pi { a += 2 * .pi }
        return a
    }
}
