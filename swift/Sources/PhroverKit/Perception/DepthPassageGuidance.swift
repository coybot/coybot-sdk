import Foundation
import RoverNav

/// Steering preference from two observed passage boundaries. This is never
/// motion authorization: the selected wheels still require a fresh clear sweep.
struct DepthPassageGuidance: Equatable, Sendable {
    let centerLateral: Double // positive is rover-right
    let lookAhead: Double
    let width: Double
    static let maximumWheelSpeed = 0.12
    static let centerTolerance = 0.03

    /// A distant gap cannot authorize steering toward a nearer occupied flank.
    func conflicts(with side: DepthObstacleSide?) -> Bool {
        (side == .right && centerLateral >= -Self.centerTolerance) ||
            (side == .left && centerLateral <= Self.centerTolerance)
    }

    func permitsFlankAvoidance(_ command: WheelCommand, side: DepthObstacleSide?) -> Bool {
        // A front/bilateral warning has no reliable escape side. Passage
        // geometry must not veto a bounded arc with an independently clear
        // exact sweep. Driver and native entry still reject caution/stop.
        if side == .front || side == .both {
            return FollowPursuitDriver.isSlowObstacleArc(command)
        }
        guard conflicts(with: side), FollowPursuitDriver.isSlowObstacleArc(command) else { return false }
        return side == .right ? command.right > command.left : command.left > command.right
    }

    func command(maximum: Double) -> WheelCommand {
        let peak = min(Self.maximumWheelSpeed, maximum)
        let offset = abs(centerLateral) <= Self.centerTolerance ? 0 : centerLateral
        // Pure pursuit toward the observed centre. A bearing-proportional wheel
        // split oversteers on this short wheelbase and can sweep into a jamb.
        let curvature = 2 * abs(offset) / (lookAhead * lookAhead + offset * offset)
        let differential = min(0.7, curvature * RoverConfig.wheelBase / 2)
        let ratio = (1 - differential) / (1 + differential)
        return offset > 0 ? .init(left: peak, right: peak * ratio) :
            .init(left: peak * ratio, right: peak)
    }

    /// Revalidate a retained preference against newly observed boundaries.
    /// The collision evaluator independently approves the exact curvature.
    func accepts(_ command: WheelCommand) -> Bool {
        let peak = max(command.left, command.right), inner = min(command.left, command.right)
        guard peak.isFinite, inner.isFinite, peak > 0, peak <= Self.maximumWheelSpeed,
              inner / peak >= 0.15 - 1e-9 else { return false }
        if abs(centerLateral) <= Self.centerTolerance { return abs(command.left - command.right) < 1e-9 }
        return centerLateral > 0 ? command.left > command.right : command.right > command.left
    }

    static func estimate(_ snapshot: DepthSafetySnapshot) -> Self? {
        assess(snapshot).guidance
    }

    struct Assessment {
        let guidance: DepthPassageGuidance?
        let reason: String
        var leftColumns: Int = 0
        var rightColumns: Int = 0
        var boundaryProfile = ""
        var requiredWidth: Double?
    }

    static func assess(_ snapshot: DepthSafetySnapshot) -> Assessment {
        let resolution = snapshot.cellResolution
        guard resolution.isFinite, resolution > 0, resolution <= 0.1,
              snapshot.geometry.validation == .valid else {
            return .init(guidance: nil, reason: "invalid_geometry")
        }
        let sections = snapshot.geometry.collisionSections
        guard let bodyLeft = sections.map({ $0.lateralOffset - $0.width / 2 }).min(),
              let bodyRight = sections.map({ $0.lateralOffset + $0.width / 2 }).max() else {
            return .init(guidance: nil, reason: "missing_body")
        }
        // A local occupied-space envelope, rebuilt from live depth. Boundaries
        // may begin at different distances (e.g. a sofa before a wall jamb).
        // Never interpret absence of occupied cells as proof of free space.
        var columns: [Int: [Int: Set<Int>]] = [:]
        for cell in snapshot.cells {
            let x = (Double(cell.lateralIndex) + 0.5) * resolution
            let y = (Double(cell.heightIndex) + 0.5) * resolution
            let z = (Double(cell.forwardIndex) + 0.5) * resolution
            guard abs(x) <= 1.5, z >= 0.2, z < 2.4,
                  sections.contains(where: { y >= $0.minimumHeight && y <= $0.maximumHeight }) else { continue }
            columns[cell.lateralIndex, default: [:]][cell.forwardIndex, default: []].insert(cell.heightIndex)
        }
        var supported: [Int: Double] = [:]
        for (lateral, depths) in columns {
            let slices = depths.filter { $0.value.count >= 3 }.keys.sorted()
            var start: Int?
            var previous: Int?
            for depth in slices {
                if previous == nil || depth != previous! + 1 { start = depth }
                previous = depth
                if let start, Double(depth - start + 1) * resolution >= 0.2 - 1e-9 {
                    supported[lateral] = (Double(start) + 0.5) * resolution
                    break
                }
            }
        }
        let lefts = supported.keys.filter { $0 < 0 }
        let rights = supported.keys.filter { $0 >= 0 }
        // Bounded replay evidence: lateral position and first supported depth,
        // nearest 16 columns to the chassis centre. No camera pixels retained.
        let profile = supported.keys.sorted { abs($0) < abs($1) }.prefix(16).sorted().map {
            String(format: "%.3f:%.3f", (Double($0) + 0.5) * resolution, supported[$0]!)
        }.joined(separator: ",")
        let requiredWidth = bodyRight - bodyLeft + 2 * (snapshot.geometry.lateralSafetyMargin + 0.15)
        func rejected(_ reason: String) -> Assessment {
            .init(guidance: nil, reason: reason, leftColumns: lefts.count, rightColumns: rights.count,
                boundaryProfile: profile, requiredWidth: requiredWidth)
        }
        let ordered = supported.keys.sorted()
        guard ordered.count >= 2 else {
            return rejected(lefts.isEmpty && rights.isEmpty ? "insufficient_boundary_support" :
                lefts.isEmpty ? "missing_left_boundary" : "missing_right_boundary")
        }
        // Adjacent occupied columns define candidate gaps, including openings
        // entirely to one side of the current heading. Projecting all observed
        // boundaries onto this envelope conservatively retains protrusions.
        let margin = snapshot.geometry.lateralSafetyMargin + 0.15
        var choices: [Self] = []
        var wide = false
        for (leftIndex, rightIndex) in zip(ordered, ordered.dropFirst()) {
            let left = Double(leftIndex + 1) * resolution, right = Double(rightIndex) * resolution
            let minimumCenter = left - bodyLeft + margin
            let maximumCenter = right - bodyRight - margin
            guard minimumCenter <= maximumCenter else { continue }
            guard right - left <= 1.2 else { wide = true; continue }
            let distance = min(supported[leftIndex]!, supported[rightIndex]!)
            choices.append(.init(centerLateral: (minimumCenter + maximumCenter) / 2,
                lookAhead: max(0.5, distance), width: right - left))
        }
        guard let guidance = choices.min(by: { abs($0.centerLateral) < abs($1.centerLateral) }) else {
            return rejected(wide ? "wide_open_space" : "insufficient_width")
        }
        return .init(guidance: guidance, reason: "supported",
            leftColumns: lefts.count, rightColumns: rights.count,
            boundaryProfile: profile, requiredWidth: requiredWidth)
    }
}
