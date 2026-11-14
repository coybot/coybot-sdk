import Foundation
import RoverNav

/// Pursuit planning only. Reactive following uses live depth for each command
/// rather than a reconstructed floor/map, stopping instead of routing around obstacles.
/// Legacy moving readiness retains its mesh preflight; Talk uses stationary spoken readiness.
public enum FollowPlanningMode: String, Sendable {
    case mesh
    case reactiveDepth = "reactive_depth"
}

enum FollowPlanningScope {
    @TaskLocal static var mode: FollowPlanningMode = .mesh
}

enum FollowPlanningPolicy {
    @MainActor
    static func plan(mode: FollowPlanningMode, start: Vec2, goal: Vec2,
                     meshPlan: () async -> FollowPursuitPlan?) async -> FollowPursuitPlan? {
        guard !Task.isCancelled, start.x.isFinite, start.y.isFinite,
              goal.x.isFinite, goal.y.isFinite else { return nil }
        switch mode {
        case .mesh: return await meshPlan()
        case .reactiveDepth:
            // No invented free-space costmap: this is an explicitly unmapped,
            // single-target route whose wheel commands still require live depth.
            return .init(path: [goal], planningMode: .reactiveDepth)
        }
    }
}
