import RoverNav

public enum PathPolicyViolation: Equatable, Sendable {
    case invalidPoint(pointIndex: Int)
    case outsideSector(pointIndex: Int)
}

public enum PathAdmissibilityResult: Equatable, Sendable {
    case admissible
    case rejected(PathPolicyViolation)
}

public protocol PathAdmissibilityPolicy: Sendable {
    func evaluate(path: [Vec2]) -> PathAdmissibilityResult
}

public enum NavigationFailure: Equatable, Sendable {
    case noPose
    case noPath
    case pathRejected(PathPolicyViolation)
    case obstacle
    case commsLost
    case tipping
    case stalled
    case commandFailed
    case trackingLost
    case cancelled
}

public enum NavigationResult: Equatable, Sendable {
    case arrived
    case failed(NavigationFailure)
    case cancelled
}

public struct UnrestrictedPathPolicy: PathAdmissibilityPolicy {
    public init() {}

    public func evaluate(path: [Vec2]) -> PathAdmissibilityResult {
        for (index, point) in path.enumerated() where !point.x.isFinite || !point.y.isFinite {
            return .rejected(.invalidPoint(pointIndex: index))
        }
        return .admissible
    }
}
