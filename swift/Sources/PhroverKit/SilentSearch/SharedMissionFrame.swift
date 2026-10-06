import Foundation
import RoverNav

public enum RoverRole: String, Codable, CaseIterable, Sendable {
    case a
    case b

    public var searchSector: SearchSector {
        switch self {
        case .a: .west
        case .b: .east
        }
    }
}

public enum SearchSector: String, Codable, CaseIterable, Sendable {
    case west
    case east
}

public struct MissionPoint: Equatable, Sendable {
    public let x: Double
    public let y: Double

    public init?(x: Double, y: Double) {
        guard x.isFinite, y.isFinite else { return nil }
        self.x = x
        self.y = y
    }
}

public struct MissionPose: Equatable, Sendable {
    public let position: MissionPoint
    public let heading: Double

    public init?(position: MissionPoint, heading: Double) {
        guard heading.isFinite else { return nil }
        self.position = position
        self.heading = normalizeAngle(heading)
    }
}

public struct SharedMissionFrame: Equatable, Sendable {
    public let localOrigin: Vec2
    public let localNorthHeading: Double
    public let sessionGeneration: UInt64

    public init?(localOrigin: Vec2, localNorthHeading: Double, sessionGeneration: UInt64) {
        guard localOrigin.x.isFinite, localOrigin.y.isFinite, localNorthHeading.isFinite else {
            return nil
        }
        self.localOrigin = localOrigin
        self.localNorthHeading = normalizeAngle(localNorthHeading)
        self.sessionGeneration = sessionGeneration
    }

    public func localPoint(from point: MissionPoint) -> Vec2 {
        let north = Vec2(cos(localNorthHeading), sin(localNorthHeading))
        let east = Vec2(north.y, -north.x)
        return localOrigin + east * point.x + north * point.y
    }

    public func localPose(from pose: MissionPose) -> Pose2D {
        Pose2D(
            position: localPoint(from: pose.position),
            yaw: normalizeAngle(localNorthHeading + pose.heading)
        )
    }

    public func missionPoint(from point: Vec2) -> MissionPoint? {
        guard point.x.isFinite, point.y.isFinite else { return nil }
        let offset = point - localOrigin
        let north = Vec2(cos(localNorthHeading), sin(localNorthHeading))
        let east = Vec2(north.y, -north.x)
        return MissionPoint(
            x: offset.x * east.x + offset.y * east.y,
            y: offset.x * north.x + offset.y * north.y
        )
    }

    public func missionPose(from pose: Pose2D) -> MissionPose? {
        guard let position = missionPoint(from: pose.position) else { return nil }
        return MissionPose(position: position, heading: pose.yaw - localNorthHeading)
    }

    public func isValid(forSessionGeneration generation: UInt64) -> Bool {
        sessionGeneration == generation
    }
}

public enum SilentSearchGeometry {
    public static let centerBandHalfWidth = 0.25
    public static let targetOffset = 0.60
    public static let positionTolerance = 0.20
    public static let headingTolerance = 10.0 * .pi / 180.0

    public static func rendezvousPoint(for role: RoverRole) -> MissionPoint {
        switch role {
        case .a: MissionPoint(x: -0.60, y: -0.80)!
        case .b: MissionPoint(x: 0.60, y: -0.80)!
        }
    }
}
