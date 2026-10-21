import Foundation
import RoverNav

@MainActor
public final class NavigationSilentSearchMotion: SilentSearchMotion {
    private let frame: SharedMissionFrame
    private let localPose: () -> Pose2D?
    private let localPath: () -> [Vec2]
    private let navigateAndWait: (Vec2, any PathAdmissibilityPolicy) async -> NavigationResult
    private let rotateAndWait: (Double) async -> NavigationResult
    private let cancelAndWait: () async -> Void
    private var finalPosition: MissionPoint?

    public init(navigation: NavigationController, ar: ARSessionManager, frame: SharedMissionFrame) {
        self.frame = frame
        localPose = { ar.pose }
        localPath = { navigation.path }
        navigateAndWait = { goal, policy in
            await navigation.navigateAndWait(to: goal, policy: policy)
        }
        rotateAndWait = { angle in await navigation.rotateAndWait(by: angle) }
        cancelAndWait = { await navigation.cancelAndWait() }
    }

    init(
        frame: SharedMissionFrame,
        currentPose: @escaping () -> Pose2D?,
        currentPath: @escaping () -> [Vec2],
        navigateAndWait: @escaping (Vec2, any PathAdmissibilityPolicy) async -> NavigationResult,
        rotateAndWait: @escaping (Double) async -> NavigationResult,
        cancelAndWait: @escaping () async -> Void
    ) {
        self.frame = frame
        localPose = currentPose
        localPath = currentPath
        self.navigateAndWait = navigateAndWait
        self.rotateAndWait = rotateAndWait
        self.cancelAndWait = cancelAndWait
    }

    public var currentMissionPose: MissionPose? {
        localPose().flatMap(frame.missionPose(from:))
    }

    public var currentMissionPath: [MissionPoint] {
        localPath().compactMap(frame.missionPoint(from:))
    }

    public func navigate(
        to target: MissionPoint,
        policy: SilentSearchMotionPolicy
    ) async -> SilentSearchMotionResult {
        finalPosition = target
        let pathPolicy: any PathAdmissibilityPolicy = switch policy {
        case .sectorConstrained(let sector): SectorPathPolicy(sector: sector, frame: frame)
        case .unrestrictedConvergence: UnrestrictedPathPolicy()
        }
        return Self.motionResult(from: await navigateAndWait(frame.localPoint(from: target), pathPolicy))
    }

    public func rotate(to heading: Double, tolerance: Double) async -> SilentSearchMotionResult {
        guard heading.isFinite, tolerance.isFinite, tolerance >= 0,
              let pose = currentMissionPose else {
            return .failed(.noPose)
        }
        let result = Self.motionResult(from: await rotateAndWait(normalizeAngle(heading - pose.heading)))
        guard result == .arrived else { return result }
        guard let finalPose = currentMissionPose else { return .failed(.noPose) }
        if let finalPosition,
           hypot(finalPose.position.x - finalPosition.x, finalPose.position.y - finalPosition.y)
            > SilentSearchGeometry.positionTolerance {
            return .failed(.positionToleranceExceeded)
        }
        guard abs(normalizeAngle(finalPose.heading - heading)) <= tolerance else {
            return .failed(.headingToleranceExceeded)
        }
        return .arrived
    }

    public func stop() async {
        await cancelAndWait()
    }

    private static func motionResult(from result: NavigationResult) -> SilentSearchMotionResult {
        switch result {
        case .arrived: .arrived
        case .cancelled: .cancelled
        case .failed(let failure):
            switch failure {
            case .noPose: .failed(.noPose)
            case .noPath: .failed(.noPath)
            case .pathRejected(let violation): .failed(.pathRejected(violation))
            case .obstacle: .failed(.obstacle)
            case .commsLost, .commandFailed: .failed(.commandLink)
            case .tipping: .failed(.tipping)
            case .stalled: .failed(.stalled)
            case .rotationResolutionInsufficient: .failed(.rotationResolutionInsufficient)
            case .trackingLost: .failed(.tracking)
            case .cancelled: .cancelled
            }
        }
    }
}
