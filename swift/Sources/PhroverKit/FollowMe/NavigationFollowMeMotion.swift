import Foundation
import RoverNav

/// Shared adapter surface for the public owner and the isolated follow runtime.
/// Production construction remains through NavigationController so motion ownership
/// is serialized with generic navigation. The internal initializer supports runtime tests.
@MainActor
protocol FollowNavigationRuntime: AnyObject {
    func performFollowMotion(_ request: FollowMotionRequest, context: FollowMotionRequestContext?) async -> FollowMotionResult
    var followSourceUptime: TimeInterval { get }
    func stopAndConfirm() async throws
    func safetyStates() -> AsyncStream<NavigationSafetyState>
    func readFollowPose() -> NavigationPoseSample
    func followMotionFailures() -> AsyncStream<FollowMotionFailureDelivery>
    func inhibitFollowScanContinuation(origin: FollowMotionStopOrigin)
    func updateFollowGoal(_ goal: Vec2) -> Bool
}

extension NavigationController: FollowNavigationRuntime {}
extension FollowNavigationController: FollowNavigationRuntime {}

@MainActor
public final class NavigationFollowMeMotion: FollowMeContextualMotion, FollowMeAbsoluteHeadingMotion {
    private let navigation: any FollowNavigationRuntime

    public init(navigation: NavigationController) { self.navigation = navigation }
    init(navigation: FollowNavigationController) { self.navigation = navigation }

    public func rotateForScan(by angle: Double) async -> NavigationResult {
        await navigation.performFollowMotion(.scan(angle), context: nil).result
    }

    public func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult {
        await navigation.performFollowMotion(.following(goal, clearance), context: nil).result
    }

    public func alignTowardPerson(by angle: Double) async -> NavigationResult {
        await navigation.performFollowMotion(.alignment(angle), context: nil).result
    }

    var sourceUptime: TimeInterval? { navigation.followSourceUptime }

    public func stopAndConfirm() async throws {
        try await FollowTurnSourceScope.$required.withValue(true) { try await navigation.stopAndConfirm() }
    }

    public func signalReady() async -> NavigationResult {
        await navigation.performFollowMotion(.ready, context: nil).result
    }

    public func safetyStates() -> AsyncStream<NavigationSafetyState> { navigation.safetyStates() }

    func perform(_ request: FollowMotionRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        await navigation.performFollowMotion(request, context: context)
    }

    func performRecovery(_ request: FollowRecoveryHeadingRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        await FollowRecoveryScope.$authorization.withValue(request.authorization) {
            await FollowRecoveryScope.$heading.withValue(request) {
                await navigation.performFollowMotion(.scan(0), context: context)
            }
        }
    }

    func recoveryPoseSample() -> NavigationPoseSample { navigation.readFollowPose() }

    func motionFailures() -> AsyncStream<FollowMotionFailureDelivery> { navigation.followMotionFailures() }

    func inhibitScanContinuation(origin: FollowMotionStopOrigin) {
        navigation.inhibitFollowScanContinuation(origin: origin)
    }

    func updateFollowGoal(_ goal: Vec2) -> Bool { navigation.updateFollowGoal(goal) }
}
