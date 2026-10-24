import Foundation
import RoverNav
@testable import PhroverKit

@MainActor
final class ManualFollowClock: FollowMeClock {
    private var time: TimeInterval = 0
    var onNextRead: (() -> Void)?
    var now: TimeInterval {
        let callback = onNextRead
        onNextRead = nil
        callback?()
        return time
    }
    private var sleepers: [(TimeInterval, CheckedContinuation<Void, Never>)] = []

    func sleep(seconds: Double) async {
        await withCheckedContinuation { sleepers.append((now + seconds, $0)) }
    }

    func advance(to time: TimeInterval, wakeSleepers: Bool = true) {
        self.time = time
        guard wakeSleepers else { return }
        let ready = sleepers.filter { $0.0 <= time }
        sleepers.removeAll { $0.0 <= time }
        for (_, continuation) in ready { continuation.resume() }
    }
}

@MainActor
final class FollowPerceptionFake: FollowMePerception {
    var detectorReady = true
    var personLabelAvailable = true
    private var continuation: AsyncStream<FollowPerceptionEvent>.Continuation?
    func events() -> AsyncStream<FollowPerceptionEvent> {
        AsyncStream { continuation = $0 }
    }
    func send(_ event: FollowPerceptionEvent) { continuation?.yield(event) }
}

@MainActor
final class FollowMotionFake: FollowMeSourceClockMotion {
    private var syntheticClock: ManualFollowClock?
    var sourceUptime: TimeInterval? { syntheticClock?.now }
    init(clock: ManualFollowClock? = nil) { syntheticClock = clock }
    func useSourceClock(_ clock: ManualFollowClock) { syntheticClock = clock }
    private(set) var readySignals = 0
    var suspendReadySignal = false
    var readySignalResult: NavigationResult = .arrived
    private var readyWaiter: CheckedContinuation<Void, Never>?
    func signalReady() async -> NavigationResult {
        readySignals += 1
        if suspendReadySignal { await withCheckedContinuation { readyWaiter = $0 } }
        return readySignalResult
    }
    func releaseReadySignal() { readyWaiter?.resume(); readyWaiter = nil }
    private(set) var rotations: [Double] = []
    private(set) var alignments: [Double] = []
    var suspendAlignment = false
    var alignmentResult: NavigationResult = .arrived
    private var alignmentWaiters: [CheckedContinuation<Void, Never>] = []
    func alignTowardPerson(by angle: Double) async -> NavigationResult {
        alignments.append(angle)
        if suspendAlignment { await withCheckedContinuation { alignmentWaiters.append($0) } }
        return alignmentResult
    }
    func releaseAlignment() {
        let waiters = alignmentWaiters
        alignmentWaiters = []
        for waiter in waiters { waiter.resume() }
    }
    private(set) var goals: [Vec2] = []
    private(set) var clearances: [Double] = []
    private(set) var stops = 0
    private(set) var stopOrigins: [FollowMotionStopOrigin] = []
    var stopError = false
    var suspendStop = false
    var suspendRotation = false
    var suspendNavigation = false
    private var stopWaiter: CheckedContinuation<Void, Never>?
    private var rotationWaiter: CheckedContinuation<Void, Never>?
    private var navigationWaiter: CheckedContinuation<Void, Never>?
    private var safetyContinuation: AsyncStream<NavigationSafetyState>.Continuation?

    func rotateForScan(by angle: Double) async -> NavigationResult {
        rotations.append(angle)
        if suspendRotation { await withCheckedContinuation { rotationWaiter = $0 } }
        return .arrived
    }
    func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult {
        goals.append(goal)
        clearances.append(clearance)
        if suspendNavigation { await withCheckedContinuation { navigationWaiter = $0 } }
        return .arrived
    }
    func stopAndConfirm() async throws {
        stopOrigins.append(FollowMotionTaskScope.stopOrigin)
        stops += 1
        if suspendStop { await withCheckedContinuation { stopWaiter = $0 } }
        if stopError { throw StopError.unconfirmed }
        if let syntheticClock {
            FollowMotionTaskScope.stopReceiptCapture?.recordSourceFence(.init(identity: UUID(),
                operationGeneration: 0, context: .unknown, acknowledgementUptime: syntheticClock.now,
                sourceGeneration: 1, highestSequence: nil, highestSourceTimestamp: nil))
        }
    }
    func releaseStop() { stopWaiter?.resume(); stopWaiter = nil }
    func releaseRotation() { rotationWaiter?.resume(); rotationWaiter = nil }
    func releaseNavigation() { navigationWaiter?.resume(); navigationWaiter = nil }
    func safetyStates() -> AsyncStream<NavigationSafetyState> {
        AsyncStream { safetyContinuation = $0 }
    }
    func safety(_ state: NavigationSafetyState) { safetyContinuation?.yield(state) }
    private enum StopError: Error { case unconfirmed }
}

/// A contextual companion over the legacy fake; controller facts remain unknown.
@MainActor
final class ContextualFollowMotionFake: FollowMeContextualMotion {
    let legacy = FollowMotionFake()
    var sourceUptime: TimeInterval? { legacy.sourceUptime }
    private(set) var requests: [FollowMotionRequest] = []
    private(set) var contexts: [FollowMotionRequestContext] = []
    private(set) var legacySubscriptions = 0
    private(set) var contextualSubscriptions = 0
    private var failureContinuation: AsyncStream<FollowMotionFailureDelivery>.Continuation?
    var resultOverride: FollowMotionResult?

    func perform(_ request: FollowMotionRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        requests.append(request)
        contexts.append(context)
        let result = await legacy.performContextual(request, context: context)
        return resultOverride ?? result
    }
    func motionFailures() -> AsyncStream<FollowMotionFailureDelivery> {
        contextualSubscriptions += 1
        return AsyncStream { failureContinuation = $0 }
    }
    func sendFailure(_ failure: FollowMotionFailureDelivery) { failureContinuation?.yield(failure) }
    func safetyStates() -> AsyncStream<NavigationSafetyState> {
        legacySubscriptions += 1
        return legacy.safetyStates()
    }
    func rotateForScan(by angle: Double) async -> NavigationResult { await legacy.rotateForScan(by: angle) }
    func alignTowardPerson(by angle: Double) async -> NavigationResult { await legacy.alignTowardPerson(by: angle) }
    func signalReady() async -> NavigationResult { await legacy.signalReady() }
    func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult {
        await legacy.navigate(to: goal, stoppingAtForwardClearance: clearance)
    }
    func stopAndConfirm() async throws { try await legacy.stopAndConfirm() }
}

/// A contextual motion whose follow drive can be steered in flight, like the real controller.
/// `navigate` suspends while the drive is "running"; `updateFollowGoal` is accepted only then.
@MainActor
final class LiveGoalFollowMotionFake: FollowMeContextualMotion {
    let legacy = FollowMotionFake()
    var sourceUptime: TimeInterval? { legacy.sourceUptime }
    private(set) var liveGoals: [Vec2] = []
    /// When false the fake refuses live updates, like a controller that is not driving.
    var acceptsLiveGoals = true

    init() { legacy.suspendNavigation = true }

    func perform(_ request: FollowMotionRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        await legacy.performContextual(request, context: context)
    }
    func updateFollowGoal(_ goal: Vec2) -> Bool {
        guard acceptsLiveGoals, !legacy.goals.isEmpty else { return false }
        liveGoals.append(goal)
        return true
    }
    func safetyStates() -> AsyncStream<NavigationSafetyState> { legacy.safetyStates() }
    func rotateForScan(by angle: Double) async -> NavigationResult { await legacy.rotateForScan(by: angle) }
    func alignTowardPerson(by angle: Double) async -> NavigationResult { await legacy.alignTowardPerson(by: angle) }
    func signalReady() async -> NavigationResult { await legacy.signalReady() }
    func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult {
        await legacy.navigate(to: goal, stoppingAtForwardClearance: clearance)
    }
    func stopAndConfirm() async throws { try await legacy.stopAndConfirm() }
}
