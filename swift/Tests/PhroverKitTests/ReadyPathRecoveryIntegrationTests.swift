import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class ReadyPathRecoveryIntegrationTests: XCTestCase {
    func testStoppedRefreshRequiresBothFreshDepthAndFreshTargetBeforeFirstPulse() async {
        let fixture = RecoveryControllerFixture()
        fixture.rawDepthTimestamp = 10 - 0.272
        fixture.freshDepthAt = 10.30
        fixture.perceptionFreshAt = 10.45
        fixture.beforeSend = { XCTAssertGreaterThanOrEqual(fixture.time, 10.45) }
        let result = await fixture.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertFalse(fixture.commands.isEmpty)
    }

    func testFreshDepthCannotRenewWaitForMissingPerceptionOrUseDeadlineFrame() async {
        for freshAt in [10.601, Double.infinity] {
            let fixture = RecoveryControllerFixture()
            fixture.rawDepthTimestamp = 9.7
            fixture.freshDepthAt = 10.30
            fixture.perceptionFreshAt = freshAt
            let result = await fixture.run()
            XCTAssertEqual(result.result, .cancelled)
            XCTAssertTrue(fixture.commands.isEmpty)
            XCTAssertLessThan(fixture.time, 10.62)
        }
    }

    func testTargetExpiryInSenderQueueCannotAuthorizeFirstProbe() async {
        let fixture = RecoveryControllerFixture()
        fixture.observationLifetime = 0.01
        fixture.beforeSend = { fixture.time += 0.02 }
        let result = await fixture.run()
        XCTAssertNotEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testDelayedAcknowledgementCannotExtendStoppedRefreshDeadline() async {
        let fixture = RecoveryControllerFixture()
        fixture.onAck = {
            if fixture.boundaries.contains(.recoveryProbeSend) { fixture.time += 0.7 }
        }
        let result = await fixture.run()
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testRealControllerWaitsForFreshStartupDepthThenCompletesReady() async {
        let fixture = RecoveryControllerFixture()
        fixture.rawDepthTimestamp = 10 - 0.27152120834216475
        fixture.freshDepthAt = 10.11
        fixture.beforeSend = { XCTAssertGreaterThanOrEqual(fixture.time, 10.11) }
        let result = await fixture.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(fixture.boundaries, [.recoveryProbe, .recoveryProbeSend, .recoveryProbeSend, .controllerFirstSend])
        XCTAssertGreaterThanOrEqual(fixture.position.x, 0.08)
        XCTAssertLessThanOrEqual(fixture.position.x, 0.12)
    }

    func testStartupRefreshCannotBypassFinalSenderDepthExpiry() async {
        let fixture = RecoveryControllerFixture()
        fixture.rawDepthTimestamp = 9.7
        fixture.freshDepthAt = 10.11
        fixture.onDepth = {
            if FollowTurnBurstTransportScope.authorization != nil {
                fixture.rawDepthTimestamp = fixture.time - 0.27
                fixture.freshDepthAt = nil
            }
        }
        let result = await fixture.run()
        XCTAssertEqual(result.result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testOwnerInhibitionDuringDepthWaitCannotSendLateFreshProbe() async {
        let fixture = RecoveryControllerFixture()
        fixture.rawDepthTimestamp = 9.7
        fixture.freshDepthAt = 10.05
        fixture.onWait = { fixture.controller.inhibitForHandoff() }
        let result = await fixture.run()
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testNormalReadyUsesBoundedCoverageWithoutRequiringMapFailure() async {
        let fixture = RecoveryControllerFixture(mappedObstacle: false)
        fixture.requiresProbeScope = true
        let result = await fixture.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(fixture.boundaries, [.controllerFirstSend])
        XCTAssertFalse(fixture.commands.isEmpty)
        XCTAssertGreaterThanOrEqual(fixture.position.x, 0.08)
        XCTAssertLessThanOrEqual(fixture.position.x, 0.12)
        XCTAssertTrue(fixture.depthScopes.allSatisfy { $0 })
        let sent = fixture.commands.count
        for purpose in [FollowMotionPurpose.followReady, .followGoal] {
            let ordinary = await fixture.controller.sendFollowTurnBurst(.init(left: 0.25, right: 0.25),
                requestedBudget: 0.02, purpose: purpose)
            XCTAssertNotNil(ordinary.result.failure)
            XCTAssertEqual(fixture.commands.count, sent)
            XCTAssertEqual(fixture.depthScopes.last, false)
        }
    }

    func testProbeAssessmentSenderAndRemainingReadyShareScopedCoverageButLaterMotionDoesNot() async {
        let fixture = RecoveryControllerFixture()
        fixture.requiresProbeScope = true
        let result = await fixture.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(fixture.boundaries, [.recoveryProbe, .recoveryProbeSend, .recoveryProbeSend, .controllerFirstSend])
        XCTAssertFalse(fixture.depthScopes.isEmpty)
        XCTAssertTrue(fixture.depthScopes.allSatisfy { $0 })
        let sent = fixture.commands.count
        let later = await fixture.controller.sendFollowTurnBurst(.init(left: 0.25, right: 0.25),
            requestedBudget: 0.02, purpose: .followReady)
        XCTAssertNotNil(later.result.failure)
        XCTAssertEqual(fixture.commands.count, sent)
        XCTAssertEqual(fixture.depthScopes.last, false)
    }
    func testMappedFalsePositiveProbesThenCompletesOriginalReadyDistance() async {
        let fixture = RecoveryControllerFixture()
        let result = await fixture.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(fixture.boundaries, [.recoveryProbe, .recoveryProbeSend, .recoveryProbeSend, .controllerFirstSend])
        XCTAssertGreaterThanOrEqual(fixture.position.x, 0.08)
        XCTAssertLessThanOrEqual(fixture.position.x, 0.12, "Probe travel is part of the original ready distance")
        XCTAssertTrue(fixture.commands.allSatisfy { $0.left > 0 && $0.right > 0 })
    }

    func testRealObstacleRetreatsThenTurnsAndRequestsFreshPersonObservation() async {
        let fixture = RecoveryControllerFixture()
        fixture.obstacleAfter = 0.02
        let result = await fixture.run()
        XCTAssertEqual(result.result, .reobservationRequired)
        XCTAssertNil(result.failure)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(fixture.boundaries, [.recoveryProbe, .recoveryProbeSend, .recoveryProbeSend], "A recovery is not a consumed ready signal")
        XCTAssertTrue(fixture.commands.contains { $0.left < 0 && $0.left == $0.right })
        XCTAssertTrue(fixture.commands.contains { $0.left == -$0.right })
        XCTAssertGreaterThanOrEqual(fixture.minimumX, -0.005)
        XCTAssertLessThanOrEqual(fixture.maximumX, 0.05)
    }

    func testFailedProbeStopCannotAuthorizeReverseOrSearch() async {
        let fixture = RecoveryControllerFixture()
        fixture.failStopAfterMovement = true
        let result = await fixture.run()
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .failed)
        XCTAssertEqual(fixture.commands.count, 1)
    }

    func testUnknownDepthNeverProbesMappedObstruction() async {
        let fixture = RecoveryControllerFixture()
        fixture.unavailable = true
        let result = await fixture.run()
        XCTAssertEqual(result.result, .failed(.depthUnavailable(.missingRawDepth)))
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testOwnerCancellationBeforeTransportCannotSendProbeOrBacktrack() async {
        let fixture = RecoveryControllerFixture()
        fixture.beforeSend = { fixture.controller.inhibitForHandoff() }
        let result = await fixture.run()
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testSourceResetAfterProbeCannotAuthorizeBacktrack() async {
        let fixture = RecoveryControllerFixture()
        fixture.obstacleAfter = 0.02
        fixture.onDepth = {
            if fixture.commands.count == 2 { fixture.generation = 2 }
        }
        let result = await fixture.run()
        XCTAssertEqual(result.result, .failed(.trackingLost))
        XCTAssertEqual(fixture.commands.count, 2)
        XCTAssertTrue(fixture.commands.allSatisfy { $0.left > 0 && $0.right > 0 })
    }
}

@MainActor
private final class RecoveryControllerFixture {
    private let transportClock = RecoveryTransportClock()
    var time: Double {
        get { transportClock.now }
        set { transportClock.now = newValue }
    }
    var sequence: UInt64 = 1
    var generation: UInt64 = 1
    var position = Vec2.zero
    var yaw = 0.0
    var minimumX = 0.0
    var maximumX = 0.0
    var commands: [WheelCommand] = []
    var boundaries: [FollowReadyAdmissionBoundary] = []
    var obstacleAfter = Double.infinity
    var blocked = false
    var unavailable = false
    var failStopAfterMovement = false
    var controller: FollowNavigationController!
    var beforeSend: (() -> Void)?
    var onDepth: (() -> Void)?
    var requiresProbeScope = false
    var depthScopes: [Bool] = []
    var rawDepthTimestamp: Double?
    var freshDepthAt: Double?
    var onWait: (() -> Void)?
    var onAck: (() -> Void)?
    var perceptionFreshAt = -Double.infinity
    var observationLifetime = Double.infinity
    var snapshot: NavigationPoseSample {
        .init(pose: .init(position: position, yaw: yaw), frameID: .init(generation: generation, sequence: sequence),
            sourceTimestamp: time - 0.001, trackingQuality: .normal)
    }
    init(mappedObstacle: Bool = true) {
        let transportClock = self.transportClock
        var map = Costmap(width: 120, height: 120, resolution: 0.10, origin: .init(-6, -6))
        let cell = map.worldToCell(.zero)
        if mappedObstacle { map.setCost(cell.cx, cell.cy, 10) }
        let capturedMap = map
        controller = FollowNavigationController(currentPose: { self.snapshot.pose }, forwardClearance: { 2 },
            depthSafetyObservation: { command in
                self.onDepth?()
                self.depthScopes.append(ReadyMotionDepthScope.active)
                if let timestamp = self.rawDepthTimestamp,
                   self.freshDepthAt.map({ self.time < $0 }) ?? true {
                    return .unavailable(.staleRawDepth, sampleAge: self.time - timestamp, motionClass: .classify(command))
                }
                if self.requiresProbeScope && !ReadyMotionDepthScope.active {
                    return .unavailable(.blindSweptVolume, sampleAge: 0.01, motionClass: .classify(command))
                }
                if self.unavailable { return .unavailable(.missingRawDepth, sampleAge: 0, motionClass: .classify(command)) }
                if self.position.x >= self.obstacleAfter { self.blocked = true }
                let obstacle = self.blocked && self.yaw < 0.3
                // Reverse deliberately has no rear depth: only the scoped return
                // authority may permit it, never the ordinary depth provider.
                if command.left < 0, command.left == command.right {
                    return .unavailable(.blindSweptVolume, sampleAge: 0, motionClass: .reverse)
                }
                return .init(state: obstacle ? .stop : .clear, clearance: obstacle ? 0.2 : 2,
                    supportCount: obstacle ? 10 : 0, sampleAge: 0.01, requiredStoppingDistance: 0.3,
                    motionClass: .classify(command), speedLimit: nil)
            }, plan: { _, goal in [goal] }, readySignalCostmap: { _ in capturedMap },
            lastAckAt: { self.onAck?(); return Date(timeIntervalSince1970: self.time) }, sendCommand: { command in
                self.beforeSend?()
                if let authority = FollowTurnBurstTransportScope.authorization, !authority.isAuthorized() {
                    throw FollowTurnBurstTransportDenial.fenced
                }
                self.commands.append(command)
                if command.left == command.right {
                    self.position.x += command.left > 0 ? 0.01 : -0.01
                } else { self.yaw += 0.15 }
                self.minimumX = min(self.minimumX, self.position.x)
                self.maximumX = max(self.maximumX, self.position.x)
                self.time += 0.03
                self.sequence += 1
            }, stopRover: {
                if self.failStopAfterMovement, !self.commands.isEmpty { throw URLError(.timedOut) }
                self.time += 0.001
            }, sleep: { duration in
                self.time += max(0.001, Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18)
                self.sequence += 1
                self.controller.ingestFollowTurnSource(self.snapshot)
                self.onWait?()
                await Task.yield()
            }, now: { Date(timeIntervalSince1970: self.time) }, poseSample: { self.snapshot },
            sourceNow: { self.time }, sourceStopSnapshot: { self.snapshot }, transportUptime: { transportClock.now })
    }
    func run() async -> FollowMotionResult {
        let admission = FollowReadyAdmission(allowsPathRecovery: true, validating: { _, _ in
            if let boundary = FollowReadyAdmissionScope.current?.boundary { self.boundaries.append(boundary) }
            if FollowReadyAdmissionScope.current?.boundary == .recoveryProbeSend {
                if self.time < self.perceptionFreshAt {
                    FollowReadyAdmissionScope.current?.rejectionCondition = "frame_stale"
                    return .staleObservation
                }
                FollowReadyAdmissionScope.current?.observationValidUntil = self.time + self.observationLifetime
            }
            return .accepted
        })
        return await NavigationFollowMeMotion(navigation: controller).performContextual(.ready,
            context: .init(sessionGeneration: 1, requestToken: 1, purpose: .followReady, phase: "aligning"),
            admission: admission)
    }
}

private final class RecoveryTransportClock: @unchecked Sendable {
    private let lock = NSLock()
    private var value = 10.0
    var now: Double {
        get { lock.withLock { value } }
        set { lock.withLock { value = newValue } }
    }
}
