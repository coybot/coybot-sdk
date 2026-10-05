import Foundation
import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class RotationCommandTests: XCTestCase {
    func testCancellingScanPreflightAwaitsFreshTransportStop() async {
        TestURLProtocol.reset()
        TestURLProtocol.delay = 0.08
        let headingStarted = expectation(description: "scan heading preflight started")
        let ar = ARSessionManager { event, _ in
            if event == "relative_heading_measurement_started" {
                headingStarted.fulfill()
            }
        }
        ingestTrackedPose(into: ar)
        let navigation = makeNavigation(ar: ar)
        let scan = Task { await navigation.rotateForScan(by: .pi / 6) }
        await fulfillment(of: [headingStarted], timeout: 1)
        let clock = ContinuousClock()

        let elapsed = await clock.measure {
            scan.cancel()
            await scan.value
        }

        XCTAssertGreaterThanOrEqual(elapsed, .milliseconds(70))
        XCTAssertEqual(TestURLProtocol.requestCount, 2)
        XCTAssertEqual(navigation.state, .idle)
    }

    func testReplacingScanDuringSuspendedSafetyStopDoesNotFailReplacementNavigation() async {
        TestURLProtocol.reset()
        TestURLProtocol.delay = 0.20
        let headingStarted = expectation(description: "scan relative-heading preflight started")
        let ar = ARSessionManager { event, _ in
            if event == "relative_heading_measurement_started" {
                headingStarted.fulfill()
            }
        }
        ingestTrackedPose(into: ar)
        let navigation = makeNavigation(
            ar: ar,
            safetyFeedback: { Self.tippingFeedback }
        )

        let scan = Task { await navigation.rotateForScan(by: .pi / 6) }
        await fulfillment(of: [headingStarted], timeout: 1)
        ingestReliableRelativeHeading(into: ar)
        await waitForRequestCount(2)

        ingestTrackedPose(into: ar, sequence: 2)
        navigation.navigate(to: Vec2(1, 0))
        await waitForNavigationCommandCount(1)
        await scan.value

        XCTAssertEqual(navigation.state, .driving)
        await navigation.stopAndWait()
    }

    func testReplacingScanDuringSuspendedPreflightStopHandsOffActiveState() async {
        TestURLProtocol.reset()
        TestURLProtocol.delay = 0.20
        let replacementHeadingStarted = expectation(description: "replacement scan heading started")
        let replacementInvocationStarted = expectation(description: "replacement scan invocation started")
        let ar = ARSessionManager { event, _ in
            if event == "relative_heading_measurement_started" {
                replacementHeadingStarted.fulfill()
            }
        }
        ingestTrackedPose(into: ar)
        let navigation = makeNavigation(ar: ar)

        let firstScan = Task { await navigation.rotateForScan(by: .pi / 6) }
        await waitForRequestCount(1)
        guard let firstRequest = TestURLProtocol.requestRecords.first else {
            firstScan.cancel()
            await firstScan.value
            XCTFail("Expected scan A's preflight request")
            return
        }
        XCTAssertEqual(TestURLProtocol.requestRecords.count, 1)
        XCTAssertEqual(
            Self.requestOpcode(from: firstRequest.url),
            RoverConfig.Opcode.emergencyStop,
            "scan A's first request must be its preflight transport stop"
        )
        XCTAssertNil(
            firstRequest.statusCode,
            "scan A's preflight stop must still be suspended"
        )

        TestURLProtocol.delay = 0
        let secondScan = Task { @MainActor in
            replacementInvocationStarted.fulfill()
            await navigation.rotateForScan(by: -.pi / 6)
        }
        await fulfillment(of: [replacementInvocationStarted], timeout: 1)
        guard let suspendedFirstRequest = TestURLProtocol.requestRecords.first else {
            secondScan.cancel()
            await secondScan.value
            firstScan.cancel()
            await firstScan.value
            XCTFail("Expected scan A's preflight request while scan B was invoked")
            return
        }
        XCTAssertNil(
            suspendedFirstRequest.statusCode,
            "scan B must claim ownership while scan A's preflight stop is suspended"
        )

        await waitForRequestStatus(at: 0, expected: 200)
        await firstScan.value
        await fulfillment(of: [replacementHeadingStarted], timeout: 1)
        await waitForRequestCount(2)
        guard let secondRequest = TestURLProtocol.requestRecords.dropFirst().first else {
            secondScan.cancel()
            await secondScan.value
            XCTFail("Expected scan B's preflight request")
            return
        }
        XCTAssertEqual(TestURLProtocol.requestRecords.count, 2)
        XCTAssertEqual(
            Self.requestOpcode(from: secondRequest.url),
            RoverConfig.Opcode.emergencyStop,
            "scan B must perform its own preflight transport stop after the handoff"
        )
        XCTAssertEqual(secondRequest.statusCode, 200)

        XCTAssertTrue(
            ar.ingestRelativeHeadingSample(Self.reliableHeadingSample()),
            "scan B must own an active relative-heading measurement after the handoff"
        )
        await waitForNavigationState(.driving, navigation: navigation)
        XCTAssertEqual(ar.relativeHeadingMeasurement().reliability, .reliable)
        XCTAssertEqual(navigation.state, .driving)

        secondScan.cancel()
        await secondScan.value
    }

    func testScanFailureWithoutTrackedPoseAwaitsTransportStop() async {
        TestURLProtocol.reset()
        let configuration = URLSessionConfiguration.ephemeral
        configuration.protocolClasses = [TestURLProtocol.self]
        let navigation = NavigationController(
            ar: ARSessionManager(),
            control: RoverControl(host: "rover.test", session: URLSession(configuration: configuration)),
            trackingRecoveryTimeout: 0
        )

        await navigation.rotateForScan(by: .pi / 6)

        XCTAssertEqual(TestURLProtocol.requestCount, 2)
        XCTAssertEqual(
            navigation.state,
            .failed("AR tracking is not ready — keep the phone still and try again.")
        )
    }

    func testLeftRotationUsesMinimumPhysicalTurnSpeed() {
        let command = RotationCommand.command(forYawError: .pi / 2)

        XCTAssertEqual(command.left, -RoverConfig.minimumRotateWheelSpeed, accuracy: 1e-9)
        XCTAssertEqual(command.right, RoverConfig.minimumRotateWheelSpeed, accuracy: 1e-9)
    }

    func testRightRotationUsesMinimumPhysicalTurnSpeed() {
        let command = RotationCommand.command(forYawError: -.pi / 2)

        XCTAssertEqual(command.left, RoverConfig.minimumRotateWheelSpeed, accuracy: 1e-9)
        XCTAssertEqual(command.right, -RoverConfig.minimumRotateWheelSpeed, accuracy: 1e-9)
    }

    func testBlindLeftScanUsesForwardDepthVisibleArc() {
        let command = RotationCommand.depthVisibleArc(forYawError: .pi / 6)

        XCTAssertGreaterThanOrEqual(command.left, 0)
        XCTAssertGreaterThan(command.right, command.left)
        XCTAssertEqual(command.right, RoverConfig.minimumRotateWheelSpeed, accuracy: 1e-9)
        XCTAssertEqual(DepthSafetyMotionClass.classify(command), .curved)
    }

    func testBlindRightScanUsesForwardDepthVisibleArc() {
        let command = RotationCommand.depthVisibleArc(forYawError: -.pi / 6)

        XCTAssertGreaterThan(command.left, command.right)
        XCTAssertGreaterThanOrEqual(command.right, 0)
        XCTAssertEqual(command.left, RoverConfig.minimumRotateWheelSpeed, accuracy: 1e-9)
        XCTAssertEqual(DepthSafetyMotionClass.classify(command), .curved)
    }

    func testScanCompletionUsesZeroBasedRelativeRotation() {
        XCTAssertFalse(NavigationController.scanTurnReachedRelativeTarget(
            requestedAngle: .pi / 6,
            accumulatedAngle: 18 * .pi / 180,
            tolerance: RoverConfig.scanTurnYawTolerance
        ))
        XCTAssertTrue(NavigationController.scanTurnReachedRelativeTarget(
            requestedAngle: .pi / 6,
            accumulatedAngle: 24 * .pi / 180,
            tolerance: RoverConfig.scanTurnYawTolerance
        ))
        XCTAssertFalse(NavigationController.scanTurnReachedRelativeTarget(
            requestedAngle: .pi / 6,
            accumulatedAngle: -24 * .pi / 180,
            tolerance: RoverConfig.scanTurnYawTolerance
        ))
        XCTAssertTrue(NavigationController.scanTurnReachedRelativeTarget(
            requestedAngle: -.pi / 6,
            accumulatedAngle: -24 * .pi / 180,
            tolerance: RoverConfig.scanTurnYawTolerance
        ))
    }

    func testCorrectDirectionOvershootCompletesScan() {
        XCTAssertTrue(NavigationController.scanTurnReachedRelativeTarget(
            requestedAngle: .pi / 6,
            accumulatedAngle: 71 * .pi / 180,
            tolerance: RoverConfig.scanTurnYawTolerance
        ))
    }

    func testOppositeDirectionRotationIsRejectedWithoutReversal() {
        XCTAssertTrue(NavigationController.scanTurnMovedOppositeDirection(
            requestedAngle: .pi / 6,
            accumulatedAngle: -3 * .pi / 180
        ))
        XCTAssertFalse(NavigationController.scanTurnMovedOppositeDirection(
            requestedAngle: .pi / 6,
            accumulatedAngle: 3 * .pi / 180
        ))
    }

    func testScanPulseAlwaysUsesMinimumDuration() {
        XCTAssertEqual(
            NavigationController.scanPulseDuration(),
            RoverConfig.scanTurnPulseDuration,
            accuracy: 0.000_001
        )
        XCTAssertEqual(RoverConfig.scanTurnPulseDuration, 0.02, accuracy: 0.000_001)
        XCTAssertGreaterThanOrEqual(RoverConfig.scanTurnSettleDuration, 0.75)
    }

    func testFreshNormalARFrameIsReadyAfterRelativeHeadingPulse() {
        XCTAssertTrue(NavigationController.isScanFrameReady(
            baselineFrame: 10,
            currentFrame: 11,
            isTrackingNormal: true,
            hasPose: true
        ))
        XCTAssertFalse(NavigationController.isScanFrameReady(
            baselineFrame: 10,
            currentFrame: 10,
            isTrackingNormal: true,
            hasPose: true
        ))
        XCTAssertFalse(NavigationController.isScanFrameReady(
            baselineFrame: 10,
            currentFrame: 11,
            isTrackingNormal: false,
            hasPose: true
        ))
    }

    func testExpectedCommandedYawIsConsistentWithRelativeHeading() {
        let before = Pose2D(position: Vec2(1, 1), yaw: 0)
        let after = Pose2D(position: Vec2(1.02, 1.01), yaw: 70 * .pi / 180)

        XCTAssertTrue(NavigationController.isSettledScanPoseConsistent(
            from: before,
            to: after,
            accumulatedAngle: 70 * .pi / 180
        ))
    }

    func testTranslationJumpAndHeadingDisagreementAreInconsistent() {
        let start = Pose2D(position: .zero, yaw: 0)
        XCTAssertFalse(NavigationController.isSettledScanPoseConsistent(
            from: start,
            to: Pose2D(position: Vec2(0.11, 0), yaw: 30 * .pi / 180),
            accumulatedAngle: 30 * .pi / 180
        ))
        XCTAssertFalse(NavigationController.isSettledScanPoseConsistent(
            from: start,
            to: Pose2D(position: Vec2(0.01, 0), yaw: 55 * .pi / 180),
            accumulatedAngle: 30 * .pi / 180
        ))
    }

    func testScanPulseLimitStopsBeforeUnboundedRotation() {
        XCTAssertFalse(NavigationController.scanPulseLimitReached(
            RoverConfig.scanTurnMaxPulseCount - 1
        ))
        XCTAssertTrue(NavigationController.scanPulseLimitReached(
            RoverConfig.scanTurnMaxPulseCount
        ))
        XCTAssertGreaterThan(
            RoverConfig.blockedHeadingRecoveryTimeout,
            RoverConfig.maximumScanTurnDuration
        )
    }

    private func makeNavigation(
        ar: ARSessionManager,
        safetyFeedback: @escaping () -> RoverFeedback? = { nil }
    ) -> NavigationController {
        let configuration = URLSessionConfiguration.ephemeral
        configuration.protocolClasses = [TestURLProtocol.self]
        return NavigationController(
            ar: ar,
            control: RoverControl(host: "rover.test", session: URLSession(configuration: configuration)),
            trackingRecoveryTimeout: 0.5,
            scanDepthRecoveryTimeout: 0.5,
            guardLayer: ObstacleGuard(),
            safetyFeedback: safetyFeedback,
            depthSafetyObservation: { command in
                DepthSafetyObservation(
                    state: .clear,
                    clearance: .infinity,
                    supportCount: 1,
                    sampleAge: 0,
                    requiredStoppingDistance: 0,
                    motionClass: DepthSafetyMotionClass.classify(command),
                    speedLimit: nil
                )
            }
        )
    }

    private func ingestTrackedPose(into ar: ARSessionManager, sequence: UInt64 = 1) {
        if sequence == 1 {
            ar.resetTracking(generation: 1, runSession: false)
        }
        XCTAssertTrue(ar.ingest(PoseObservation(
            pose: Pose2D(position: .zero, yaw: 0),
            frameSequence: sequence,
            timestamp: ProcessInfo.processInfo.systemUptime,
            trackingQuality: .normal,
            sessionGeneration: 1
        )))
    }

    private func ingestReliableRelativeHeading(into ar: ARSessionManager) {
        XCTAssertTrue(ar.ingestRelativeHeadingSample(Self.reliableHeadingSample()))
    }

    private static func reliableHeadingSample() -> RelativeHeadingSample {
        RelativeHeadingSample(
            timestamp: ProcessInfo.processInfo.systemUptime,
            rotationRate: .zero,
            gravity: SIMD3<Double>(0, -1, 0)
        )
    }

    private func waitForRequestCount(_ expected: Int) async {
        let deadline = ContinuousClock.now + .seconds(1)
        while TestURLProtocol.requestCount < expected, ContinuousClock.now < deadline {
            try? await Task.sleep(for: .milliseconds(2))
        }
        XCTAssertGreaterThanOrEqual(TestURLProtocol.requestCount, expected)
    }

    private func waitForRequestStatus(at index: Int, expected: Int) async {
        let deadline = ContinuousClock.now + .seconds(1)
        while ContinuousClock.now < deadline {
            let requests = TestURLProtocol.requestRecords
            if requests.indices.contains(index), requests[index].statusCode == expected {
                return
            }
            try? await Task.sleep(for: .milliseconds(2))
        }
        let status = TestURLProtocol.requestRecords.indices.contains(index)
            ? TestURLProtocol.requestRecords[index].statusCode
            : nil
        XCTAssertEqual(status, expected)
    }

    private func waitForNavigationState(
        _ expected: NavigationController.State,
        navigation: NavigationController
    ) async {
        let deadline = ContinuousClock.now + .seconds(1)
        while navigation.state != expected, ContinuousClock.now < deadline {
            try? await Task.sleep(for: .milliseconds(2))
        }
        XCTAssertEqual(navigation.state, expected)
    }

    private func waitForNavigationCommandCount(_ expected: Int) async {
        let deadline = ContinuousClock.now + .seconds(2)
        while navigationCommandCount < expected, ContinuousClock.now < deadline {
            try? await Task.sleep(for: .milliseconds(2))
        }
        XCTAssertGreaterThanOrEqual(navigationCommandCount, expected)
    }

    private var navigationCommandCount: Int {
        TestURLProtocol.requestURLs.filter { Self.requestOpcode(from: $0) == RoverConfig.Opcode.speedControl }.count
    }

    private static func requestOpcode(from url: URL) -> Int? {
        guard let value = URLComponents(url: url, resolvingAgainstBaseURL: false)?
            .queryItems?.first(where: { $0.name == "json" })?.value,
              let data = value.data(using: .utf8),
              let payload = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            return nil
        }
        return payload["T"] as? Int
    }

    private static let tippingFeedback = RoverFeedback(
        T: nil,
        L: nil,
        R: nil,
        ax: nil,
        ay: nil,
        az: nil,
        gx: nil,
        gy: nil,
        gz: nil,
        roll: 1,
        pitch: nil,
        yaw: nil,
        v: nil
    )
}
