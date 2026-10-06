import RoverNav
import XCTest
@testable import PhroverKit

@MainActor
final class NavigationSilentSearchMotionTests: XCTestCase {
    func testConvertsMissionTargetAndAppliesSectorPolicy() async throws {
        let frame = try XCTUnwrap(SharedMissionFrame(
            localOrigin: Vec2(10, 20), localNorthHeading: .pi / 2, sessionGeneration: 4
        ))
        var requestedGoal: Vec2?
        var policyResult: PathAdmissibilityResult?
        let motion = makeMotion(frame: frame) { goal, policy in
            requestedGoal = goal
            policyResult = policy.evaluate(path: [goal])
            return .arrived
        }

        let result = await motion.navigate(
            to: MissionPoint(x: -1, y: 2)!, policy: .sectorConstrained(.west)
        )

        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(requestedGoal, Vec2(9, 22))
        XCTAssertEqual(policyResult, .admissible)
    }

    func testUnrestrictedConvergenceDoesNotApplySectorConstraint() async {
        var policyResult: PathAdmissibilityResult?
        let motion = makeMotion { goal, policy in
            policyResult = policy.evaluate(path: [goal])
            return .arrived
        }

        _ = await motion.navigate(
            to: MissionPoint(x: 1, y: 0)!, policy: .unrestrictedConvergence
        )

        XCTAssertEqual(policyResult, .admissible)
    }

    func testMapsTypedNavigationFailures() async {
        let expected: [(NavigationResult, SilentSearchMotionResult)] = [
            (.failed(.noPath), .failed(.noPath)),
            (.failed(.pathRejected(.outsideSector(pointIndex: 2))),
             .failed(.pathRejected(.outsideSector(pointIndex: 2)))),
            (.failed(.obstacle), .failed(.obstacle)),
            (.failed(.commandFailed), .failed(.commandLink)),
            (.failed(.commsLost), .failed(.commandLink)),
            (.failed(.trackingLost), .failed(.tracking)),
            (.cancelled, .cancelled),
        ]

        for (navigationResult, motionResult) in expected {
            let motion = makeMotion { _, _ in navigationResult }
            let result = await motion.navigate(
                to: MissionPoint(x: 0, y: 0)!, policy: .unrestrictedConvergence
            )
            XCTAssertEqual(result, motionResult)
        }
    }

    func testFinalRotationUsesLocalHeadingAndVerifiesBothTolerances() async {
        let frame = SharedMissionFrame(
            localOrigin: .zero, localNorthHeading: .pi / 2, sessionGeneration: 1
        )!
        var pose = Pose2D(position: Vec2(-0.1, 0), yaw: .pi / 2)
        var rotation: Double?
        let motion = NavigationSilentSearchMotion(
            frame: frame,
            currentPose: { pose }, currentPath: { [] },
            navigateAndWait: { _, _ in .arrived },
            rotateAndWait: { angle in
                rotation = angle
                pose = Pose2D(position: Vec2(-0.1, 0), yaw: .pi)
                return .arrived
            },
            cancelAndWait: {}
        )
        _ = await motion.navigate(
            to: MissionPoint(x: 0, y: 0)!, policy: .unrestrictedConvergence
        )

        let result = await motion.rotate(to: .pi / 2, tolerance: 0.1)

        XCTAssertEqual(rotation ?? 0, .pi / 2, accuracy: 0.000_001)
        XCTAssertEqual(result, .arrived)
    }

    func testFinalPositionAndHeadingOutsideToleranceAreTypedFailures() async {
        var pose = Pose2D(position: Vec2(0.21, 0), yaw: 0)
        let motion = NavigationSilentSearchMotion(
            frame: identityFrame, currentPose: { pose }, currentPath: { [] },
            navigateAndWait: { _, _ in .arrived }, rotateAndWait: { _ in .arrived },
            cancelAndWait: {}
        )
        _ = await motion.navigate(
            to: MissionPoint(x: 0, y: 0)!, policy: .unrestrictedConvergence
        )
        let positionResult = await motion.rotate(to: 0, tolerance: 0.1)
        XCTAssertEqual(positionResult, .failed(.positionToleranceExceeded))

        pose = Pose2D(position: .zero, yaw: 0.11)
        let headingResult = await motion.rotate(to: 0, tolerance: 0.1)
        XCTAssertEqual(headingResult, .failed(.headingToleranceExceeded))
    }

    func testStopAwaitsNavigationCancellation() async {
        var cancellationFinished = false
        let motion = NavigationSilentSearchMotion(
            frame: identityFrame, currentPose: { nil }, currentPath: { [] },
            navigateAndWait: { _, _ in .arrived }, rotateAndWait: { _ in .arrived },
            cancelAndWait: {
                await Task.yield()
                cancellationFinished = true
            }
        )

        await motion.stop()

        XCTAssertTrue(cancellationFinished)
    }

    private var identityFrame: SharedMissionFrame {
        SharedMissionFrame(localOrigin: .zero, localNorthHeading: 0, sessionGeneration: 1)!
    }

    private func makeMotion(
        frame: SharedMissionFrame? = nil,
        navigate: @escaping (Vec2, any PathAdmissibilityPolicy) async -> NavigationResult
    ) -> NavigationSilentSearchMotion {
        NavigationSilentSearchMotion(
            frame: frame ?? identityFrame,
            currentPose: { Pose2D(position: .zero, yaw: 0) }, currentPath: { [] },
            navigateAndWait: navigate, rotateAndWait: { _ in .arrived }, cancelAndWait: {}
        )
    }
}
