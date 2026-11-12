import XCTest
import RoverNav
@testable import PhroverKit

final class FollowPursuitAccelerationTests: XCTestCase {
    func testNormalApproachTapersToCrawlBeforeArrivalAndPreservesCurvature() {
        let desired = WheelCommand(left: 0.35, right: 0.175)
        let params = PursuitController.Params()
        let speeds = [1.0, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2].map { distance -> Double in
            let command = FollowPursuitAcceleration.approaching(desired, distance: distance, params: params)
            XCTAssertEqual(command.right, command.left / 2, accuracy: 1e-12)
            return command.left
        }
        XCTAssertEqual(speeds[0], 0.35, accuracy: 1e-12)
        XCTAssertEqual(speeds[2], 0.35 * 2 / 3, accuracy: 1e-12)
        XCTAssertEqual(speeds[3], 0.35 / 3, accuracy: 1e-12)
        XCTAssertEqual(speeds[4], 0.04, accuracy: 1e-12)
        XCTAssertEqual(speeds[5], 0.04, accuracy: 1e-12)
        XCTAssertEqual(speeds[6], 0)
        for (a, b) in zip(speeds, speeds.dropFirst()) { XCTAssertGreaterThanOrEqual(a, b) }
        let cautious = WheelCommand(left: 0.02, right: 0.01)
        XCTAssertEqual(FollowPursuitAcceleration.approaching(cautious, distance: 0.25, params: params), cautious)
        XCTAssertEqual(FollowPursuitAcceleration.approaching(.stop, distance: 1, params: params), .stop)
    }

    func testRampPreservesCurvatureCapsGapsAndResetsAfterStops() {
        var ramp = FollowPursuitAcceleration()
        let desired = WheelCommand(left: 0.35, right: 0.175)
        let first = ramp.limited(desired, at: 10)
        XCTAssertEqual(first.left, 0.1, accuracy: 1e-12)
        XCTAssertEqual(first.right, 0.05, accuracy: 1e-12)
        XCTAssertEqual(ramp.limited(desired, at: 20).left, 0.1, accuracy: 1e-12,
            "Planning and unsent requests cannot advance acceleration")
        ramp.sent(first, at: 10)
        XCTAssertEqual(ramp.limited(desired, at: 10.1).left, 0.125, accuracy: 1e-12)
        XCTAssertEqual(ramp.limited(desired, at: 20).left, 0.15, accuracy: 1e-12,
            "Long scheduling gaps cannot jump directly to cruise")
        XCTAssertEqual(ramp.limited(desired, at: 9).left, 0.1, accuracy: 1e-12)
        ramp.reset()
        XCTAssertEqual(ramp.limited(desired, at: 30).left, 0.1, accuracy: 1e-12)
    }

    func testSafetyReductionsAndStopsNeverWaitForRamp() {
        var ramp = FollowPursuitAcceleration()
        ramp.sent(.init(left: 0.3, right: 0.3), at: 10)
        XCTAssertEqual(ramp.limited(.init(left: 0.04, right: 0.02), at: 10), .init(left: 0.04, right: 0.02))
        XCTAssertEqual(ramp.limited(.init(left: 0, right: 0), at: 10), .init(left: 0, right: 0))
        let rotation = WheelCommand(left: -0.25, right: 0.25)
        XCTAssertEqual(ramp.limited(rotation, at: 10), rotation)
        ramp.sent(rotation, at: 10)
        XCTAssertEqual(ramp.limited(.init(left: 0.35, right: 0.35), at: 11).left, 0.1, accuracy: 1e-12)
    }
}
