import XCTest
import RoverNav
@testable import PhroverKit

final class ObstacleGuardScalingTests: XCTestCase {
    func testCautionDoesNotAdmitInvalidValuesOrIgnoreAStricterLimit() {
        let guardLayer = ObstacleGuard()
        func observation(_ limit: Double?) -> DepthSafetyObservation {
            .init(state: .caution, clearance: 0.55, supportCount: 34,
                sampleAge: 0.125, requiredStoppingDistance: 0.3, motionClass: .curved, speedLimit: limit)
        }
        for limit: Double? in [nil, 0, -0.1, .nan, .infinity] {
            guard case .stopDepth = guardLayer.evaluate(command: .init(left: 0.2, right: 0.15), depthSafety: observation(limit)) else {
                return XCTFail("Invalid caution limits must stop")
            }
        }
        for invalid in [Double.nan, .infinity, -.infinity] {
            guard case .stopDepth = guardLayer.evaluate(command: .init(left: invalid, right: 0.1), depthSafety: observation(0.12)) else {
                return XCTFail("Invalid wheel requests must not become valid through clamping")
            }
        }
        let request = WheelCommand(left: 0.12, right: 0.096)
        let stricter = 0.12.nextDown
        guard case .allow(let reduced, _) = guardLayer.evaluate(command: request, depthSafety: observation(stricter)) else { return XCTFail() }
        XCTAssertNotEqual(reduced, request)
        XCTAssertLessThanOrEqual(max(abs(reduced.left), abs(reduced.right)), stricter)
    }

    func testCautionScalingIsBoundedAndExactlyIdempotent() {
        let guardLayer = ObstacleGuard()
        let observation = DepthSafetyObservation(state: .caution, clearance: 0.55, supportCount: 34,
            sampleAge: 0.125, requiredStoppingDistance: 0.3, motionClass: .curved, speedLimit: 0.12)
        for peak in [0.1337, 0.1366, 0.1397, 0.142, 0.35] {
            for sign in [-1.0, 1.0] {
                let request = WheelCommand(left: sign * peak, right: peak * 0.8)
                guard case .allow(let first, _) = guardLayer.evaluate(command: request, depthSafety: observation),
                      case .allow(let repeated, _) = guardLayer.evaluate(command: first, depthSafety: observation) else {
                    return XCTFail("Caution should allow the bounded command")
                }
                XCTAssertLessThanOrEqual(max(abs(first.left), abs(first.right)), 0.12)
                XCTAssertEqual(repeated, first, "Transport must approve precisely the already limited command")
                XCTAssertEqual(first.right / abs(first.left), 0.8, accuracy: 1e-12)
            }
        }
    }
}
