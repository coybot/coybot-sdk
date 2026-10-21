import XCTest
@testable import RoverNav

final class TargetKalmanFilterTests: XCTestCase {
    /// A person walking at (0.4, 0) m/s, measured every 0.1 s with a small fixed offset.
    private func trained(seconds: Double = 3.0) -> (TargetKalmanFilter, TimeInterval) {
        var filter = TargetKalmanFilter(position: Vec2(0, 0), at: 0)
        var t = 0.0
        while t < seconds {
            t += 0.1
            filter.predict(to: t)
            filter.update(position: Vec2(0.4 * t, 0), sigma: 0.08)
        }
        return (filter, t)
    }

    func testLearnsConstantVelocityFromPositionMeasurements() {
        let (filter, t) = trained()
        let e = filter.estimate(at: t)
        XCTAssertEqual(e.velocity.x, 0.4, accuracy: 0.05)
        XCTAssertEqual(e.velocity.y, 0.0, accuracy: 0.05)
        XCTAssertEqual(e.position.x, 0.4 * t, accuracy: 0.05)
    }

    func testCoastsAlongLearnedVelocityAndUncertaintyGrows() {
        let (filter, t) = trained()
        let now = filter.estimate(at: t)
        let later = filter.estimate(at: t + 2.0)
        XCTAssertEqual(later.position.x - now.position.x, 0.8, accuracy: 0.12)
        XCTAssertGreaterThan(later.positionSigma, now.positionSigma * 2,
                             "a coasting estimate must admit it is getting less certain")
    }

    func testEstimateDoesNotMutateTheFilter() {
        let (filter, t) = trained()
        let before = filter.estimate(at: t)
        _ = filter.estimate(at: t + 5)
        XCTAssertEqual(filter.estimate(at: t), before)
    }

    func testGateRejectsFarMeasurementAndWidensWhileCoasting() {
        let (filter, t) = trained()
        let expected = filter.estimate(at: t).position
        let farOff = Vec2(expected.x, expected.y + 1.5)
        XCTAssertGreaterThan(filter.gate(position: farOff, sigma: 0.08), 9.21)
        XCTAssertLessThan(filter.gate(position: expected, sigma: 0.08), 0.5)
        var coasting = filter
        coasting.predict(to: t + 3.0)
        let beyond = coasting.estimate(at: t + 3.0).position
        let reappears = Vec2(beyond.x, beyond.y + 1.5)
        XCTAssertLessThan(coasting.gate(position: reappears, sigma: 0.08), 9.21,
                          "after 3 s of silence a person 1.5 m off the prediction is still plausible")
    }

    func testBearingUpdateCorrectsDirectionButNotRange() {
        // Rover at the origin looking along +x. Track believes the person is 2 m ahead, 0.4 m left.
        var filter = TargetKalmanFilter(position: Vec2(2.0, 0.4), at: 0, positionSigma: 0.4)
        let rangeBefore = hypot(2.0, 0.4)
        // The camera actually sees them dead ahead (bearing 0).
        filter.update(bearing: 0, from: Vec2(0, 0), sigma: 0.03)
        let e = filter.estimate(at: 0)
        XCTAssertLessThan(abs(e.position.y), 0.15, "bearing should pull the lateral error in")
        XCTAssertEqual(hypot(e.position.x, e.position.y), rangeBefore, accuracy: 0.25,
                       "a bearing alone must not move the range estimate much")
    }

    func testBearingGateRejectsADifferentPersonAndAcceptsTheSame() {
        let filter = TargetKalmanFilter(position: Vec2(2.0, 0.0), at: 0, positionSigma: 0.15)
        XCTAssertLessThan(filter.gate(bearing: 0.02, from: Vec2(0, 0), sigma: 0.03), 6.63)
        XCTAssertGreaterThan(filter.gate(bearing: 0.5, from: Vec2(0, 0), sigma: 0.03), 6.63)
    }

    func testBearingWrapsAroundPi() {
        let filter = TargetKalmanFilter(position: Vec2(-2.0, 0.01), at: 0, positionSigma: 0.1)
        // Predicted bearing is ~ +pi; a measurement just across the seam at ~ -pi is close.
        XCTAssertLessThan(filter.gate(bearing: -Double.pi + 0.01, from: Vec2(0, 0), sigma: 0.03), 6.63)
    }

    func testBearingInnovationIsSignedAndWrapped() throws {
        let filter = TargetKalmanFilter(position: Vec2(-2.0, 0.0), at: 0)
        XCTAssertEqual(try XCTUnwrap(filter.predictedBearing(from: Vec2(0, 0))), .pi, accuracy: 1e-9)
        XCTAssertEqual(try XCTUnwrap(filter.bearingInnovation(-Double.pi + 0.1, from: Vec2(0, 0))), 0.1, accuracy: 1e-9)
        XCTAssertEqual(try XCTUnwrap(filter.bearingInnovation(Double.pi - 0.1, from: Vec2(0, 0))), -0.1, accuracy: 1e-9)
        XCTAssertNil(filter.predictedBearing(from: Vec2(-2.0, 0.0)), "undefined when the camera is on the person")
    }

    func testPredictIgnoresPastAndNonFiniteTimes() {
        var filter = TargetKalmanFilter(position: Vec2(1, 1), at: 5)
        filter.predict(to: 4)
        filter.predict(to: .nan)
        XCTAssertEqual(filter.time, 5)
        XCTAssertEqual(filter.currentEstimate.position, Vec2(1, 1))
    }
}
