import XCTest
@testable import PhroverKit

final class DepthSafetyModelsTests: XCTestCase {
    func testCameraMountCalibrationAcceptsMeasuredPhoneHeights() {
        for height in [0.40, 0.55, 0.70] {
            let calibration = CameraMountCalibration(
                cameraHeight: height,
                forwardOffset: 0.08,
                lateralOffset: 0,
                headingAlignment: 0
            )

            XCTAssertEqual(calibration.validation, .valid)
        }
    }

    func testCameraMountCalibrationRejectsUnsafeValuesWithStableReasons() {
        XCTAssertEqual(
            CameraMountCalibration(cameraHeight: .nan).validation,
            .invalid(.nonFiniteValue)
        )
        XCTAssertEqual(
            CameraMountCalibration(cameraHeight: 0.05).validation,
            .invalid(.cameraHeightOutOfRange)
        )
        XCTAssertEqual(
            CameraMountCalibration(cameraHeight: 0.55, forwardOffset: 1.1).validation,
            .invalid(.mountOffsetOutOfRange)
        )
    }

    func testCollisionGeometryRejectsNonPositiveAndInvertedDimensions() {
        XCTAssertEqual(
            RoverCollisionGeometry(length: 0, width: 0.25, minimumCollisionHeight: 0.03, maximumCollisionHeight: 0.55).validation,
            .invalid(.nonPositiveChassisDimension)
        )
        XCTAssertEqual(
            RoverCollisionGeometry(length: 0.30, width: 0.25, minimumCollisionHeight: 0.60, maximumCollisionHeight: 0.55).validation,
            .invalid(.invalidCollisionHeightBand)
        )
    }

    func testUnavailableObservationPreservesDiagnosticReason() {
        let observation = DepthSafetyObservation.unavailable(
            .staleRawDepth,
            sampleAge: 0.31,
            motionClass: .forward
        )

        XCTAssertEqual(observation.state, .unavailable(.staleRawDepth))
        XCTAssertEqual(observation.sampleAge, 0.31)
        XCTAssertEqual(observation.motionClass, .forward)
        XCTAssertEqual(observation.state.telemetryReason, "stale_raw_depth")
    }
}
