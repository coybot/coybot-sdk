import XCTest
@testable import PhroverKit

final class DepthSafetyModelsTests: XCTestCase {
    func testHeightSectionsRequireCompleteBoundedUnambiguousAssembly() {
        let chassis = RoverCollisionGeometry.Section(name: "chassis", length: 0.30, width: 0.25,
            minimumHeight: 0.04, maximumHeight: 0.14)
        let phone = RoverCollisionGeometry.Section(name: "phone", length: 0.08, width: 0.10,
            minimumHeight: 0.14, maximumHeight: 0.34, forwardOffset: -0.085)
        func model(_ sections: [RoverCollisionGeometry.Section]) -> RoverCollisionGeometry {
            .init(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04, maximumCollisionHeight: 0.34, sections: sections)
        }
        XCTAssertEqual(model([chassis, phone]).validation, .valid)
        XCTAssertEqual(model([]).collisionSections.count, 1)
        for sections in [[chassis], [phone, chassis], [chassis, chassis],
                         [chassis, .init(name: "gap", length: 0.08, width: 0.10, minimumHeight: 0.15, maximumHeight: 0.34)],
                         [chassis, .init(name: "overlap", length: 0.08, width: 0.10, minimumHeight: 0.13, maximumHeight: 0.34)],
                         [chassis, .init(name: "wide", length: 0.08, width: 0.30, minimumHeight: 0.14, maximumHeight: 0.34)],
                         [chassis, .init(name: "offset", length: 0.08, width: 0.10, minimumHeight: 0.14, maximumHeight: 0.34, forwardOffset: 0.15)],
                         [chassis, .init(name: "invalid", length: .nan, width: 0.10, minimumHeight: 0.14, maximumHeight: 0.34)]] {
            XCTAssertEqual(model(sections).validation, .invalid(.invalidCollisionSections))
        }
    }

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
