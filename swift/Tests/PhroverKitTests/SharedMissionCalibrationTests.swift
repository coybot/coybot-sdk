import XCTest
import RoverNav
@testable import PhroverKit

final class SharedMissionCalibrationTests: XCTestCase {
    func testThreeSyntheticFramesDeriveTranslatedRotatedSharedFrame() throws {
        var calibrator = try XCTUnwrap(SharedMissionCalibrator(
            configuration: SharedMissionCalibrationConfiguration(markerID: "SILENT_SEARCH_01")!,
            sessionGeneration: 12
        ))

        XCTAssertEqual(calibrator.observe(observation(frameID: 1, timestamp: 10)), .collecting(frameCount: 1))
        XCTAssertEqual(calibrator.observe(observation(frameID: 2, timestamp: 10.5)), .collecting(frameCount: 2))
        let result = calibrator.observe(observation(frameID: 3, timestamp: 11))

        guard case let .accepted(frame) = result else {
            return XCTFail("expected accepted calibration, got \(result)")
        }
        XCTAssertEqual(frame.localOrigin.x, 2, accuracy: 1e-12)
        XCTAssertEqual(frame.localOrigin.y, -3, accuracy: 1e-12)
        XCTAssertEqual(frame.localNorthHeading, .pi / 3, accuracy: 1e-12)
        XCTAssertEqual(frame.sessionGeneration, 12)
    }

    func testPayloadGrammarAcceptsOnlyCanonicalMarkerIDs() {
        XCTAssertEqual(
            SharedMissionCalibrator.payload(forMarkerID: "SILENT_SEARCH_01"),
            "PHROVER-CAL|1|SILENT_SEARCH_01"
        )
        XCTAssertEqual(
            SharedMissionCalibrator.markerID(fromPayload: "PHROVER-CAL|1|SILENT_SEARCH_01"),
            "SILENT_SEARCH_01"
        )
        XCTAssertNil(SharedMissionCalibrator.markerID(fromPayload: "PHROVER-CAL|2|SILENT_SEARCH_01"))
        XCTAssertNil(SharedMissionCalibrator.markerID(fromPayload: "PHROVER-CAL|1|lowercase"))
        XCTAssertNotNil(SharedMissionCalibrator.payload(forMarkerID: "ABCDEFGHIJKLMNOPQRSTUVWX"))
        XCTAssertNil(SharedMissionCalibrator.payload(forMarkerID: "ABCDEFGHIJKLMNOPQRSTUVWXY"))
    }

    func testMarkerGenerationAndDuplicateDiagnosticsHaveDeterministicPrecedence() throws {
        var calibrator = makeCalibrator()

        XCTAssertEqual(
            calibrator.observe(observation(frameID: 1, timestamp: 0, markerID: "bad", generation: 99)),
            .rejected(.invalidMarkerID)
        )
        XCTAssertEqual(
            calibrator.observe(observation(frameID: 1, timestamp: 0, markerID: "OTHER", generation: 99)),
            .rejected(.unexpectedMarkerID)
        )
        XCTAssertEqual(
            calibrator.observe(observation(frameID: 1, timestamp: 0, generation: 99)),
            .rejected(.generationMismatch)
        )
        XCTAssertEqual(calibrator.observe(observation(frameID: 1, timestamp: 0)), .collecting(frameCount: 1))
        XCTAssertEqual(
            calibrator.observe(observation(frameID: 1, timestamp: 0.5)),
            .rejected(.duplicateFrame)
        )
        XCTAssertEqual(calibrator.observe(observation(frameID: 2, timestamp: 1)), .collecting(frameCount: 2))
    }

    func testRejectsNonFiniteAndDegenerateCorners() {
        var calibrator = makeCalibrator()
        let valid = observation(frameID: 1, timestamp: 0)
        let nonFinite = SharedMissionCalibrationObservation(
            markerID: valid.markerID,
            sessionGeneration: valid.sessionGeneration,
            frameID: valid.frameID,
            monotonicTimestamp: valid.monotonicTimestamp,
            corners: OrientedMarkerCorners(
                topLeft: Vec2(.nan, 0),
                topRight: valid.corners.topRight,
                bottomLeft: valid.corners.bottomLeft,
                bottomRight: valid.corners.bottomRight
            )
        )
        let same = Vec2(1, 1)
        let degenerate = SharedMissionCalibrationObservation(
            markerID: valid.markerID,
            sessionGeneration: valid.sessionGeneration,
            frameID: 2,
            monotonicTimestamp: 0,
            corners: OrientedMarkerCorners(
                topLeft: same,
                topRight: same,
                bottomLeft: same,
                bottomRight: same
            )
        )

        XCTAssertEqual(calibrator.observe(nonFinite), .rejected(.nonFiniteObservation))
        XCTAssertEqual(calibrator.observe(degenerate), .rejected(.degenerateCorners))
    }

    func testTwoSecondWindowBoundaryIsInclusive() {
        var accepted = makeCalibrator()
        _ = accepted.observe(observation(frameID: 1, timestamp: 10))
        _ = accepted.observe(observation(frameID: 2, timestamp: 11))
        XCTAssertAccepted(accepted.observe(observation(frameID: 3, timestamp: 12)))

        var rejected = makeCalibrator()
        _ = rejected.observe(observation(frameID: 1, timestamp: 10))
        _ = rejected.observe(observation(frameID: 2, timestamp: 11))
        XCTAssertEqual(
            rejected.observe(observation(frameID: 3, timestamp: 12.000_001)),
            .rejected(.timeWindowExceeded)
        )
    }

    func testOriginDeviationBoundaryAndDiagnosticPrecedence() {
        var accepted = makeCalibrator()
        _ = accepted.observe(observation(frameID: 1, timestamp: 0, origin: Vec2(1.9, -3)))
        _ = accepted.observe(observation(frameID: 2, timestamp: 1, origin: Vec2(2, -3)))
        XCTAssertAccepted(accepted.observe(observation(frameID: 3, timestamp: 2, origin: Vec2(2.1, -3))))

        var rejected = makeCalibrator()
        _ = rejected.observe(observation(frameID: 1, timestamp: 0, origin: Vec2(1.899, -3)))
        _ = rejected.observe(observation(frameID: 2, timestamp: 1, origin: Vec2(2, -3)))
        XCTAssertEqual(
            rejected.observe(observation(frameID: 3, timestamp: 2, origin: Vec2(2.1, -3), heading: 1.5, width: 0.30)),
            .rejected(.originDeviationExceeded)
        )
    }

    func testCircularHeadingMeanAcceptsWraparoundAndRejectsBeyondFiveDegrees() {
        var accepted = makeCalibrator()
        _ = accepted.observe(observation(frameID: 1, timestamp: 0, heading: 179 * .pi / 180))
        _ = accepted.observe(observation(frameID: 2, timestamp: 1, heading: -179 * .pi / 180))
        let result = accepted.observe(observation(frameID: 3, timestamp: 2, heading: .pi))
        guard case let .accepted(frame) = result else { return XCTFail("expected wraparound acceptance") }
        XCTAssertEqual(frame.localNorthHeading, .pi, accuracy: 1e-12)

        var rejected = makeCalibrator()
        _ = rejected.observe(observation(frameID: 1, timestamp: 0, heading: 0))
        _ = rejected.observe(observation(frameID: 2, timestamp: 1, heading: 0))
        XCTAssertEqual(
            rejected.observe(observation(frameID: 3, timestamp: 2, heading: 8 * .pi / 180)),
            .rejected(.headingDeviationExceeded)
        )

        var boundary = makeCalibrator()
        _ = boundary.observe(observation(frameID: 1, timestamp: 0, heading: -5 * .pi / 180))
        _ = boundary.observe(observation(frameID: 2, timestamp: 1, heading: 0))
        XCTAssertAccepted(boundary.observe(observation(frameID: 3, timestamp: 2, heading: 5 * .pi / 180)))
    }

    func testHeadingDiagnosticPrecedesWidthDiagnostic() {
        var calibrator = makeCalibrator()
        _ = calibrator.observe(observation(frameID: 1, timestamp: 0, heading: 0, width: 0.30))
        _ = calibrator.observe(observation(frameID: 2, timestamp: 1, heading: 0, width: 0.30))
        XCTAssertEqual(
            calibrator.observe(observation(frameID: 3, timestamp: 2, heading: 8 * .pi / 180, width: 0.30)),
            .rejected(.headingDeviationExceeded)
        )
    }

    func testWidthFractionBoundaryIsInclusive() {
        var accepted = makeCalibrator()
        _ = accepted.observe(observation(frameID: 1, timestamp: 0, width: 0.17))
        _ = accepted.observe(observation(frameID: 2, timestamp: 1, width: 0.20))
        XCTAssertAccepted(accepted.observe(observation(frameID: 3, timestamp: 2, width: 0.23)))

        var rejected = makeCalibrator()
        _ = rejected.observe(observation(frameID: 1, timestamp: 0, width: 0.169))
        _ = rejected.observe(observation(frameID: 2, timestamp: 1, width: 0.20))
        XCTAssertEqual(
            rejected.observe(observation(frameID: 3, timestamp: 2, width: 0.23)),
            .rejected(.widthDeviationExceeded)
        )
    }

    private func makeCalibrator() -> SharedMissionCalibrator {
        SharedMissionCalibrator(
            configuration: SharedMissionCalibrationConfiguration(markerID: "SILENT_SEARCH_01")!,
            sessionGeneration: 12
        )
    }

    private func XCTAssertAccepted(
        _ result: SharedMissionCalibrator.Result,
        file: StaticString = #filePath,
        line: UInt = #line
    ) {
        guard case .accepted = result else {
            return XCTFail("expected accepted calibration, got \(result)", file: file, line: line)
        }
    }

    private func observation(
        frameID: UInt64,
        timestamp: TimeInterval,
        markerID: String = "SILENT_SEARCH_01",
        generation: UInt64 = 12,
        origin: Vec2 = Vec2(2, -3),
        heading: Double = .pi / 3,
        width: Double = 0.20
    ) -> SharedMissionCalibrationObservation {
        let north = Vec2(cos(heading), sin(heading))
        let east = Vec2(north.y, -north.x)
        let half = width / 2
        return SharedMissionCalibrationObservation(
            markerID: markerID,
            sessionGeneration: generation,
            frameID: frameID,
            monotonicTimestamp: timestamp,
            corners: OrientedMarkerCorners(
                topLeft: origin + north * half - east * half,
                topRight: origin + north * half + east * half,
                bottomLeft: origin - north * half - east * half,
                bottomRight: origin - north * half + east * half
            )
        )
    }
}
