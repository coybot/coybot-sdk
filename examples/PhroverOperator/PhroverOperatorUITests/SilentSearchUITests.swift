import XCTest

@MainActor
final class SilentSearchUITests: XCTestCase {
    override func setUpWithError() throws {
        continueAfterFailure = false
    }

    func testSetupNotReadyGatesCalibration() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "setup-not-ready")

        XCTAssertTrue(app.descendants(matching: .any)["silent_search_tab"].waitForExistence(timeout: 10))
        XCTAssertTrue(app.otherElements["silent_search_map"].waitForExistence(timeout: 10))
        XCTAssertTrue(app.staticTexts["Not ready"].exists)
        XCTAssertFalse(app.buttons["silent_search_start"].isEnabled)
        XCTAssertTrue(app.segmentedControls["silent_search_role_picker"].exists)
        XCTAssertTrue(app.buttons["silent_search_target_picker"].exists)
        XCTAssertTrue(app.steppers["silent_search_duration"].exists)
    }

    func testCalibrationCannotAdvanceWithoutAcceptedSamples() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "calibrating")

        XCTAssertTrue(app.staticTexts["Calibration 1 of 3"].waitForExistence(timeout: 10))
        XCTAssertTrue(app.otherElements["silent_search_calibration_preview"].exists)
        XCTAssertEqual(app.otherElements["silent_search_calibration_qr_stage"].value as? String, "Pending")
        XCTAssertEqual(app.otherElements["silent_search_calibration_grounding_stage"].value as? String, "Waiting")
        XCTAssertEqual(app.otherElements["silent_search_calibration_sample_stage"].value as? String, "Waiting")
        XCTAssertEqual(
            app.staticTexts["silent_search_calibration_guidance"].label,
            "Center the complete marker with its white border visible."
        )
        XCTAssertEqual(app.staticTexts["silent_search_calibration_progress"].label, "1 of 3")
        XCTAssertFalse(app.buttons["silent_search_start"].exists)
        XCTAssertFalse(app.otherElements["silent_search_qr"].exists)
        XCTAssertFalse(app.otherElements["silent_search_scanner"].exists)
    }

    func testCalibrationPreviewAppearsOnlyDuringCalibration() {
        var app = XCUIApplication.launchingSilentSearch(scenario: "calibrating")
        XCTAssertTrue(app.otherElements["silent_search_calibration_preview"].waitForExistence(timeout: 10))
        app.terminate()

        app = XCUIApplication.launchingSilentSearch(scenario: "setup-not-ready")
        XCTAssertTrue(app.descendants(matching: .any)["silent_search_tab"].waitForExistence(timeout: 10))
        XCTAssertFalse(app.otherElements["silent_search_calibration_preview"].exists)
    }

    func testDecodedMarkerShowsPolygonMarkerAndPendingGrounding() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "calibration-qr-detected")

        XCTAssertTrue(app.otherElements["silent_search_calibration_polygon"].waitForExistence(timeout: 10))
        XCTAssertEqual(app.staticTexts["silent_search_calibration_marker"].label, "Marker SILENT_SEARCH_01")
        XCTAssertEqual(app.otherElements["silent_search_calibration_qr_stage"].value as? String, "Succeeded")
        XCTAssertEqual(app.otherElements["silent_search_calibration_grounding_stage"].value as? String, "Pending")
    }

    func testNewerPreviewDoesNotShowOlderMarkerPolygon() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "calibration-context-mismatch")

        XCTAssertTrue(app.otherElements["silent_search_calibration_preview"].waitForExistence(timeout: 10))
        XCTAssertFalse(app.otherElements["silent_search_calibration_polygon"].exists)
    }

    func testLostMarkerClearsLabelButPreservesCompletedStages() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "calibration-qr-lost")

        XCTAssertTrue(app.otherElements["silent_search_calibration_preview"].waitForExistence(timeout: 10))
        XCTAssertFalse(app.staticTexts["silent_search_calibration_marker"].exists)
        XCTAssertEqual(app.otherElements["silent_search_calibration_qr_stage"].value as? String, "Succeeded")
        XCTAssertEqual(app.otherElements["silent_search_calibration_grounding_stage"].value as? String, "Succeeded")
        XCTAssertEqual(app.otherElements["silent_search_calibration_sample_stage"].value as? String, "Succeeded")
    }

    func testGroundingFailureShowsActionableGuidance() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "calibration-grounding-failed")

        XCTAssertTrue(app.otherElements["silent_search_calibration_polygon"].waitForExistence(timeout: 10))
        XCTAssertEqual(
            app.staticTexts["silent_search_calibration_guidance"].label,
            "Move the marker toward center or adjust the camera angle."
        )
    }

    func testAcceptedSampleShowsAllLatchedStagesAndProgress() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "calibration-sample-accepted")

        XCTAssertTrue(app.otherElements["silent_search_calibration_preview"].waitForExistence(timeout: 10))
        XCTAssertEqual(app.otherElements["silent_search_calibration_qr_stage"].value as? String, "Succeeded")
        XCTAssertEqual(app.otherElements["silent_search_calibration_grounding_stage"].value as? String, "Succeeded")
        XCTAssertEqual(app.otherElements["silent_search_calibration_sample_stage"].value as? String, "Succeeded")
        XCTAssertEqual(app.staticTexts["silent_search_calibration_progress"].label, "1 of 3")
    }

    func testOfferShowsFullQRCodeAndAbort() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "display-offer")

        XCTAssertTrue(app.images["silent_search_qr"].waitForExistence(timeout: 10))
        XCTAssertTrue(app.staticTexts["Position the other rover's rear camera over this code."].exists)
        XCTAssertTrue(app.buttons["silent_search_qr_complete"].isEnabled)
        XCTAssertTrue(app.buttons["silent_search_abort"].exists)
    }

    func testScannerTimeoutOffersRetryAndAbort() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "scan-timeout")

        XCTAssertTrue(app.otherElements["silent_search_scanner"].waitForExistence(timeout: 10))
        XCTAssertTrue(app.staticTexts["No valid QR code received within 30 seconds."].exists)
        XCTAssertTrue(app.buttons["silent_search_retry"].isEnabled)
        XCTAssertTrue(app.buttons["silent_search_abort"].isEnabled)
    }

    func testPresentationTimeoutOffersRetryAndAbort() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "display-timeout")

        XCTAssertTrue(app.images["silent_search_qr"].waitForExistence(timeout: 10))
        XCTAssertTrue(app.buttons["silent_search_retry"].isEnabled)
        XCTAssertTrue(app.buttons["silent_search_abort"].isEnabled)
    }

    func testStopPersistsInEveryMovingPhase() {
        for scenario in ["searching", "returning", "rendezvous-rotating", "converging"] {
            let app = XCUIApplication.launchingSilentSearch(scenario: scenario)
            XCTAssertTrue(
                app.buttons["silent_search_stop"].waitForExistence(timeout: 10),
                "Stop missing in \(scenario)"
            )
            app.terminate()
        }
    }

    func testIntentionalWaitIsExplicitlyStopped() {
        let app = XCUIApplication.launchingSilentSearch(scenario: "intentional-wait")

        XCTAssertTrue(app.staticTexts["Stopped intentionally while waiting for the other rover."].waitForExistence(timeout: 10))
        XCTAssertFalse(app.buttons["silent_search_stop"].exists)
    }

    func testExactTerminalStates() {
        var app = XCUIApplication.launchingSilentSearch(scenario: "not-found")
        XCTAssertTrue(app.staticTexts["silent_search_terminal_not_found"].waitForExistence(timeout: 10))
        XCTAssertEqual(app.staticTexts["silent_search_terminal_not_found"].label, "NOT FOUND")
        app.terminate()

        app = XCUIApplication.launchingSilentSearch(scenario: "found")
        XCTAssertTrue(app.staticTexts["silent_search_terminal_found"].waitForExistence(timeout: 10))
        XCTAssertEqual(app.staticTexts["silent_search_terminal_found"].label, "FOUND")
        app.terminate()

        app = XCUIApplication.launchingSilentSearch(scenario: "navigation-failure")
        XCTAssertTrue(app.staticTexts["silent_search_terminal_failure"].waitForExistence(timeout: 10))
        XCTAssertEqual(app.staticTexts["silent_search_failure_reason"].label, "Navigation failed: no path.")
    }
}
