import XCTest

/// UI tests for Phrover Operator. See eco/e2e/README.md (in the sibling `eco` repo) for
/// the full picture across all vehicle types.
///
/// The manual-teleop Drive tab is the ARKit-independent path: DriveView's D-pad calls
/// RoverControl.send() straight from the drag gesture, without waiting on ar.pose, so
/// it works in the iOS Simulator (which has no real camera/motion for ARKit world
/// tracking) — unlike NavigateView's autonomy loop, which needs a real device.
@MainActor
final class PhroverOperatorUITests: XCTestCase {

    override func setUpWithError() throws {
        continueAfterFailure = false
    }

    func testLaunch_showsAuthOrTabs() throws {
        let app = XCUIApplication()
        app.launch()

        let authHero = app.staticTexts["Phrover Operator"]
        let driveTab = app.buttons["Drive"]

        XCTAssertTrue(
            authHero.waitForExistence(timeout: 15) || driveTab.waitForExistence(timeout: 15),
            "Expected sign-in hero (Phrover Operator) or the Drive tab."
        )
    }

    func testTalk_failedCommandKeepsPushToTalkHittable() throws {
        let app = XCUIApplication()
        app.launchArguments.append("-ui-test-failed-command")
        app.launch()

        let talkTab = app.buttons["Talk"]
        XCTAssertTrue(talkTab.waitForExistence(timeout: 15))
        talkTab.tap()

        let failedCard = app.otherElements["last-command-card"]
        XCTAssertTrue(failedCard.waitForExistence(timeout: 10))

        let recognizedCommand = app.staticTexts["recognized-command-text"]
        XCTAssertTrue(recognizedCommand.waitForExistence(timeout: 10))
        XCTAssertEqual(recognizedCommand.label, "Go to our room")

        let mic = app.descendants(matching: .any)
            .matching(identifier: "push-to-talk-control").element
        XCTAssertTrue(mic.waitForExistence(timeout: 10))
        XCTAssertTrue(mic.isHittable)
        XCTAssertLessThanOrEqual(mic.frame.maxY, talkTab.frame.minY)
        screenshot(app, name: "talk_failed_command_retry")
    }

    func testTalkShowsLiveDetectorDiagnostics() throws {
        let app = XCUIApplication()
        app.launchArguments.append("-ui-test-perception-diagnostics")
        app.launch()

        let talkTab = app.buttons["Talk"]
        XCTAssertTrue(talkTab.waitForExistence(timeout: 15))
        talkTab.tap()

        let detector = app.staticTexts["detector-status-text"]
        let visible = app.staticTexts["visible-objects-text"]
        XCTAssertTrue(detector.waitForExistence(timeout: 10))
        XCTAssertEqual(detector.label, "Detector: loaded")
        XCTAssertTrue(visible.waitForExistence(timeout: 10))
        XCTAssertEqual(visible.label, "Visible: refrigerator 99%")
    }

    func testTalkShowsModelNotReadyDiagnostic() throws {
        let app = XCUIApplication()
        app.launchArguments.append("-ui-test-brain-model-not-ready")
        app.launch()

        let talkTab = app.buttons["Talk"]
        XCTAssertTrue(talkTab.waitForExistence(timeout: 15))
        talkTab.tap()

        let diagnostic = app.staticTexts["brain-availability-message"]
        XCTAssertTrue(diagnostic.waitForExistence(timeout: 10))
        XCTAssertTrue(diagnostic.label.contains("Apple Intelligence is still preparing"))

        let mic = app.descendants(matching: .any)
            .matching(identifier: "push-to-talk-control").element
        XCTAssertTrue(mic.waitForExistence(timeout: 10))
        XCTAssertTrue(mic.isHittable)
        XCTAssertLessThanOrEqual(mic.frame.maxY, talkTab.frame.minY)
    }

    func testTalkHidesBrainDiagnosticWhenAvailable() throws {
        let app = XCUIApplication()
        app.launchArguments.append("-ui-test-brain-available")
        app.launch()

        let talkTab = app.buttons["Talk"]
        XCTAssertTrue(talkTab.waitForExistence(timeout: 15))
        talkTab.tap()

        let mic = app.descendants(matching: .any)
            .matching(identifier: "push-to-talk-control").element
        XCTAssertTrue(mic.waitForExistence(timeout: 10))
        XCTAssertTrue(mic.isHittable)
        XCTAssertLessThanOrEqual(mic.frame.maxY, talkTab.frame.minY)

        let diagnostic = app.staticTexts["brain-availability-message"]
        let diagnosticAppears = XCTNSPredicateExpectation(
            predicate: NSPredicate(format: "exists == true"),
            object: diagnostic
        )
        diagnosticAppears.isInverted = true
        XCTAssertEqual(XCTWaiter.wait(for: [diagnosticAppears], timeout: 0.5), .completed)
    }

    /// Gated integration path: drive forward on the D-pad and confirm the command
    /// actually reached the (mocked) WAVE ROVER base — proves the phone->ESP32 control
    /// loop end-to-end with no chassis needed.
    ///
    /// Env: RUN_HEZARFEN_E2E=1, E2E_EMAIL, E2E_PASSWORD, E2E_ROVER_HOST (host:port of a
    /// running eco/e2e/harness/mock_esp32.py — started by eco/e2e/run_phone.sh). Sign-in
    /// only applies if a PhroverCloud.plist is bundled; without one the app skips
    /// straight to the Drive tab.
    func testDrive_forwardCommand_reachesMockChassis() async throws {
        guard ProcessInfo.processInfo.environment["RUN_HEZARFEN_E2E"] == "1" else {
            throw XCTSkip("Set RUN_HEZARFEN_E2E=1 to enable integration tests.")
        }
        guard let mockHost = ProcessInfo.processInfo.environment["E2E_ROVER_HOST"],
              !mockHost.isEmpty else {
            throw XCTSkip("Set E2E_ROVER_HOST to a running mock_esp32.py (host:port).")
        }
        let env = ProcessInfo.processInfo.environment

        let app = XCUIApplication()
        app.launch()

        signInIfNeeded(app: app, email: env["E2E_EMAIL"], password: env["E2E_PASSWORD"])

        let driveTab = app.buttons["Drive"]
        XCTAssertTrue(driveTab.waitForExistence(timeout: 25), "Did not reach the Drive tab.")
        driveTab.tap()

        let forward = app.descendants(matching: .any).matching(identifier: "e2e_drive_forward").element
        XCTAssertTrue(forward.waitForExistence(timeout: 15), "Forward D-pad button not found.")
        screenshot(app, name: "00_drive_tab")

        forward.press(forDuration: 1.0)
        screenshot(app, name: "01_drive_forward_pressed")

        let received = try await pollReceivedDriveCommands(mockHost: mockHost, timeout: 15)
        XCTAssertTrue(received, "mock_esp32 never received a drive (T=1) command.")
    }

    // MARK: - Helpers

    private func signInIfNeeded(app: XCUIApplication, email: String?, password: String?) {
        let driveTab = app.buttons["Drive"]
        if driveTab.waitForExistence(timeout: 8) { return }  // already authed or no cloud configured

        guard let email, let password, !email.isEmpty, !password.isEmpty else {
            XCTFail("On Auth screen but E2E_EMAIL/E2E_PASSWORD not provided.")
            return
        }
        screenshot(app, name: "00_sign_in")
        let emailField = app.textFields["Email"]
        XCTAssertTrue(emailField.waitForExistence(timeout: 15), "Email field not found.")
        emailField.tap(); emailField.typeText(email)

        let pwField = app.secureTextFields["Password"]
        XCTAssertTrue(pwField.waitForExistence(timeout: 5))
        pwField.tap(); pwField.typeText(password)

        app.buttons["Sign In"].tap()
    }

    private func pollReceivedDriveCommands(mockHost: String, timeout: TimeInterval) async throws -> Bool {
        guard let url = URL(string: "http://\(mockHost)/__received") else { return false }
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if let (data, _) = try? await URLSession.shared.data(from: url),
               let commands = try? JSONSerialization.jsonObject(with: data) as? [[String: Any]] {
                if commands.contains(where: { ($0["T"] as? Int) == 1 }) {
                    return true
                }
            }
            try? await Task.sleep(nanoseconds: 500_000_000)
        }
        return false
    }

    private func screenshot(_ app: XCUIApplication, name: String) {
        let shot = XCUIScreen.main.screenshot()
        let attachment = XCTAttachment(screenshot: shot)
        attachment.name = name
        attachment.lifetime = .keepAlways
        add(attachment)

        let dir = "/tmp/ios_screenshots"
        try? FileManager.default.createDirectory(atPath: dir, withIntermediateDirectories: true)
        try? shot.pngRepresentation.write(to: URL(fileURLWithPath: "\(dir)/\(name).png"))
    }
}
