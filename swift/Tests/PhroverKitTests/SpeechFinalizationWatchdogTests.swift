import XCTest
@testable import PhroverKit

@MainActor
final class SpeechFinalizationWatchdogTests: XCTestCase {
    func testBlankFinalTranscriptFallsBackToLastNonEmptyPartial() {
        XCTAssertEqual(
            SpeechTranscriptSelection.finalTranscript(
                latest: "   ",
                lastNonEmptyPartial: "  Go to Zia's room  "
            ),
            "Go to Zia's room"
        )
    }

    func testNonEmptyFinalTranscriptWinsOverPartial() {
        XCTAssertEqual(
            SpeechTranscriptSelection.finalTranscript(
                latest: "Go to the kitchen",
                lastNonEmptyPartial: "Go to the other room"
            ),
            "Go to the kitchen"
        )
    }

    func testTimeoutSubmitsTrimmedPartialTranscript() async {
        let watchdog = SpeechFinalizationWatchdog(timeout: .milliseconds(10))
        let result = expectation(description: "fallback transcript submitted")
        var submittedTranscript: String?

        watchdog.schedule(partialTranscript: { "  Go to the chair  " }) { transcript in
            submittedTranscript = transcript
            result.fulfill()
        }

        await fulfillment(of: [result], timeout: 1)
        XCTAssertEqual(submittedTranscript, "Go to the chair")
    }

    func testTimeoutReturnsNilWhenNoSpeechWasRecognized() async {
        let watchdog = SpeechFinalizationWatchdog(timeout: .milliseconds(10))
        let result = expectation(description: "empty capture completed")
        var didComplete = false
        var submittedTranscript: String?

        watchdog.schedule(partialTranscript: { "   " }) { transcript in
            didComplete = true
            submittedTranscript = transcript
            result.fulfill()
        }

        await fulfillment(of: [result], timeout: 1)
        XCTAssertTrue(didComplete)
        XCTAssertNil(submittedTranscript)
    }

    func testCancelPreventsTimeoutCompletion() async throws {
        let watchdog = SpeechFinalizationWatchdog(timeout: .milliseconds(30))
        var didComplete = false

        watchdog.schedule(partialTranscript: { "Go forward" }) { _ in
            didComplete = true
        }
        watchdog.cancel()

        try await Task.sleep(for: .milliseconds(60))
        XCTAssertFalse(didComplete)
    }
}
