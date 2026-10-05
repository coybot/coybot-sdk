import XCTest
@testable import PhroverKit

@MainActor
final class SpeechCaptureEventTests: XCTestCase {
    func testStartingCapturesImmediatelyEmitsMonotonicIdentifiers() {
        let lifecycle = SpeechCaptureLifecycle()
        var events: [SpeechCaptureEvent] = []

        let first = lifecycle.begin(onEvent: { events.append($0) }, onFinal: { _, _ in })
        let second = lifecycle.begin(onEvent: { events.append($0) }, onFinal: { _, _ in })

        XCTAssertEqual(first, SpeechCaptureID(rawValue: 1))
        XCTAssertEqual(second, SpeechCaptureID(rawValue: 2))
        XCTAssertEqual(events, [.started(id: first), .started(id: second)])
    }

    func testOnlyNonEmptyPartialsAreEmittedAndPartialsNeverFinalize() {
        let lifecycle = SpeechCaptureLifecycle()
        var events: [SpeechCaptureEvent] = []
        var finalTranscripts: [(SpeechCaptureID, String)] = []
        let id = lifecycle.begin(
            onEvent: { events.append($0) },
            onFinal: { finalTranscripts.append(($0, $1)) }
        )

        lifecycle.receivePartial("  Go to the other room  ", for: id)
        lifecycle.receivePartial("   ", for: id)

        XCTAssertEqual(events, [
            .started(id: id),
            .partial(id: id, transcript: "Go to the other room"),
        ])
        XCTAssertTrue(finalTranscripts.isEmpty)
    }

    func testFinalClosesCaptureAndRejectsItsLaterCallbacks() {
        let lifecycle = SpeechCaptureLifecycle()
        var events: [SpeechCaptureEvent] = []
        var finalTranscripts: [(SpeechCaptureID, String)] = []
        let id = lifecycle.begin(
            onEvent: { events.append($0) },
            onFinal: { finalTranscripts.append(($0, $1)) }
        )

        lifecycle.complete("  Stop  ", for: id)
        lifecycle.receivePartial("stale", for: id)
        lifecycle.fail(id: id, message: "stale failure")

        XCTAssertEqual(finalTranscripts.count, 1)
        XCTAssertEqual(finalTranscripts.first?.0, id)
        XCTAssertEqual(finalTranscripts.first?.1, "Stop")
        XCTAssertEqual(events, [.started(id: id)])
    }

    func testBlankFinalFailsAsNoSpeechAndClosesCapture() {
        let lifecycle = SpeechCaptureLifecycle()
        var events: [SpeechCaptureEvent] = []
        var finalTranscripts: [(SpeechCaptureID, String)] = []
        let id = lifecycle.begin(
            onEvent: { events.append($0) },
            onFinal: { finalTranscripts.append(($0, $1)) }
        )

        lifecycle.complete("   ", for: id)

        XCTAssertEqual(events, [
            .started(id: id),
            .failed(id: id, message: "No speech detected. Try again."),
        ])
        XCTAssertTrue(finalTranscripts.isEmpty)
        XCTAssertNil(lifecycle.activeID)
    }

    func testAppleNoSpeechRecognizerErrorProducesActionableFailure() {
        let lifecycle = SpeechCaptureLifecycle()
        var events: [SpeechCaptureEvent] = []
        let id = lifecycle.begin(onEvent: { events.append($0) }, onFinal: { _, _ in })
        let error = NSError(
            domain: "kAFAssistantErrorDomain",
            code: 1110,
            userInfo: [NSLocalizedDescriptionKey: "No speech detected"]
        )

        lifecycle.fail(id: id, recognitionError: error)

        XCTAssertEqual(events, [
            .started(id: id),
            .failed(id: id, message: "No speech detected. Try again."),
        ])
        XCTAssertNil(lifecycle.activeID)
    }

    func testNoSpeechFailureClosesCaptureAndStaleCaptureCannotAffectNewerOne() {
        let lifecycle = SpeechCaptureLifecycle()
        var events: [SpeechCaptureEvent] = []
        var finalTranscripts: [(SpeechCaptureID, String)] = []
        let first = lifecycle.begin(
            onEvent: { events.append($0) },
            onFinal: { finalTranscripts.append(($0, $1)) }
        )
        let second = lifecycle.begin(
            onEvent: { events.append($0) },
            onFinal: { finalTranscripts.append(($0, $1)) }
        )

        lifecycle.receivePartial("stale", for: first)
        lifecycle.complete("stale", for: first)
        lifecycle.fail(id: first, message: "stale failure")
        lifecycle.fail(id: second, message: "No speech detected. Try again.")
        lifecycle.receivePartial("too late", for: second)

        XCTAssertEqual(events, [
            .started(id: first),
            .started(id: second),
            .failed(id: second, message: "No speech detected. Try again."),
        ])
        XCTAssertTrue(finalTranscripts.isEmpty)
    }
}
