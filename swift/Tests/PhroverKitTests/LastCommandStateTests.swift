import XCTest
@testable import PhroverKit

final class LastCommandStateTests: XCTestCase {
    func testCaptureStartImmediatelyCreatesListeningDraftAndPartialUpdatesIt() throws {
        var state = LastCommandState()
        let captureID = SpeechCaptureID(rawValue: 1)

        state.reduce(.started(id: captureID))

        XCTAssertEqual(state.record?.captureID, captureID)
        XCTAssertEqual(state.record?.command, "Listening…")
        XCTAssertEqual(state.record?.status, .listening)
        XCTAssertEqual(state.record?.message, "Listening…")

        state.reduce(.partial(id: captureID, transcript: "go to the other room"))

        XCTAssertEqual(state.record?.command, "go to the other room")
        XCTAssertEqual(state.record?.status, .listening)
    }

    func testEmptyPartialDoesNotEraseRetainedDraftText() {
        var state = LastCommandState()
        let captureID = SpeechCaptureID(rawValue: 1)
        state.reduce(.started(id: captureID))
        state.reduce(.partial(id: captureID, transcript: "stop"))

        state.reduce(.partial(id: captureID, transcript: "   "))

        XCTAssertEqual(state.record?.command, "stop")
    }

    func testNoSpeechFailureMakesDraftActionable() {
        var state = LastCommandState()
        let captureID = SpeechCaptureID(rawValue: 1)
        state.reduce(.started(id: captureID))

        state.reduce(.failed(id: captureID, message: "No speech detected. Try again."))

        XCTAssertEqual(state.record?.captureID, captureID)
        XCTAssertEqual(state.record?.status, .failed)
        XCTAssertEqual(state.record?.message, "No speech detected. Try again.")
    }

    func testFinalClosesDraftAndIdentifiedMissionStatusReplacesIt() throws {
        var state = LastCommandState()
        let captureID = SpeechCaptureID(rawValue: 1)
        state.reduce(.started(id: captureID))
        state.reduce(.partial(id: captureID, transcript: "stop"))

        state.reduce(.recognized(id: 7, command: "stop"))
        XCTAssertEqual(state.record?.captureID, captureID)
        XCTAssertEqual(state.record?.status, .listening)

        state.finalizeCapture(captureID)
        state.reduce(.recognized(id: 7, command: "stop"))

        let record = try XCTUnwrap(state.record)
        XCTAssertNil(record.captureID)
        XCTAssertEqual(record.id, 7)
        XCTAssertEqual(record.command, "stop")
        XCTAssertEqual(record.status, .recognized)
    }

    func testStaleCaptureEventsAndFinalCannotOverwriteNewerDraft() {
        var state = LastCommandState()
        let first = SpeechCaptureID(rawValue: 1)
        let second = SpeechCaptureID(rawValue: 2)
        state.reduce(.started(id: first))
        state.reduce(.started(id: second))

        state.reduce(.partial(id: first, transcript: "stale"))
        state.reduce(.failed(id: first, message: "stale failure"))
        state.finalizeCapture(first)

        XCTAssertEqual(state.record?.captureID, second)
        XCTAssertEqual(state.record?.command, "Listening…")
        XCTAssertEqual(state.record?.status, .listening)
    }

    func testOlderTerminalCannotOverwriteNewerIdenticalCommand() throws {
        var state = LastCommandState()

        state.reduce(.recognized(id: 1, command: "go to the other room"))
        state.reduce(.working(id: 1, command: "go to the other room"))
        state.reduce(.recognized(id: 2, command: "go to the other room"))
        state.reduce(.working(id: 2, command: "go to the other room"))
        state.reduce(.failed(id: 1, command: "go to the other room", message: "Older failure"))

        let record = try XCTUnwrap(state.record)
        XCTAssertEqual(record.id, 2)
        XCTAssertEqual(record.status, .working)
        XCTAssertEqual(record.message, "Working")
    }

    func testOlderRecognizedCannotOverwriteNewerIdenticalCommand() throws {
        var state = LastCommandState()
        state.reduce(.recognized(id: 2, command: "go to the other room"))
        state.reduce(.working(id: 2, command: "go to the other room"))

        state.reduce(.recognized(id: 1, command: "go to the other room"))

        let record = try XCTUnwrap(state.record)
        XCTAssertEqual(record.id, 2)
        XCTAssertEqual(record.status, .working)
    }

    func testOlderRecognizedCannotOverwriteNewerDifferentCommand() throws {
        var state = LastCommandState()
        state.reduce(.recognized(id: 2, command: "stop"))

        state.reduce(.recognized(id: 1, command: "go forward"))

        let record = try XCTUnwrap(state.record)
        XCTAssertEqual(record.id, 2)
        XCTAssertEqual(record.command, "stop")
        XCTAssertEqual(record.status, .recognized)
    }

    func testSuccessProgressionPersistsItsTerminalRecord() throws {
        var state = LastCommandState()

        state.reduce(.recognized(id: 1, command: "go to the other room"))
        state.reduce(.working(id: 1, command: "go to the other room"))
        state.reduce(.succeeded(id: 1, command: "go to the other room", message: "Entered another room."))

        let record = try XCTUnwrap(state.record)
        XCTAssertEqual(record.command, "go to the other room")
        XCTAssertEqual(record.status, .succeeded)
        XCTAssertEqual(record.message, "Entered another room.")
    }

    func testFailureAndCancellationPersistUntilNewRecognitionReplacesThem() throws {
        var state = LastCommandState()
        state.reduce(.recognized(id: 1, command: "turn around"))
        state.reduce(.working(id: 1, command: "turn around"))
        state.reduce(.failed(id: 1, command: "turn around", message: "I can’t safely turn."))

        XCTAssertEqual(state.record?.status, .failed)
        XCTAssertEqual(state.record?.message, "I can’t safely turn.")

        state.reduce(.cancelled(id: 1, command: "turn around"))
        XCTAssertEqual(state.record?.status, .cancelled)
        XCTAssertEqual(state.record?.message, "Cancelled")

        state.reduce(.recognized(id: 2, command: "stop"))
        XCTAssertEqual(state.record?.command, "stop")
        XCTAssertEqual(state.record?.status, .recognized)
        XCTAssertEqual(state.record?.message, "Recognized")
    }
}
