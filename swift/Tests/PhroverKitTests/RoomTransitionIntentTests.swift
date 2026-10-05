import XCTest
@testable import PhroverKit

final class RoomTransitionIntentTests: XCTestCase {
    func testMatchesStandaloneRoomTransitions() {
        let utterances = [
            "go to the other room",
            "move into another room",
            "enter the next room",
            "Go to the OTHER ROOM!",
            "Move into another room.",
            "Enter the next room?",
        ]

        for utterance in utterances {
            XCTAssertTrue(RoomTransitionIntent.matches(utterance), utterance)
        }
    }

    func testRejectsObjectTargetsAndUnrelatedCommands() {
        let utterances = [
            "go to the chair in the other room",
            "find another room's table",
            "stop",
            "",
            "   ",
            "the room is cold",
            "make room for me",
        ]

        for utterance in utterances {
            XCTAssertFalse(RoomTransitionIntent.matches(utterance), utterance)
        }
    }
}
