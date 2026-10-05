import XCTest
@testable import PhroverKit

final class OfflineObjectMissionIntentParserTests: XCTestCase {
    func testParsesVisibleObjectWithoutImplicitReturn() throws {
        let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse("Go to the fridge"))
        XCTAssertEqual(intent.objectQuery, "refrigerator")
        XCTAssertEqual(intent.targetLabel, "refrigerator")
        XCTAssertEqual(intent.requestedColors, [])
        XCTAssertFalse(intent.searchOtherRooms)
        XCTAssertFalse(intent.shouldReturn)
    }

    func testParsesColorOtherRoomAndExplicitReturn() throws {
        let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse(
            "Go to the black chair in the other room and come back"
        ))
        XCTAssertEqual(intent.objectQuery, "black chair")
        XCTAssertEqual(intent.targetLabel, "chair")
        XCTAssertEqual(intent.requestedColors, [.black])
        XCTAssertTrue(intent.searchOtherRooms)
        XCTAssertTrue(intent.shouldReturn)
    }

    func testParsesThenComeBackAfterOtherRoom() throws {
        let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse(
            "Go to the green chair in the other room, then come back"
        ))
        XCTAssertEqual(intent.objectQuery, "green chair")
        XCTAssertEqual(intent.targetLabel, "chair")
        XCTAssertEqual(intent.requestedColors, [.green])
        XCTAssertTrue(intent.searchOtherRooms)
        XCTAssertTrue(intent.shouldReturn)
    }

    func testParsesNamedRoomWithoutIncludingItInTargetLabel() throws {
        let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse(
            "Go to the black chair in our room"
        ))

        XCTAssertEqual(intent.objectQuery, "black chair")
        XCTAssertEqual(intent.targetLabel, "chair")
        XCTAssertEqual(intent.requestedColors, [.black])
        XCTAssertTrue(intent.searchOtherRooms)
        XCTAssertFalse(intent.shouldReturn)
    }

    func testParsesExplicitGoBackReturn() throws {
        let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse(
            "Go to the chair and go back"
        ))
        XCTAssertEqual(intent.objectQuery, "chair")
        XCTAssertEqual(intent.targetLabel, "chair")
        XCTAssertTrue(intent.shouldReturn)
    }

    func testScreenAliasesParseToCanonicalScreenTarget() throws {
        for command in [
            "Go to the monitor",
            "Go to the computer monitor",
            "Go to the display",
            "Go to the computer screen",
            "Go to the television",
            "Go to the TV",
        ] {
            let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse(command), command)
            XCTAssertEqual(intent.objectQuery, "screen", command)
            XCTAssertEqual(intent.targetLabel, "screen", command)
        }
    }

    func testRejectsCommandsOutsideObjectNavigationContract() {
        XCTAssertNil(OfflineObjectMissionIntentParser.parse("Tell me a joke"))
        XCTAssertNil(OfflineObjectMissionIntentParser.parse("Go to the chair and bring me a book"))
        XCTAssertNil(OfflineObjectMissionIntentParser.parse("Go to"))
    }
}
