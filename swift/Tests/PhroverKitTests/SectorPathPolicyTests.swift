import XCTest
import RoverNav
@testable import PhroverKit

final class SectorPathPolicyTests: XCTestCase {
    func testWestAndEastPoliciesAcceptOnlyTheirOpenSectors() {
        let frame = SharedMissionFrame(localOrigin: .zero, localNorthHeading: 0, sessionGeneration: 1)!

        XCTAssertEqual(
            SectorPathPolicy(sector: .west, frame: frame).evaluate(path: [Vec2(0, 0.26)]),
            .admissible
        )
        XCTAssertEqual(
            SectorPathPolicy(sector: .east, frame: frame).evaluate(path: [Vec2(0, -0.26)]),
            .admissible
        )
    }

    func testPolicyUsesSharedMissionFrameTransformationForEveryPoint() {
        let frame = SharedMissionFrame(
            localOrigin: Vec2(4, 5),
            localNorthHeading: .pi / 2,
            sessionGeneration: 1
        )!
        let policy = SectorPathPolicy(sector: .west, frame: frame)

        XCTAssertEqual(policy.evaluate(path: [Vec2(3.6, 5), Vec2(3.7, 5.2)]), .admissible)
        XCTAssertEqual(
            policy.evaluate(path: [Vec2(3.6, 5), Vec2(4.3, 5)]),
            .rejected(.outsideSector(pointIndex: 1))
        )
    }

    func testBoundaryCenterBandAndOppositeSectorAreRejected() {
        let frame = SharedMissionFrame(localOrigin: .zero, localNorthHeading: 0, sessionGeneration: 1)!
        let west = SectorPathPolicy(sector: .west, frame: frame)

        XCTAssertEqual(west.evaluate(path: [Vec2(0, 0.251)]), .admissible)
        XCTAssertEqual(west.evaluate(path: [Vec2(0, 0.25)]), .rejected(.outsideSector(pointIndex: 0)))
        XCTAssertEqual(west.evaluate(path: [Vec2(0, 0)]), .rejected(.outsideSector(pointIndex: 0)))
        XCTAssertEqual(west.evaluate(path: [Vec2(0, -0.251)]), .rejected(.outsideSector(pointIndex: 0)))
    }

    func testNonFinitePointFailsClosed() {
        let frame = SharedMissionFrame(localOrigin: .zero, localNorthHeading: 0, sessionGeneration: 1)!
        let policy = SectorPathPolicy(sector: .west, frame: frame)

        XCTAssertEqual(
            policy.evaluate(path: [Vec2(.nan, 1)]),
            .rejected(.invalidPoint(pointIndex: 0))
        )
    }
}
