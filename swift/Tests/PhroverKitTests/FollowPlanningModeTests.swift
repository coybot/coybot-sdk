import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPlanningModeTests: XCTestCase {
    func testReactivePlanDoesNotReadMeshOrInventAFreeCostmap() async throws {
        let plan = await FollowPlanningPolicy.plan(mode: .reactiveDepth, start: .zero, goal: Vec2(2, 0)) {
            XCTFail("Basic following must not evaluate historical mesh or floor confidence")
            return .init(path: [], rejection: .startBlocked)
        }
        let result = try XCTUnwrap(plan)
        XCTAssertEqual(result.path, [Vec2(2, 0)])
        XCTAssertNil(result.map)
        XCTAssertTrue(result.isDirectDepthRoute)
        let updated = try XCTUnwrap(result.retargeted(to: Vec2(2, 1), from: .zero))
        XCTAssertEqual(updated.path, [Vec2(2, 1)])
        XCTAssertEqual(updated.planningMode, .reactiveDepth)
    }

    func testMeshModeRetainsMapRejectionsAndInvalidTargetsFailClosed() async {
        var calls = 0
        let blocked = await FollowPlanningPolicy.plan(mode: .mesh, start: .zero, goal: Vec2(2, 0)) {
            calls += 1
            return .init(path: [], rejection: .startBlocked)
        }
        XCTAssertEqual(calls, 1)
        XCTAssertEqual(blocked?.rejection, .startBlocked)
        let invalid = await FollowPlanningPolicy.plan(mode: .reactiveDepth, start: .zero, goal: Vec2(.nan, 0)) {
            XCTFail("Invalid target must not be planned"); return nil
        }
        XCTAssertNil(invalid)
    }
}
