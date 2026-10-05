import XCTest
@testable import PhroverKit

final class CostmapBuilderTests: XCTestCase {
    func testFloorEstimateUsesGeometryAndRetainsLowFurniture() {
        let heights: [Float] = Array(repeating: 0.02, count: 20)
            + [0.18, 0.22, 0.75, 2.40]
        let floor = CostmapBuilder.estimatedFloorHeight(fromWorldHeights: heights)

        XCTAssertEqual(floor, 0.02, accuracy: 0.001)
        XCTAssertTrue(CostmapBuilder.isObstacleHeight(0.18, floorY: floor))
        XCTAssertTrue(CostmapBuilder.isObstacleHeight(0.75, floorY: floor))
        XCTAssertFalse(CostmapBuilder.isObstacleHeight(0.02, floorY: floor))
        XCTAssertFalse(CostmapBuilder.isObstacleHeight(2.40, floorY: floor))
    }
}
