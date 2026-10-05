import XCTest
@testable import RoverNav

final class FrontierFinderTests: XCTestCase {
    func testStraightBoundaryDirectionPointsTowardUnknownCells() throws {
        let size = 7
        let map = Costmap(width: size, height: size, resolution: 0.1, origin: .zero)
        var observed = ObservedGrid(matching: map)
        for cy in 0..<size {
            for cx in 0...3 {
                observed.markObserved(at: map.cellCenter(cx, cy))
            }
        }

        let frontier = try XCTUnwrap(FrontierFinder.candidates(costmap: map, observed: observed).first)
        let direction = try XCTUnwrap(frontier.outwardDirection)

        XCTAssertEqual(direction.length, 1, accuracy: 1e-12)
        XCTAssertEqual(direction.x, 1, accuracy: 1e-12)
        XCTAssertEqual(direction.y, 0, accuracy: 1e-12)
    }

    func testRotatedBoundaryDirectionPointsTowardUnknownCells() throws {
        let size = 7
        let map = Costmap(width: size, height: size, resolution: 0.1, origin: .zero)
        var observed = ObservedGrid(matching: map)
        for cy in 0..<size {
            for cx in 0..<size where cx + cy <= size - 1 {
                observed.markObserved(at: map.cellCenter(cx, cy))
            }
        }

        let frontier = try XCTUnwrap(FrontierFinder.candidates(costmap: map, observed: observed).first)
        let direction = try XCTUnwrap(frontier.outwardDirection)
        let diagonalComponent = 1.0 / Double(2).squareRoot()

        XCTAssertEqual(direction.length, 1, accuracy: 1e-12)
        XCTAssertEqual(direction.x, diagonalComponent, accuracy: 1e-12)
        XCTAssertEqual(direction.y, diagonalComponent, accuracy: 1e-12)
    }

    func testSymmetricBoundaryKeepsFrontierWithoutDirection() throws {
        let size = 7
        let map = Costmap(width: size, height: size, resolution: 0.1, origin: .zero)
        var observed = ObservedGrid(matching: map)
        for cx in 2...4 {
            observed.markObserved(at: map.cellCenter(cx, 3))
        }

        let frontiers = FrontierFinder.candidates(costmap: map, observed: observed)

        XCTAssertEqual(frontiers.count, 1)
        XCTAssertEqual(frontiers[0].cellCount, 3)
        XCTAssertNil(frontiers[0].outwardDirection)
    }

    /// A 20x20 map, fully observed on the left half, unobserved on the right, with a wall
    /// along the boundary column broken by two door-sized gaps. The frontier cells are the
    /// free boundary cells inside the gaps — expect exactly two clusters, one per gap.
    func testTwoDoorGapsYieldTwoCandidates() {
        let size = 20
        var map = Costmap(width: size, height: size, resolution: 0.1, origin: .zero)
        var observed = ObservedGrid(matching: map)

        let wallX = 9
        for cy in 0..<size {
            for cx in 0...wallX {
                observed.markObserved(at: map.cellCenter(cx, cy))
            }
        }
        // Wall along wallX with gaps at rows 3-5 and 13-15.
        for cy in 0..<size where !(3...5).contains(cy) && !(13...15).contains(cy) {
            map.setCost(wallX, cy, Costmap.lethal)
        }

        let frontiers = FrontierFinder.candidates(costmap: map, observed: observed)

        XCTAssertEqual(frontiers.count, 2, "expected one frontier cluster per door gap, got \(frontiers)")
        let ys = frontiers.map(\.centroid.y).sorted()
        // Gap rows 3-5 center on cell 4 (y = 0.45m), rows 13-15 on cell 14 (y = 1.45m).
        XCTAssertEqual(ys[0], 0.45, accuracy: 0.06)
        XCTAssertEqual(ys[1], 1.45, accuracy: 0.06)
        for f in frontiers {
            XCTAssertEqual(f.centroid.x, 0.95, accuracy: 0.06, "frontier should sit in the gap column")
            XCTAssertEqual(f.widthMeters, 0.3, accuracy: 0.05, "3-cell gap at 0.1m/cell")
        }
    }

    func testFullyObservedMapYieldsNoCandidates() {
        let size = 10
        var map = Costmap(width: size, height: size, resolution: 0.1, origin: .zero)
        var observed = ObservedGrid(matching: map)
        for cy in 0..<size {
            for cx in 0..<size {
                observed.markObserved(at: map.cellCenter(cx, cy))
            }
        }
        XCTAssertTrue(FrontierFinder.candidates(costmap: map, observed: observed).isEmpty)
    }

    func testBlockedBoundaryIsNotAFrontier() {
        // Left half observed, right half not, but the entire boundary column is wall:
        // nowhere to go — no candidates.
        let size = 10
        var map = Costmap(width: size, height: size, resolution: 0.1, origin: .zero)
        var observed = ObservedGrid(matching: map)
        for cy in 0..<size {
            for cx in 0...4 {
                observed.markObserved(at: map.cellCenter(cx, cy))
            }
            map.setCost(4, cy, Costmap.lethal)
        }
        XCTAssertTrue(FrontierFinder.candidates(costmap: map, observed: observed).isEmpty)
    }

    func testTinyClustersDroppedAsNoise() {
        let size = 10
        var map = Costmap(width: size, height: size, resolution: 0.1, origin: .zero)
        var observed = ObservedGrid(matching: map)
        // Observe everything except two isolated cells — their observed neighbors become
        // frontier cells, but in clusters smaller than minCells.
        for cy in 0..<size {
            for cx in 0..<size where !((cx == 2 && cy == 2) || (cx == 7 && cy == 7)) {
                observed.markObserved(at: map.cellCenter(cx, cy))
            }
        }
        XCTAssertTrue(FrontierFinder.candidates(costmap: map, observed: observed, minCells: 5).isEmpty)
    }
}
