import XCTest
import RoverNav
@testable import PhroverKit

final class ClassifiedFloorTrackerTests: XCTestCase {
    private let boundary = [Vec2(-1, -1), Vec2(1, -1), Vec2(1, 1), Vec2(-1, 1)]
    private func plane(_ height: Double = -0.42) -> ClassifiedFloorTracker.Plane {
        .init(origin: SIMD3(0, height, 0), normal: SIMD3(0, 1, 0), boundary: boundary)!
    }
    private func stable(_ tracker: inout ClassifiedFloorTracker, id: UUID, height: Double = -0.42) {
        for time in [10.0, 10.1, 10.2] { tracker.record(id: id, timestamp: time, plane: plane(height)) }
    }

    func testRequiresDistinctUpdatesAndStableGeometryWithinObservedFootprint() throws {
        var tracker = ClassifiedFloorTracker()
        let id = UUID()
        for _ in 0..<10 { tracker.record(id: id, timestamp: 10, plane: plane()) }
        XCTAssertNil(tracker.reference(at: .zero, now: 10.2))
        stable(&tracker, id: id)
        XCTAssertEqual(try XCTUnwrap(tracker.reference(at: .zero, now: 10.3)), -0.42, accuracy: 1e-6)
        XCTAssertNil(tracker.reference(at: Vec2(0.9, 0), now: 10.3))
        XCTAssertNil(tracker.reference(at: Vec2(2, 0), now: 10.3))
        XCTAssertNil(tracker.reference(at: .zero, now: 12.01), "The whole stability window must be recent")
        XCTAssertNil(tracker.reference(at: .zero, now: 9.9))
        XCTAssertNil(tracker.reference(at: .zero, now: .nan))
        tracker.record(id: id, timestamp: 10.21, plane: plane(-0.3))
        XCTAssertNil(tracker.reference(at: .zero, now: 10.3), "A rapid conflicting update cannot be coalesced away")
    }

    func testRemovalReclassificationResetAndConflictingLevelsRevokeEvidence() {
        for fault in ["remove", "reclassify", "reset", "regress", "conflict"] {
            var tracker = ClassifiedFloorTracker()
            let id = UUID()
            stable(&tracker, id: id)
            switch fault {
            case "remove": tracker.remove([id])
            case "reclassify": tracker.record(id: id, timestamp: 10.3, plane: nil)
            case "reset": tracker.clear()
            case "regress": tracker.record(id: id, timestamp: 9, plane: plane())
            default: tracker.record(id: UUID(), timestamp: 10.3, plane: plane(-0.6))
            }
            XCTAssertNil(tracker.reference(at: .zero, now: 10.4), fault)
        }
    }

    func testFrequentGeometryUpdatesCanEstablishTheFullStabilityWindow() {
        var tracker = ClassifiedFloorTracker()
        let id = UUID()
        for index in 0..<40 { tracker.record(id: id, timestamp: 10 + Double(index) / 60, plane: plane()) }
        XCTAssertNotNil(tracker.reference(at: .zero, now: 10.7))
    }

    func testRejectsSteepMalformedAndUnsupportedBoundaries() {
        XCTAssertNil(ClassifiedFloorTracker.Plane(origin: .zero, normal: SIMD3(0.2, 1, 0), boundary: boundary))
        XCTAssertNil(ClassifiedFloorTracker.Plane(origin: SIMD3(0, .nan, 0), normal: SIMD3(0, 1, 0), boundary: boundary))
        let concave = [Vec2(-1, -1), Vec2(1, -1), Vec2(1, -0.5), Vec2(-0.5, -0.5), Vec2(-0.5, 1), Vec2(-1, 1)]
        let plane = ClassifiedFloorTracker.Plane(origin: .zero, normal: SIMD3(0, 1, 0), boundary: concave)
        XCTAssertNil(plane?.height(at: .zero), "A bounding-box hit is not observed floor support")
    }
}
