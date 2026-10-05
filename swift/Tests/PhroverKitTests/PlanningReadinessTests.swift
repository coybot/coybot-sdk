import XCTest
import RoverNav
@testable import PhroverKit

final class PlanningReadinessTests: XCTestCase {
    func testTrackingFlickerResetsNormalStreak() {
        var tracker = PlanningReadinessTracker(sessionGeneration: 7)
        tracker.ingest(observation(sequence: 1, quality: .normal))
        tracker.ingest(observation(sequence: 2, quality: .limited))
        tracker.ingest(observation(sequence: 3, quality: .normal))

        XCTAssertEqual(tracker.snapshot.normalObservationStreak, 1)
        XCTAssertFalse(tracker.snapshot.isPoseReady)
    }

    func testThreeNormalsEnablePoseReadinessAndTrustedMeshRevision() {
        var tracker = PlanningReadinessTracker(sessionGeneration: 7)
        for sequence in 1...3 {
            tracker.ingest(observation(sequence: UInt64(sequence), quality: .normal))
        }

        XCTAssertTrue(tracker.snapshot.isPoseReady)
        XCTAssertTrue(tracker.recordTrustedMeshUpdate())
        XCTAssertEqual(tracker.snapshot.trustedMeshRevision, 1)
    }

    func testMeshRevisionDoesNotAdvanceBeforePoseReadiness() {
        var tracker = PlanningReadinessTracker(sessionGeneration: 7)
        tracker.ingest(observation(sequence: 1, quality: .normal))

        XCTAssertFalse(tracker.recordTrustedMeshUpdate())
        XCTAssertEqual(tracker.snapshot.trustedMeshRevision, 0)
    }

    func testResetChangesGenerationAndClearsReadiness() {
        var tracker = PlanningReadinessTracker(sessionGeneration: 7)
        for sequence in 1...3 {
            tracker.ingest(observation(sequence: UInt64(sequence), quality: .normal))
        }
        tracker.recordTrustedMeshUpdate()

        tracker.reset(sessionGeneration: 8)

        XCTAssertEqual(tracker.snapshot, PlanningReadinessSnapshot(
            sessionGeneration: 8,
            normalObservationStreak: 0,
            trustedMeshRevision: 0
        ))
    }

    private func observation(sequence: UInt64, quality: PoseTrackingQuality) -> PoseObservation {
        PoseObservation(
            pose: Pose2D(position: .zero, yaw: 0),
            frameSequence: sequence,
            timestamp: Double(sequence),
            trackingQuality: quality,
            sessionGeneration: 7
        )
    }
}
