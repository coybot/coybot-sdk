import Foundation
import RoverNav
import XCTest
@testable import PhroverKit

final class FollowAssociationDiagnosticsTests: XCTestCase {
    func testRecoveryEvaluationReportsGenerationAndBatchRejectionsBeforeSpatialGate() {
        let candidate = person(3, position: Vec2(4, 0))
        let tracker = FollowTargetTracker()
        let wrongGeneration = tracker.reacquireEvaluated([candidate], lastPosition: Vec2(4, 0), now: 10,
            expectedGeneration: candidate.frameID.generation + 1, frameID: candidate.frameID)
        XCTAssertEqual(wrongGeneration.evaluation.outcome, "lost")
        XCTAssertEqual(wrongGeneration.evaluation.candidates.first?["rejection_reason"], .string("frame_generation_mismatch"))
        let wrongBatch = tracker.reacquireEvaluated([candidate], lastPosition: Vec2(4, 0), now: 10,
            expectedGeneration: candidate.frameID.generation,
            frameID: .init(generation: candidate.frameID.generation, sequence: 99))
        XCTAssertEqual(wrongBatch.evaluation.outcome, "lost")
        XCTAssertEqual(wrongBatch.evaluation.candidates.first?["rejection_reason"], .string("candidate_frame_mismatch"))
    }

    func testDiagnosticHeadingUsesHalfOpenNormalizationAndUnknownHealthStaysExplicit() throws {
        let candidate = person(1, position: Vec2(-1, 0))
        let evaluation = FollowTargetTracker().selectInitialEvaluated([candidate], now: 10).evaluation
        XCTAssertEqual(evaluation.candidates[0]["heading_rad"], .number(-Double.pi))
        let batch = FollowFrameBatch(frameID: candidate.frameID, timestamp: 10, pose: nil,
                                     depthAvailable: false, people: [candidate])
        let payload = evaluation.payload(batch: batch, now: 10, previousOutcome: nil)
        XCTAssertEqual(payload["same_frame"], .null)
        XCTAssertEqual(payload["same_frame_availability"], .string("unknown"))
        XCTAssertEqual(payload["tracking_state"], .null)
        XCTAssertEqual(payload["tracking_state_availability"], .string("unknown"))
        XCTAssertEqual(payload["inference_duration_s"], .null)
        XCTAssertEqual(payload["inference_duration_s_availability"], .string("not_measured"))
        XCTAssertEqual(payload["tracking_reason_availability"], .string("unknown"))
    }

    func testEvaluatedTrackerBoundaryMatrixRetainsLegacyDecisionsAndFiniteRejections() throws {
        let tracker = FollowTargetTracker()
        let invalid = [
            person(1, confidence: .nan), person(2, timestamp: .nan),
            person(3, timestamp: 10.001), person(4, timestamp: 9.499),
            person(5, position: Vec2(.infinity, 0)),
            person(6, pose: Pose2D(position: Vec2(0, .nan), yaw: 0)),
            person(7, pose: Pose2D(position: .zero, yaw: .infinity)),
            person(8, box: CGRect(x: 0, y: 0, width: 0, height: 1)),
            person(9, box: CGRect(x: CGFloat.infinity, y: 0, width: 1, height: 1))
        ]
        let evaluated = tracker.selectInitialEvaluated(invalid, now: 10)
        XCTAssertNil(evaluated.decision)
        XCTAssertNil(tracker.selectInitial(invalid, now: 10))
        XCTAssertEqual(evaluated.evaluation.eligibleCount, 0)
        XCTAssertEqual(evaluated.evaluation.candidates.compactMap { $0["rejection_reason"] },
                       [.string("confidence_nonfinite"), .string("observation_age_nonfinite"),
                        .string("observation_from_future"), .string("observation_stale"),
                        .string("invalid_geometry"), .string("invalid_geometry"), .string("invalid_geometry"),
                        .string("invalid_geometry"), .string("invalid_geometry")])
        XCTAssertEqual(evaluated.evaluation.candidates[4]["range_m"], .null)
        XCTAssertEqual(evaluated.evaluation.candidates[4]["geometry_availability"], .string("nonfinite_paired_geometry"))
        let fresh = person(10, confidence: 0.5, timestamp: 9.5, position: .zero)
        XCTAssertEqual(tracker.selectInitialEvaluated([fresh], now: 10).decision?.frameID, fresh.frameID)
        XCTAssertEqual(tracker.selectInitialEvaluated([], now: 10).evaluation.outcome, "lost")

        // Exact binary interior geometry: IoU is 0.5, centre displacement is 0.125.
        // Edge-touching boxes are rejected by the approved clipping gate before association.
        var config = FollowMeConfiguration()
        config.minimumBoxIoU = 0.5
        config.maximumScreenCenterDistance = 0.0625
        let exact = FollowTargetTracker(configuration: config)
        let prior = person(11, position: .zero, box: CGRect(x: 0.125, y: 0.125, width: 0.375, height: 0.5))
        let iouBoundary = person(12, position: Vec2(0.75, 0), box: CGRect(x: 0.25, y: 0.125, width: 0.375, height: 0.5))
        let iouBelow = person(13, position: .zero, box: CGRect(x: 0.2501, y: 0.125, width: 0.375, height: 0.5))
        XCTAssertEqual(exact.continueTrackEvaluated([iouBoundary], previous: prior, predictedPosition: .zero, now: 10).evaluation.outcome, "continued")
        XCTAssertEqual(exact.continueTrackEvaluated([iouBelow], previous: prior, predictedPosition: .zero, now: 10).evaluation.outcome, "lost")
        config.minimumBoxIoU = 1
        config.maximumScreenCenterDistance = 0.125
        let screenTracker = FollowTargetTracker(configuration: config)
        let screenBoundary = screenTracker.continueTrackEvaluated([iouBoundary], previous: prior, predictedPosition: .zero, now: 10)
        XCTAssertEqual(screenBoundary.evaluation.outcome, "continued")
        guard case .object(let gates) = screenBoundary.evaluation.candidates[0]["gate_metrics"] else { return XCTFail("Missing gates") }
        XCTAssertEqual(gates["box_iou"], .number(0.5))
        XCTAssertEqual(gates["screen_displacement"], .number(0.125))
        XCTAssertEqual(screenTracker.continueTrackEvaluated([iouBelow], previous: prior, predictedPosition: .zero, now: 10).evaluation.outcome, "lost")
        XCTAssertEqual(screenTracker.continueTrackEvaluated([iouBoundary, iouBoundary], previous: prior, predictedPosition: .zero, now: 10).evaluation.outcome, "ambiguous")
        guard case .matched(let selected) = exact.continueTrack([iouBoundary], previous: prior, predictedPosition: .zero, now: 10) else { return XCTFail("Legacy continuity changed") }
        XCTAssertEqual(selected.frameID, iouBoundary.frameID)
        guard case .lost = screenTracker.continueTrack([iouBelow], previous: prior, predictedPosition: .zero, now: 10) else { return XCTFail("Legacy loss changed") }
        guard case .ambiguous = tracker.reacquire([fresh, fresh], lastPosition: .zero, now: 10) else { return XCTFail("Legacy ambiguity changed") }
        let reacquired = tracker.reacquireEvaluated([fresh], lastPosition: .zero, now: 10)
        guard case .matched(let reacquiredPerson) = reacquired.decision else { return XCTFail("Expected one match") }
        XCTAssertEqual(reacquiredPerson.frameID, fresh.frameID)
        XCTAssertEqual(tracker.reacquireEvaluated(invalid, lastPosition: .zero, now: 10).evaluation.matchedCount, 0)
    }

    func testSharedBudgetExactBoundaryTransitionsAndRepeatedLossDeduplication() {
        var budget = FollowSummaryBudget()
        XCTAssertTrue(budget.takeHealthy(now: 0))
        XCTAssertFalse(budget.takeHealthy(now: 0.999))
        XCTAssertTrue(budget.takeHealthy(now: 1))
        XCTAssertTrue(budget.takeAssociation(outcome: "initial", now: 1.01))
        XCTAssertTrue(budget.takeAssociation(outcome: "continued", now: 1.02))
        XCTAssertFalse(budget.takeAssociation(outcome: "continued", now: 1.03))
        XCTAssertFalse(budget.takeHealthy(now: 2.019))
        XCTAssertTrue(budget.takeAssociation(outcome: "continued", now: 2.02))
        XCTAssertFalse(budget.takeHealthy(now: 2.02))
        XCTAssertTrue(budget.takeAssociation(outcome: "lost", now: 2.03))
        XCTAssertFalse(budget.takeAssociation(outcome: "lost", now: 4))
        XCTAssertEqual(budget.previousOutcome, "lost")
        XCTAssertTrue(budget.takeAssociation(outcome: "ambiguous", now: 4.01))
        XCTAssertFalse(budget.takeAssociation(outcome: "ambiguous", now: 5.5))
        XCTAssertTrue(budget.takeAssociation(outcome: "reacquired", now: 5.51))
        budget = FollowSummaryBudget()
        XCTAssertNil(budget.previousOutcome)
        XCTAssertTrue(budget.takeHealthy(now: 5.52))
        XCTAssertFalse(budget.takeHealthy(now: 5))
    }

    @MainActor
    func testAssociationEnvelopeEmptyProjectedListAndPairedGeometryArePrecise() throws {
        var records: [[String: Any]] = []
        let emitter = FollowDiagnosticEmitter(streamID: "association-test", monotonic: { 10 }, utc: { Date(timeIntervalSince1970: 0) }) {
            _, fields in
            records.append(try! JSONSerialization.jsonObject(with: fields["payload"]!.data(using: .utf8)!) as! [String: Any])
        }
        let batch = FollowFrameBatch(frameID: ARFrameID(generation: 1, sequence: 7), timestamp: 9.75,
            pose: Pose2D(position: Vec2(100, 100), yaw: 2), depthAvailable: true, people: [], trackingQuality: .normal)
        let empty = FollowTargetTracker().selectInitialEvaluated([], now: 10).evaluation
        emitter.emit(.init(event: "follow_person.association", context: .init(sessionGeneration: 4, phase: "searching", outcome: "lost"),
                           payload: empty.payload(batch: batch, now: 10, previousOutcome: nil)))
        let event = try XCTUnwrap(records.first)
        XCTAssertEqual(event["schema_version"] as? Int, 1)
        XCTAssertEqual(event["association_outcome"] as? String, "lost")
        XCTAssertEqual(event["projected_person_count"] as? Int, 0)
        XCTAssertEqual(event["reason"] as? String, nil)
        XCTAssertEqual(event["candidate_availability"] as? String, "no projected person candidate available")
        XCTAssertEqual(event["frame_id"] as? String, "1:7")
        XCTAssertEqual(event["observation_age_s"] as? Double, 0.25)
        XCTAssertEqual(event["tracking_state"] as? String, "normal")
        XCTAssertTrue(event["matched_candidate_count"] is NSNull)
        XCTAssertEqual(event["matched_candidate_count_availability"] as? String, "not_applicable_initial_selection")
        XCTAssertTrue(event["selected_candidate"] is NSNull)
        XCTAssertTrue(event["raw_detector_count"] is NSNull)
        XCTAssertEqual(event["raw_detector_count_availability"] as? String, "not_evaluated_or_unknown")
        XCTAssertNil(event["projection_count"])
        let observed = person(7, position: Vec2(3.123456789, 4), pose: Pose2D(position: Vec2(3.123456789, 0), yaw: .pi))
        let evaluation = FollowTargetTracker().selectInitialEvaluated([observed], now: 10).evaluation
        let payload = evaluation.payload(batch: batch, now: 10, previousOutcome: "lost")
        guard case .object(let selected) = payload["selected_candidate"] else { return XCTFail("Missing selected geometry") }
        XCTAssertEqual(selected["range_m"], .number(4))
        XCTAssertEqual(selected["heading_rad"], .number(-Double.pi / 2))
        XCTAssertEqual(selected["projected_position"], .object(["x": .number(3.123456789), "y": .number(4)]))
    }

    private func person(_ index: UInt64, confidence: Float = 0.9, timestamp: Double = 10,
                        position: Vec2 = Vec2(3, 4), pose: Pose2D = Pose2D(position: .zero, yaw: 0),
                        box: CGRect = CGRect(x: 0.4, y: 0.4, width: 0.2, height: 0.2)) -> FollowPersonObservation {
        .init(frameID: ARFrameID(generation: 1, sequence: index), timestamp: timestamp,
              confidence: confidence, boundingBox: box, position: position, pose: pose)
    }

    func testInitialEvaluationPreservesTieOrderInclusiveEligibilityAndSkippedGates() throws {
        let people = [person(1, confidence: 0.499), person(2, confidence: 0.5, timestamp: 9.5), person(3)]
        let result = FollowTargetTracker().selectInitialEvaluated(people, now: 10)
        XCTAssertEqual(result.decision?.frameID, people[1].frameID)
        XCTAssertEqual(result.evaluation.eligibleCount, 2)
        XCTAssertNil(result.evaluation.matchedCount)
        XCTAssertEqual(result.evaluation.selectedIndex, 1)
        XCTAssertEqual(result.evaluation.candidates.count, 3)
        guard result.evaluation.candidates.count == 3 else { return }
        XCTAssertEqual(result.evaluation.candidates[0]["rejection_reason"], .string("confidence_below_minimum"))
        guard case .object(let gates) = result.evaluation.candidates[0]["gate_metrics"] else { return XCTFail("Missing gates") }
        XCTAssertEqual(gates["observation_age_s"], .null)
        XCTAssertEqual(gates["observation_age_s_availability"], .string("not_evaluated"))
        XCTAssertEqual(result.evaluation.candidates[1]["range_m"], .number(5))
        XCTAssertEqual(result.evaluation.candidates[1]["heading_rad"], .number(0.9272952180016122))
        XCTAssertEqual(result.evaluation.candidates[1]["same_frame"], .bool(true))
    }

    func testContinuityAndReacquisitionRecordOnlyExecutedGatesAndMatchCounts() throws {
        let tracker = FollowTargetTracker()
        let previous = person(1, position: .zero)
        let weak = person(2, confidence: 0.499, position: .zero)
        let jump = person(3, position: Vec2(0.751, 0))
        let boundary = person(4, position: Vec2(0.75, 0))
        let result = tracker.continueTrackEvaluated([weak, jump, boundary], previous: previous,
                                                   predictedPosition: .zero, now: 10)
        XCTAssertEqual(result.evaluation.matchedCount, 1)
        XCTAssertEqual(result.evaluation.eligibleCount, 2)
        XCTAssertEqual(result.evaluation.selectedIndex, 2)
        XCTAssertEqual(result.evaluation.candidates.count, 3)
        guard result.evaluation.candidates.count == 3 else { return }
        guard case .object(let skipped) = result.evaluation.candidates[0]["gate_metrics"],
              case .object(let distant) = result.evaluation.candidates[1]["gate_metrics"],
              case .object(let overlap) = result.evaluation.candidates[2]["gate_metrics"] else { return XCTFail("Missing gates") }
        XCTAssertEqual(skipped["world_distance_m"], .null)
        XCTAssertEqual(distant["world_distance_m"], .number(0.751))
        XCTAssertEqual(distant["box_iou_availability"], .string("not_evaluated"))
        guard case .number(let iou) = overlap["box_iou"] else { return XCTFail("Missing IoU") }
        XCTAssertEqual(iou, 1, accuracy: 1e-12)
        XCTAssertEqual(overlap["screen_displacement_availability"], .string("not_evaluated"))
        let near = person(5, position: Vec2(1.5, 0))
        let far = person(6, position: Vec2(1.501, 0))
        let reacquired = tracker.reacquireEvaluated([far, near], lastPosition: .zero, now: 10)
        XCTAssertEqual(reacquired.evaluation.outcome, "reacquired")
        XCTAssertEqual(reacquired.evaluation.matchedCount, 1)
        XCTAssertEqual(reacquired.evaluation.selectedIndex, 1)
        XCTAssertEqual(tracker.reacquireEvaluated([near, near], lastPosition: .zero, now: 10).evaluation.outcome, "ambiguous")
        XCTAssertEqual(tracker.continueTrackEvaluated([], previous: previous, predictedPosition: .zero, now: 10).evaluation.outcome, "lost")
    }
}
