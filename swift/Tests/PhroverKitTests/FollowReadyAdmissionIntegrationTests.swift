import XCTest
import RoverNav
import CoreVideo
import simd
@testable import PhroverKit

@MainActor
final class FollowReadyAdmissionIntegrationTests: XCTestCase {
    func testPendingUnknownTrackingReportsNewestFrameHealthAtActualReadyBoundary() async throws {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        let stopsBefore = fixture.stops
        fixture.send(3, range: 1.6, tracking: nil)
        fixture.clock.onNextRead = { fixture.feedback.release() }
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        guard let rejected = fixture.records.first(where: { $0["event"] as? String == "follow_ready.admission_rejected" }) else {
            // The processor can stop/cancel the request before the controller reaches admission.
            XCTAssertGreaterThan(fixture.stops, stopsBefore)
            XCTAssertEqual(fixture.coordinator.perceptionIssue, .trackingUnavailable)
            XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
            XCTAssertFalse(fixture.records.contains { $0["event"] as? String == "follow_ready.admission_authorized" })
            _ = await fixture.coordinator.stop()
            return
        }
        XCTAssertEqual(rejected["frame_id"] as? String, "1:5", "Report the actual newest pending frame, not older healthy authority")
        XCTAssertEqual(rejected["tracking_state_availability"] as? String, "unknown")
        XCTAssertEqual(rejected["observation_rejection_condition"] as? String, "tracking_unknown",
            "Keep source health facts independently of a stale callback's ownership denial")
        if rejected["pending_frame_evaluation"] as? String == "evaluated" {
            XCTAssertEqual(rejected["rejection_condition"] as? String, "tracking_unknown")
        } else {
            XCTAssertEqual(rejected["pending_frame_evaluation"] as? String, "not_pending",
                "The latest transaction must be captured even after its frame processor won the race")
            XCTAssertEqual(rejected["rejection_condition"] as? String, "operation_replaced",
                "Processed outage stop retains ownership denial; captured facts cannot reauthorize it")
        }
        _ = await fixture.coordinator.stop()
    }

    func testReadyFinalStopRequiresStrictNewCaptureBeforeFixedDepartureBaseline() async {
        let fixture = Fixture()
        await fixture.acquire(aligned: false)
        fixture.clock.advance(to: 0.31)
        fixture.send(2, range: 1.6)
        await drain()
        fixture.clock.advance(to: 0.62)
        fixture.send(3, range: 1.6)
        await drain()
        fixture.clock.advance(to: 0.93)
        fixture.send(4, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.plans, 1)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        fixture.send(5, range: 1.6) // Distinct ID, exactly the final ACK capture time.
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .signalingReady,
            "A same-time capture cannot establish the fixed departure baseline")
        fixture.clock.advance(to: fixture.clock.now + 0.301)
        fixture.send(6, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement)
        XCTAssertEqual(fixture.commands, 1)
        _ = await fixture.coordinator.stop()
    }

    func testAlignmentWaitsForPostDetectionStopMatchedSourceAndDoesNotStarvePendingProcessor() async {
        let fixture = Fixture()
        await fixture.acquire(aligned: false)
        XCTAssertEqual(fixture.stops, 1, "Cached detection cannot launch controller alignment after its stop")
        XCTAssertEqual(fixture.plans, 0)
        fixture.clock.advance(to: 0.31)
        fixture.send(2, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.stops, 2, "Distinct settled matched source launches one controller operation")
        fixture.clock.advance(to: 0.62)
        fixture.send(3, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.stops, 3)
        fixture.clock.advance(to: 0.93)
        fixture.send(4, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.plans, 1, "Pending source can complete alignment and hand off exactly once")
        fixture.feedback.release()
        await drain()
        _ = await fixture.coordinator.stop()
    }

    func testNewestAlignmentStopRejectsDeliveredLateAndEqualCaptureBeforeGenuineMatch() async {
        let fixture = Fixture()
        fixture.holdStopNumber = 3
        await fixture.acquire(aligned: false)
        fixture.clock.advance(to: 0.31)
        fixture.send(2, range: 1.6)
        await drain()
        fixture.clock.advance(to: 0.62)
        fixture.send(3, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.stops, 3, "Stopped controller arrival must reach final coordinator confirmation")
        fixture.clock.advance(to: 0.7, wakeSleepers: false)
        fixture.stopFeedback.release()
        await drain()
        fixture.send(4, range: 1.6, timestamp: 0.65)
        await drain()
        XCTAssertEqual(fixture.plans, 0, "Delivery after ACK does not make a pre-ACK capture fresh")
        fixture.send(5, range: 1.6, timestamp: 0.7)
        await drain()
        XCTAssertEqual(fixture.plans, 0, "Capture equal to ACK must fail strict after")
        fixture.clock.advance(to: 1.01)
        fixture.send(6, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.plans, 1, "A genuine settled post-stop normal matched frame permits ready preflight")
        fixture.feedback.release()
        await drain()
        _ = await fixture.coordinator.stop()
    }

    func testAcceptedRecoveryDecisionCannotAuthorizeNewPendingOrInvalidatedSource() async {
        for condition in ["new_pending", "stale", "generation", "deadline", "late_frame"] {
            let fixture = Fixture()
            fixture.enriched = true
            await fixture.acquire(aligned: false)
            fixture.send(2, range: 1.6, peopleAvailable: false)
            await drain()
            await fixture.recoverReady(range: 1.7)
            fixture.holdReadyAcknowledgement = true
            fixture.feedback.release()
            await fixture.readyFeedback.waitUntilEntered()
            fixture.send(7, range: 2.3, secondPersonRange: 2.6)
            await drain()
            switch condition {
            case "new_pending":
                fixture.send(8, range: 2.3, secondPersonRange: 2.6)
                fixture.clock.onNextRead = { fixture.readyFeedback.release() }
            case "stale":
                fixture.clock.advance(to: fixture.clock.now + 0.501, wakeSleepers: false)
                fixture.readyFeedback.release()
            case "generation":
                fixture.send(8, range: 2.3, generation: 2)
                fixture.clock.onNextRead = { fixture.readyFeedback.release() }
            case "deadline":
                fixture.clock.advance(to: 10, wakeSleepers: false)
                fixture.send(8, range: 2.3)
                fixture.clock.onNextRead = { fixture.readyFeedback.release() }
            default:
                fixture.send(6, range: 0, peopleAvailable: false)
                await drain()
                fixture.readyFeedback.release()
            }
            await drain()
            XCTAssertEqual(fixture.commands, 1, condition)
            let completions = fixture.records.filter { $0["event"] as? String == "follow_ready.completed" }.count
            XCTAssertEqual(completions, condition == "late_frame" ? 1 : 0, condition)
            if condition == "generation" { XCTAssertEqual(fixture.coordinator.state, .failed("AR session reset during recovery.")) }
            if condition == "deadline" { XCTAssertEqual(fixture.coordinator.state, .failed("Person lost.")) }
            _ = await fixture.coordinator.stop()
        }
    }

    func testRecoveryReadyKeepsProcessedUniqueDecisionAcrossHeldPostSendAcknowledgement() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire(aligned: false)
        fixture.send(2, range: 1.6, peopleAvailable: false)
        await drain()
        await fixture.recoverReady(range: 1.7)
        fixture.holdReadyAcknowledgement = true
        fixture.feedback.release()
        await fixture.readyFeedback.waitUntilEntered()
        XCTAssertEqual(fixture.commands, 1)
        fixture.send(7, range: 2.3, secondPersonRange: 2.6)
        await drain() // Original 1.7 m lock uniquely accepts 2.3 m, rejects 2.6 m.
        fixture.readyFeedback.release()
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .signalingReady,
            "The accepted frame must not be reassociated against its own updated 2.3 m lock")
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.completed" }.count, 1)
        XCTAssertEqual(fixture.commands, 1)
        await fixture.fresh(8, range: 2.3)
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement, "Baseline still requires a distinct fresh frame")
        XCTAssertEqual(fixture.commands, 1, "Recovery cannot repeat its consumed ready signal")
        _ = await fixture.coordinator.stop()
    }

    func testPendingUniqueReacquisitionFencesRealScanBeforeAcknowledgementResumes() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire(aligned: false)
        fixture.yaw = 0.5
        fixture.holdAcknowledgement = true
        fixture.send(2, range: 1.6, peopleAvailable: false)
        await drain()
        await fixture.fresh(3, range: 1.6, peopleAvailable: false)
        await fixture.scanFeedback.waitUntilEntered()
        await drain()
        let sends = fixture.wheelCommands.count
        let stops = fixture.stops
        fixture.send(4, range: 3.0) // Within frozen 1.5 m anchor gate; outside continuity.
        fixture.clock.onNextRead = { fixture.scanFeedback.release() }
        await drain()
        XCTAssertEqual(fixture.stopSendCounts.dropFirst(stops).first, sends,
            "A queued unique reacquisition must fence the real controller before another nonzero send")
        XCTAssertGreaterThan(fixture.stops, stops, "Detection must drain confirmed stop before alignment")
        XCTAssertEqual(fixture.coordinator.state, .aligning)
        let retained = try XCTUnwrap(fixture.records.last { $0["event"] as? String == "follow_recovery.retained" })
        XCTAssertEqual(retained["anchor_person_x"] as? Double, 1.6)
        XCTAssertEqual(retained["deadline_s"] as? Double, 10)
        fixture.holdAcknowledgement = false
        _ = await fixture.coordinator.stop()
    }

    func testRealRecoveryReadyFeedbackRetainsDeadlineAndSuccessfulBaselineClearsIt() async {
        for expire in [false, true] {
            let fixture = Fixture()
            await fixture.acquire(aligned: false)
            fixture.send(2, range: 1.6, peopleAvailable: false)
            await drain()
            fixture.enriched = true
            await fixture.recoverReady(range: 1.6)
            if expire {
                fixture.clock.advance(to: 9.9, wakeSleepers: false)
                fixture.send(7, range: 1.6)
                await drain()
                fixture.clock.advance(to: 10, wakeSleepers: false)
            }
            fixture.feedback.release()
            await drain()
            if expire {
                XCTAssertEqual(fixture.coordinator.state, .failed("Person lost."))
                XCTAssertEqual(fixture.commands, 0)
                XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
            } else {
                XCTAssertEqual(fixture.coordinator.state, .signalingReady)
                XCTAssertEqual(fixture.commands, 1)
                await fixture.fresh(7, range: 1.6)
                XCTAssertEqual(fixture.coordinator.state, .waitingForMovement)
                let acceptedFrame = fixture.frameIDs[7]
                fixture.send(8, range: 1.6, peopleAvailable: false)
                await drain()
                let episodes = fixture.records.filter { $0["event"] as? String == "follow_recovery.started" }
                XCTAssertEqual(episodes.count, 2)
                XCTAssertEqual(episodes.last?["anchor_frame_id"] as? String, acceptedFrame.map { "1:\($0)" })
                XCTAssertEqual(fixture.commands, 1)
            }
            _ = await fixture.coordinator.stop()
        }
    }

    func testRecoveryReadyClearanceDeferralCommitsNewestNormalMatchWithoutConsumingAttempt() async throws {
        let fixture = Fixture()
        await fixture.acquire(aligned: false)
        fixture.send(2, range: 1.6, peopleAvailable: false)
        await drain()
        fixture.enriched = true
        await fixture.recoverReady(range: 1.6)
        fixture.send(7, range: 1.3)
        await drain()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .waitingForClearance)
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        fixture.send(8, range: 1.3, peopleAvailable: false)
        await drain()
        let episodes = fixture.records.filter { $0["event"] as? String == "follow_recovery.started" }
        XCTAssertEqual(episodes.count, 2, "Confirmed clearance deferral is an actual healthy normal phase")
        XCTAssertEqual(episodes.last?["anchor_frame_id"] as? String, "1:7")
        _ = await fixture.coordinator.stop()
    }

    func testAcceptedPendingContinuityFreezesItsOriginalPairedMemoryAtLoss() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        fixture.suspendSend = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.7)
        await drain()
        fixture.send(4, range: 2.3, secondPersonRange: 2.6)
        // Resume feedback at the ingress clock check: frame 4 will be queued
        // before the controller resumes, and before its processor is scheduled.
        fixture.clock.onNextRead = { fixture.feedback.release() }
        await fixture.sendFeedback.waitUntilEntered()
        fixture.send(5, range: 0, peopleAvailable: false)
        await drain()
        let loss = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_recovery.started" })
        XCTAssertEqual(loss["anchor_frame_id"] as? String, "1:6")
        let authorized = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_authorized" })
        let pending = authorized["pending_frame_evaluation"] as? String == "evaluated"
        XCTAssertEqual(loss["anchor_association"] as? String, pending ? "acceptedPendingContinuity" : "continued",
            "Memory source must describe whether admission or the normal processor accepted the original association")
        XCTAssertEqual(loss["anchor_person_x"] as? Double, 2.3)
        XCTAssertEqual(loss["anchor_rover_x"] as? Double, 0)
        XCTAssertEqual(loss["anchor_yaw_rad"] as? Double, 0)
        fixture.suspendSend = false
        fixture.sendFeedback.release()
        await drain()
        _ = await fixture.coordinator.stop()
    }

    func testUnknownTrackingOnPendingFrameCannotAuthorizeOlderHealthyObservation() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        let stopsBefore = fixture.stops
        fixture.send(3, range: 1.6, tracking: nil)
        fixture.clock.onNextRead = { fixture.feedback.release() }
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.perceptionIssue, .trackingUnavailable)
        if let rejection = fixture.records.first(where: { $0["event"] as? String == "follow_ready.admission_rejected" }) {
            XCTAssertEqual(rejection["observation_rejection_condition"] as? String, "tracking_unknown")
            if rejection["pending_frame_evaluation"] as? String == "evaluated" {
                XCTAssertEqual(rejection["rejection_condition"] as? String, "tracking_unknown")
            } else {
                XCTAssertEqual(rejection["pending_frame_evaluation"] as? String, "not_pending")
                XCTAssertEqual(rejection["rejection_condition"] as? String, "operation_replaced")
            }
        } else {
            XCTAssertGreaterThan(fixture.stops, stopsBefore)
            XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        }
        _ = await fixture.coordinator.stop()
    }

    func testFinalizedReceiptOwnershipAndFullPauseUseOneMonotonicClock() async throws {
        let fixture = Fixture()
        fixture.stationaryPauseSeconds = 5
        fixture.clock.advance(to: 100)
        let mission = SuspendedMission()
        let router = OperatorCommandRouter(mission: mission, follow: fixture.coordinator,
            clock: fixture.clock, eventSink: { fixture.record($0, fields: $1) })
        let starting = Task { await router.submit("follow me", finalizedTextReceivedAt: 100) }
        await mission.stopFeedback.waitUntilEntered()
        XCTAssertEqual(fixture.coordinator.state, .idle)
        fixture.clock.advance(to: 103)
        mission.stopFeedback.release()
        let result = await starting.value
        XCTAssertEqual(result, .accepted)
        await drain()
        fixture.send(1, range: 1.6)
        await drain()
        fixture.clock.advance(to: 107.999)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .pausing)
        XCTAssertEqual(fixture.plans, 0)
        XCTAssertEqual(fixture.commands, 0)
        fixture.clock.advance(to: 108)
        fixture.send(2, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .aligning)
        for (event, time) in [("operator_command.received", 100.0), ("operator_command.ownership_stop_requested", 100.0),
                              ("operator_command.ownership_stop_confirmed", 103.0), ("operator_command.follow_start_requested", 103.0),
                              ("follow_session.started", 103.0), ("follow_pause.started", 103.0), ("follow_pause.completed", 108.0)] {
            let record = try XCTUnwrap(fixture.records.first { $0["event"] as? String == event }, event)
            XCTAssertEqual(record["monotonic_s"] as? Double, time, event)
            XCTAssertEqual(record["command_received_at_s"] as? Double, 100, event)
            XCTAssertEqual(record["command_elapsed_s"] as? Double, time - 100, event)
        }
        let receipt = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "operator_command.received" })
        XCTAssertEqual(receipt["receipt_origin"] as? String, "finalized_text_receipt")
        XCTAssertEqual(receipt["physical_utterance_time_availability"] as? String, "not_measured")
        let pause = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_pause.completed" })
        XCTAssertEqual(pause["pause_elapsed_s"] as? Double, 5)
        let phase = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_phase" && $0["phase"] as? String == "searching" })
        XCTAssertEqual(phase["previous_phase_elapsed_s"] as? Double, 5)
        XCTAssertEqual(phase["session_elapsed_s"] as? Double, 5)
        _ = await fixture.coordinator.stop()
    }

    func testUnsafeNewestPendingObservationNeverFallsBackToOlderGoodTrack() async throws {
        for condition in ["target_lost", "target_ambiguous", "trackingLimited", "trackingUnavailable",
                          "frame_stale_or_invalid", "frame_timestamp_nonfinite", "frame_from_future", "depthUnavailable", "poseUnavailable",
                          "confidence", "world_distance", "screen_association", "frame_generation_mismatch", "locked_frame_mismatch"] {
            let fixture = Fixture()
            fixture.enriched = true
            await fixture.acquire()
            fixture.send(2, range: 1.6)
            await fixture.feedback.waitUntilEntered()
            let stopsBefore = fixture.stops
            switch condition {
            case "target_lost": fixture.send(3, range: 1.6, peopleAvailable: false)
            case "target_ambiguous": fixture.send(3, range: 1.6, secondPersonRange: 1.7)
            case "trackingLimited": fixture.send(3, range: 1.6, tracking: .limited)
            case "trackingUnavailable": fixture.send(3, range: 1.6, tracking: .unavailable)
            case "frame_stale_or_invalid": fixture.send(3, range: 1.6, timestamp: fixture.clock.now - 0.501)
            case "frame_timestamp_nonfinite": fixture.send(3, range: 1.6, timestamp: .nan)
            case "frame_from_future": fixture.send(3, range: 1.6, timestamp: fixture.clock.now + 0.001)
            case "depthUnavailable": fixture.send(3, range: 1.6, depthAvailable: false)
            case "poseUnavailable": fixture.send(3, range: 1.6, poseAvailable: false)
            case "confidence": fixture.send(3, range: 1.6, confidence: 0.49)
            case "world_distance": fixture.send(3, range: 2.5)
            case "screen_association": fixture.send(3, range: 1.6, box: CGRect(x: 0.9, y: 0.9, width: 0.05, height: 0.05))
            case "frame_generation_mismatch": fixture.send(3, range: 1.6, generation: 2)
            default: fixture.send(3, range: 1.6, personFrameID: .init(generation: 1, sequence: 99))
            }
            fixture.clock.onNextRead = { fixture.feedback.release() }
            await drain()
            XCTAssertEqual(fixture.commands, 0, condition)
            XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.admission_authorized" }.count, 0, condition)
            XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false", condition)
            let rejection = fixture.records.first { $0["event"] as? String == "follow_ready.admission_rejected" }
            if let rejection {
                XCTAssertEqual(rejection["frame_id"] as? String, condition == "frame_generation_mismatch" ? "2:5" : "1:5")
                let expected = ["confidence": "target_lost", "world_distance": "target_lost", "screen_association": "target_lost",
                                "locked_frame_mismatch": "target_lost", "trackingLimited": "tracking_limited",
                                "trackingUnavailable": "tracking_unavailable", "depthUnavailable": "depth_unavailable",
                                "poseUnavailable": "frame_pose_missing", "frame_stale_or_invalid": "frame_stale"][condition] ?? condition
                XCTAssertEqual(rejection["observation_rejection_condition"] as? String, expected)
                if rejection["pending_frame_evaluation"] as? String == "evaluated" {
                    XCTAssertEqual(rejection["rejection_condition"] as? String, expected)
                } else {
                    XCTAssertEqual(rejection["pending_frame_evaluation"] as? String, "not_pending")
                    XCTAssertEqual(rejection["rejection_condition"] as? String, "operation_replaced",
                        "A frame already stopped this request; late admission cannot reuse its old authority")
                }
                if let candidateReason = ["confidence": "confidence_below_minimum", "world_distance": "world_distance_exceeded",
                                          "screen_association": "screen_association_rejected", "locked_frame_mismatch": "candidate_frame_mismatch"][condition] {
                    let candidates = try XCTUnwrap(rejection["candidate_evaluations"] as? [[String: Any]])
                    XCTAssertEqual(candidates.first?["rejection_reason"] as? String, candidateReason)
                }
            } else {
                XCTAssertGreaterThan(fixture.stops, stopsBefore, "Processed invalid source stops before a late callback: \(condition)")
                let association = fixture.records.last { $0["event"] as? String == "follow_person.association"
                    && $0["frame_id"] as? String == "1:5" }
                if let reason = ["confidence": "confidence_below_minimum", "world_distance": "world_distance_exceeded",
                    "screen_association": "screen_association_rejected", "locked_frame_mismatch": "candidate_frame_mismatch"][condition] {
                    let candidates = try XCTUnwrap(association?["candidate_evaluations"] as? [[String: Any]])
                    XCTAssertEqual(candidates.first?["rejection_reason"] as? String, reason)
                }
                let issue: FollowPerceptionIssue? = ["trackingLimited": .trackingLimited, "trackingUnavailable": .trackingUnavailable,
                    "frame_stale_or_invalid": .staleFrame, "frame_timestamp_nonfinite": .staleFrame, "frame_from_future": .staleFrame,
                    "depthUnavailable": .depthUnavailable, "poseUnavailable": .poseUnavailable][condition]
                if let issue { XCTAssertEqual(fixture.coordinator.perceptionIssue, issue) }
            }
            if ["target_lost", "target_ambiguous", "confidence", "world_distance", "screen_association",
                "frame_generation_mismatch", "locked_frame_mismatch"].contains(condition) {
                XCTAssertEqual(fixture.coordinator.state, .reacquiring, condition)
            }
            _ = await fixture.coordinator.stop()
        }
    }

    func testNewestOfMultiplePendingFramesAloneAuthorizesAndBecomesPostSignalTrack() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.3)
        fixture.send(4, range: 1.7)
        await drain()
        // Only 2.3 is within 0.75 m of the ORIGINAL 1.7 m lock. Reassociating
        // against the adopted 2.3 m lock would wrongly make 2.6 ambiguous.
        fixture.send(5, range: 2.3, secondPersonRange: 2.6)
        fixture.clock.onNextRead = { fixture.feedback.release() }
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        let authorized = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_authorized" })
        XCTAssertEqual(authorized["frame_id"] as? String, "1:7")
        XCTAssertEqual(authorized["controller_pose_frame_id"] as? String, "1:7")
        if authorized["pending_frame_evaluation"] as? String == "evaluated" {
            XCTAssertEqual(authorized["matched_candidate_count"] as? Int, 1)
        } else {
            XCTAssertEqual(authorized["pending_frame_evaluation"] as? String, "not_pending")
            XCTAssertEqual(authorized["matched_candidate_count"] as? Int, 1,
                "Retain the original unique decision; reassociating against the updated lock would count two")
        }
        XCTAssertEqual(fixture.coordinator.state, .signalingReady, "Consuming that raw frame cannot establish a new post-stop baseline")
        await fixture.fresh(6, range: 3.0)
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement)
        fixture.send(7, range: 3.1)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement)
        XCTAssertEqual(fixture.commands, 1)
        _ = await fixture.coordinator.stop()
    }

    func testPendingFrameAndPostAckControllerPoseAtInclusiveAgeAndHeadingGatesAdmit() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        let capturedAt = fixture.clock.now
        fixture.clock.advance(to: capturedAt + 0.5, wakeSleepers: false)
        fixture.controllerSourceTimestamp = capturedAt
        fixture.yaw = 0.05
        fixture.send(3, range: 1.6, timestamp: capturedAt)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        let authorized = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_authorized" })
        XCTAssertEqual(authorized["frame_id"] as? String, "1:5")
        XCTAssertEqual(authorized["controller_source_age_s"] as? Double, 0.5)
        XCTAssertEqual(try XCTUnwrap(authorized["heading_rad"] as? Double), -0.05, accuracy: 0.000001)
        _ = await fixture.coordinator.stop()
    }

    func testStopWithHealthyPendingFrameFencesSuspendedFeedbackAndNewSession() async {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.7)
        fixture.coordinator.inhibitMotion()
        let stopping = Task { await fixture.coordinator.stop() }
        await drain()
        fixture.feedback.release()
        let stopped = await stopping.value
        XCTAssertTrue(stopped)
        XCTAssertEqual(fixture.commands, 0)
        fixture.stationaryPauseSeconds = 0
        let restarted = await fixture.coordinator.start()
        XCTAssertTrue(restarted)
        await drain()
        fixture.send(10, range: 1.3)
        await drain()
        fixture.send(11, range: 1.3)
        await drain()
        await fixture.fresh(12, range: 1.3)
        await fixture.fresh(13, range: 1.3)
        await fixture.fresh(14, range: 1.3)
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .waitingForClearance)
        _ = await fixture.coordinator.stop()
    }

    func testHealthyFramesArrivingBeforeFeedbackResumesCannotStarveReadyAdmission() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        fixture.stationaryPauseSeconds = 5
        let started = await fixture.coordinator.start()
        XCTAssertTrue(started)
        await drain()
        fixture.send(1, range: 1.68)
        await drain()
        fixture.clock.advance(to: 4.999)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .pausing)
        XCTAssertEqual(fixture.plans, 0)
        XCTAssertEqual(fixture.commands, 0)
        fixture.clock.advance(to: 5)
        fixture.send(2, range: 1.68)
        await drain()
        XCTAssertNotEqual(fixture.coordinator.state, .pausing)
        await fixture.fresh(3, range: 1.68)
        let frame3Time = fixture.storedControllerSample.sourceTimestamp
        await fixture.fresh(4, range: 1.68)
        XCTAssertEqual(fixture.storedControllerSample.frameID, .init(generation: 1, sequence: 4))
        XCTAssertEqual(try XCTUnwrap(fixture.storedControllerSample.sourceTimestamp) - XCTUnwrap(frame3Time),
                       0.301, accuracy: 0.000001, "Fresh means an actual post-settle capture")
        let frame4Time = fixture.storedControllerSample.sourceTimestamp

        await fixture.fresh(5, range: 1.68)
        XCTAssertEqual(fixture.storedControllerSample.frameID, .init(generation: 1, sequence: 5))
        XCTAssertEqual(try XCTUnwrap(fixture.storedControllerSample.sourceTimestamp) - XCTUnwrap(frame4Time),
                       0.301, accuracy: 0.000001)
        XCTAssertEqual(fixture.storedControllerSample.sourceTimestamp, fixture.clock.now,
                       "Pose reads must not manufacture a newer source timestamp")
        await fixture.feedback.waitUntilEntered()
        XCTAssertEqual(fixture.plans, 1)
        XCTAssertEqual(fixture.commands, 0)
        fixture.clock.advance(to: fixture.clock.now + 0.1, wakeSleepers: false)
        fixture.yaw = 0.034
        await fixture.sendAtIngressAndReleaseFeedback(6, range: 1.68)
        await drain()

        XCTAssertNil(fixture.coordinator.perceptionIssue)
        XCTAssertTrue(fixture.coordinator.isActive)
        XCTAssertEqual(fixture.turns, 0)
        XCTAssertEqual(fixture.commands, 1,
                       "Fresh matched frames during acknowledgement must not starve the one ready signal")
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.admission_authorized" }.count, 1)
        let authorized = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_authorized" })
        XCTAssertEqual(authorized["frame_id"] as? String, "1:6")
        XCTAssertEqual(authorized["controller_pose_frame_id"] as? String, "1:6")
        XCTAssertEqual(authorized["controller_source_age_s"] as? Double, 0)
        XCTAssertEqual(try XCTUnwrap(authorized["heading_rad"] as? Double), -0.034, accuracy: 0.000001)
        XCTAssertEqual(try XCTUnwrap(authorized["range_m"] as? Double), 1.68, accuracy: 0.000001)
        XCTAssertEqual(fixture.authorizedCommands, [0], "Authorization must precede the one consumed move")
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.completed" }.count, 1)
        XCTAssertEqual(fixture.coordinator.state, .signalingReady, "The admitted frame cannot establish the post-stop baseline")
        await fixture.fresh(7, range: 1.68)
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement)
        XCTAssertEqual(fixture.commands, 1)
        _ = await fixture.coordinator.stop()
    }

    func testControllerOnlyYawCorrectionDefersHeadingWithoutConsumingAttempt() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.yaw = 0.06
        fixture.captureControllerFrame(sequence: 100, timestamp: fixture.clock.now)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .aligning)
        // Alignment remains stop-bracketed and requires a new matched frame.
        let deferred = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_deferred" })
        XCTAssertEqual(deferred["reason"] as? String, "heading")
        XCTAssertEqual(try XCTUnwrap(deferred["heading_rad"] as? Double), -0.06, accuracy: 0.000001)
        XCTAssertEqual(deferred["ready_signal_attempted"] as? Bool, false)
        XCTAssertEqual(deferred["ready_admission_pending"] as? Bool, false)
        _ = await fixture.coordinator.stop()
    }
    func testPreSendApparentCompletionFailsCoordinatorAndClearsUnconsumedToken() async {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.position = Vec2(0.10, 0)
        fixture.captureControllerFrame(sequence: 100, timestamp: fixture.clock.now)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .failed("Ready signal stopped: pose changed before the first motor command. Stop confirmed. Restart following to try again."))
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        XCTAssertEqual(fixture.states.last?["ready_admission_pending"], "false")
        let terminalStates = fixture.states.filter { $0["state"]?.hasPrefix("failed") == true }
        XCTAssertFalse(terminalStates.isEmpty)
        XCTAssertTrue(terminalStates.allSatisfy { $0["ready_admission_pending"] == "false" },
            "The first terminal failure must revoke admission even when its safety stream beats the movement result")
        fixture.send(3, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.plans, 1)
        XCTAssertEqual(fixture.commands, 0)
        _ = await fixture.coordinator.stop()
    }
    func testLifecycleInhibitionClearsAdmissionBeforeSuspendedReadyResultReturns() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()

        fixture.coordinator.inhibitMotion()

        XCTAssertEqual(fixture.coordinator.state, .stopped)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        XCTAssertEqual(fixture.states.last?["ready_admission_pending"], "false",
            "Lifecycle fencing must revoke admission before the suspended movement returns")
        fixture.feedback.release()
        _ = await fixture.coordinator.stop()
        XCTAssertEqual(fixture.commands, 0)
    }

    func testDifferentControllerARGenerationCannotAuthorizeLockedWorldPosition() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.controllerGeneration = 2
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        let deferred = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_deferred" })
        XCTAssertEqual(deferred["reason"] as? String, "observation")
        XCTAssertEqual(deferred["rejection_condition"] as? String, "controller_source_generation_changed")
        XCTAssertEqual(deferred["ready_signal_attempted"] as? Bool, false)
        XCTAssertEqual(deferred["ready_admission_pending"] as? Bool, false)
        _ = await fixture.coordinator.stop()
    }
    func testControllerOnlyForwardCorrectionDefersClearanceWithoutConsumingAttempt() async throws {
        let fixture = Fixture()
        fixture.enriched = true
        await fixture.acquire()
        fixture.send(2, range: 1.4)
        await fixture.feedback.waitUntilEntered()
        fixture.position = Vec2(0.05, 0)
        fixture.captureControllerFrame(sequence: 100, timestamp: fixture.clock.now)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .waitingForClearance)
        let deferred = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_deferred" })
        XCTAssertEqual(deferred["reason"] as? String, "clearance")
        XCTAssertEqual(try XCTUnwrap(deferred["range_m"] as? Double), 1.35, accuracy: 0.000001)
        XCTAssertEqual(deferred["ready_signal_attempted"] as? Bool, false)
        XCTAssertEqual(deferred["ready_admission_pending"] as? Bool, false)
        XCTAssertEqual(deferred["controller_perception_pairing"] as? String, "independently_sampled")
        _ = await fixture.coordinator.stop()
    }
    func testRepeatedHealthyWaitUsesOneCombinedSummaryBudgetAndRetainsTrackerCounts() async throws {
        let fixture = Fixture()
        fixture.synthetic = true
        await fixture.acquire()
        fixture.send(2, range: 1.3)
        await drain()
        for sequence: UInt64 in 3...14 {
            fixture.clock.advance(to: fixture.clock.now + 0.1, wakeSleepers: false)
            fixture.send(sequence, range: 1.3)
            await drain()
        }
        let periodic = fixture.records.filter { $0["event"] as? String == "follow_person.association"
            && $0["phase"] as? String == "waitingForClearance" }
        XCTAssertEqual(periodic.count, 1)
        let record = try XCTUnwrap(periodic.first)
        XCTAssertEqual(record["raw_person_count"] as? Int, 1)
        XCTAssertEqual(record["projection_accepted_count"] as? Int, 1)
        XCTAssertEqual(record["eligible_candidate_count"] as? Int, 1)
        XCTAssertEqual(record["matched_candidate_count"] as? Int, 1)
        XCTAssertEqual(record["selected_candidate_count"] as? Int, 1)
        XCTAssertEqual(record["ready_signal_attempted"] as? Bool, false)
        XCTAssertEqual(record["stop_state"] as? String, "confirmed")
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.clearance_entered" }.count, 1)
        XCTAssertFalse(fixture.records.contains { $0["event"] as? String == "follow_frame" })
        _ = await fixture.coordinator.stop()
    }
    func testAdmissionUsesExactPostFeedbackControllerSampleAndLabelsFramePairing() async throws {
        for independent in [false, true] {
            let fixture = Fixture()
            fixture.synthetic = true
            fixture.enriched = true
            await fixture.acquire()
            fixture.independentControllerFrame = independent
            fixture.send(2, range: 1.6)
            await fixture.feedback.waitUntilEntered()
            fixture.send(3, range: 1.7)
            await drain()
            fixture.feedback.release()
            await drain()
            let event = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_authorized" })
            XCTAssertEqual(event["frame_id"] as? String, "1:5")
            XCTAssertEqual(event["controller_pose_frame_id"] as? String, independent ? "1:100" : "1:5")
            XCTAssertEqual(event["controller_perception_pairing"] as? String, independent ? "independently_sampled" : "same_frame")
            XCTAssertEqual(event["controller_source_age_s"] as? Double, 0)
            XCTAssertEqual(event["controller_source_clock"] as? String, "system_uptime")
            XCTAssertEqual(event["controller_pose_yaw_rad"] as? Double, 0)
            _ = await fixture.coordinator.stop()
        }
    }
    func testSyntheticPipelineCloseWaitStepBackOneSignalNewBaselineAndDeparture() async throws {
        let fixture = Fixture()
        fixture.synthetic = true
        fixture.suspendDeparture = true
        await fixture.acquire()
        fixture.send(2, range: 1.3)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .waitingForClearance)
        XCTAssertEqual(fixture.commands, 0)
        let entry = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.clearance_entered" })
        XCTAssertEqual(entry["reason"] as? String, "too_close_before_send")
        XCTAssertEqual(entry["stop_state"] as? String, "confirmed")
        XCTAssertEqual(entry["ready_signal_attempted"] as? Bool, false)
        fixture.send(3, range: 1.35)
        await drain()
        fixture.send(4, range: 1.5)
        await fixture.feedback.waitUntilEntered()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        XCTAssertEqual(fixture.coordinator.state, .signalingReady)
        await fixture.fresh(5, range: 1.5)
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement)
        fixture.send(6, range: 1.7)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement, "Post-signal baseline stays fixed")
        fixture.send(7, range: 1.9)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .following)
        XCTAssertGreaterThanOrEqual(fixture.plans, 2)
        let associations = fixture.records.filter { $0["event"] as? String == "follow_person.association" }
        XCTAssertTrue(associations.contains { $0["raw_person_count"] as? Int == 1 && $0["projection_accepted_count"] as? Int == 1 })
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.admission_authorized" }.count, 1)
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.clearance_exited" }.count, 1)
        fixture.sendFeedback.release()
        _ = await fixture.coordinator.stop()
    }

    func testSyntheticPipelineSuspendedFeedbackDeferralThenLocalCancellationIsTruthful() async throws {
        let fixture = Fixture()
        fixture.synthetic = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.3)
        await drain()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .waitingForClearance)
        _ = await fixture.coordinator.stop()
        let cancellation = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.cancelled" })
        XCTAssertEqual(cancellation["reason"] as? String, "local_stop")
        XCTAssertEqual(cancellation["ready_signal_attempted"] as? Bool, false)
        XCTAssertEqual(cancellation["ready_admission_pending"] as? Bool, false)
    }
    func testAdmissionDiagnosticsCapturePendingDeferredAndAuthorizedAtActualSendBoundary() async throws {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.3)
        await drain()
        fixture.feedback.release()
        await drain()
        let pending = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_pending" })
        XCTAssertEqual(pending["ready_admission_pending"] as? Bool, true)
        XCTAssertEqual(pending["ready_signal_attempted"] as? Bool, false)
        let deferred = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_deferred" })
        XCTAssertEqual(deferred["reason"] as? String, "clearance")
        XCTAssertEqual(deferred["frame_id"] as? String, "1:5")
        XCTAssertEqual(deferred["range_m"] as? Double, 1.3)
        XCTAssertEqual(deferred["heading_rad"] as? Double, 0)
        XCTAssertEqual(deferred["ready_signal_clearance_m"] as? Double, 1.37)
        XCTAssertEqual(deferred["stop_state"] as? String, "confirmed")
        XCTAssertEqual(deferred["admission_boundary"] as? String, "controller_first_send")
        XCTAssertEqual(deferred["first_wheel_telemetry_availability"] as? String, "unknown")
        XCTAssertEqual(fixture.commands, 0)
        fixture.send(4, range: 1.37)
        await drain()
        let authorized = try XCTUnwrap(fixture.records.first { $0["event"] as? String == "follow_ready.admission_authorized" })
        XCTAssertEqual(authorized["ready_admission_pending"] as? Bool, false)
        XCTAssertEqual(authorized["ready_signal_attempted"] as? Bool, true)
        XCTAssertEqual(authorized["frame_id"] as? String, "1:6")
        XCTAssertEqual(authorized["admission_boundary"] as? String, "controller_first_send")
        XCTAssertEqual(authorized["first_wheel_telemetry_availability"] as? String, "send_initiation_only")
        XCTAssertEqual(fixture.authorizedCommands, [0], "Authorization records precede actual first send, never controller entry")
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.completed" }.count, 1)
        _ = await fixture.coordinator.stop()
    }
    func testLatestClosePersonDuringRealReadyFeedbackDefersWithoutSending() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.3)
        await drain()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(String(describing: fixture.coordinator.state), "waitingForClearance")
        XCTAssertEqual(fixture.turns, 0, "Adequate heading must remain stationary under the clearance-wait label")
        XCTAssertTrue(fixture.coordinator.isActive)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        XCTAssertEqual(fixture.states.last?["ready_admission_pending"], "false")
        _ = await fixture.coordinator.stop()
    }

    private func drain() async { for _ in 0..<100 { await Task.yield() } }

    func testZeroMeasuredReadyMotionConsumesAttemptAndFailsAtExistingWatchdog() async {
        let fixture = Fixture()
        fixture.stall = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.feedback.release()
        let deadline = Date().addingTimeInterval(2)
        while fixture.coordinator.isActive, Date() < deadline { await Task.yield() }
        await drain()
        XCTAssertGreaterThan(fixture.commands, 0)
        XCTAssertLessThan(fixture.commands, 55)
        XCTAssertEqual(fixture.coordinator.state, .failed("Ready signal stopped: insufficient forward progress within 2.5 seconds. Stop confirmed. Restart following to try again."))
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "true")
        let commands = fixture.commands
        fixture.stall = false
        fixture.send(3, range: 1.7)
        await drain()
        XCTAssertEqual(fixture.commands, commands)
        _ = await fixture.coordinator.stop()
    }

    func testStalePerceptionAfterRealSendCannotRetryWhenHealthyFramesRecover() async {
        let fixture = Fixture()
        fixture.suspendSend = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.feedback.release()
        await fixture.sendFeedback.waitUntilEntered()
        fixture.clock.advance(to: fixture.clock.now + 0.501)
        await drain()
        fixture.suspendSend = false
        fixture.sendFeedback.release()
        await drain()
        await fixture.fresh(3, range: 1.6)
        await fixture.fresh(4, range: 1.6)
        await fixture.fresh(5, range: 1.6)
        await fixture.fresh(6, range: 1.6)
        XCTAssertEqual(fixture.commands, 1)
        XCTAssertEqual(fixture.plans, 1)
        XCTAssertEqual(fixture.coordinator.state, .failed("Ready signal interrupted. Stop and start following again."))
        _ = await fixture.coordinator.stop()
    }

    func testLossAfterRealSendCannotRepeatReadyAfterAlignedReacquisition() async {
        let fixture = Fixture()
        fixture.enriched = true
        fixture.suspendSend = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.feedback.release()
        await fixture.sendFeedback.waitUntilEntered()
        fixture.send(3, range: 1.6, peopleAvailable: false)
        await drain()
        fixture.suspendSend = false
        fixture.sendFeedback.release()
        await drain()
        fixture.send(4, range: 1.6)
        await drain()
        fixture.send(5, range: 1.6)
        await drain()
        await fixture.fresh(6, range: 1.6)
        await fixture.fresh(7, range: 1.6)
        await fixture.fresh(8, range: 1.6)
        XCTAssertEqual(fixture.commands, 1)
        XCTAssertEqual(fixture.plans, 1)
        XCTAssertEqual(fixture.coordinator.state, .failed("Ready signal interrupted. Stop and start following again."))
        _ = await fixture.coordinator.stop()
    }

    func testApproachAfterRealSendInitiationStopsAndNeverReturnsToResumableWait() async {
        let fixture = Fixture()
        fixture.suspendSend = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.feedback.release()
        await fixture.sendFeedback.waitUntilEntered()
        XCTAssertEqual(fixture.states.last?["ready_admission_pending"], "false",
                       "Authorization consumes the reservation; an initiated signal is no longer pending admission")
        fixture.send(3, range: 1.3)
        await drain()
        XCTAssertEqual(fixture.coordinator.state,
                       .failed("Person too close during ready signal. Step back and start following again."))
        fixture.sendFeedback.release()
        await drain()
        fixture.send(4, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        XCTAssertFalse(fixture.coordinator.isActive)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "true")
        let cancellation = fixture.records.first { $0["event"] as? String == "follow_ready.cancelled" }
        XCTAssertEqual(cancellation?["reason"] as? String, "person_approached_during_signal")
        XCTAssertEqual(cancellation?["ready_signal_attempted"] as? Bool, true)
        let stopped = fixture.records.first { $0["event"] as? String == "follow_ready.stop_confirmed" }
        XCTAssertEqual(stopped?["stop_state"] as? String, "confirmed")
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.stop_confirmed" }.count, 1)
        _ = await fixture.coordinator.stop()
    }

    func testUnhealthyLatestFrameDuringRealPreflightKeepsTwoSecondOutageDeadline() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.6, tracking: .limited)
        await drain()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.perceptionIssue, .trackingLimited)
        fixture.clock.advance(to: fixture.clock.now + 2)
        await drain()
        XCTAssertFalse(fixture.coordinator.isActive)
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        _ = await fixture.coordinator.stop()
    }

    func testHeadingDriftDuringRealPreflightAlignsBeforeNewPostStopFrameCanAdmit() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.yaw = 0.06
        fixture.send(3, range: 1.6)
        await drain()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .aligning)
        XCTAssertEqual(fixture.turns, 0, "Heading deferral cannot turn on the pre-stop candidate")
        await fixture.fresh(4, range: 1.6)
        await fixture.fresh(5, range: 1.6)
        XCTAssertEqual(fixture.turns, 1)
        fixture.yaw = 0
        await fixture.fresh(6, range: 1.6)
        await fixture.fresh(7, range: 1.6)
        await fixture.fresh(8, range: 1.6)
        XCTAssertEqual(fixture.commands, 1)
        _ = await fixture.coordinator.stop()
    }

    func testExactlyFiveHundredMillisecondLatestObservationStillAdmitsOneSend() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.clock.advance(to: fixture.clock.now + 0.5, wakeSleepers: false)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        XCTAssertEqual(fixture.records.filter { $0["event"] as? String == "follow_ready.admission_authorized" }.count, 1,
            "Admission at 500 ms is valid; elapsed pulse time may subsequently age that detector frame")
        _ = await fixture.coordinator.stop()
    }

    func testLossDuringPendingRealAdmissionCannotSendReadyAndKeepsReacquisitionDeadline() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.6, peopleAvailable: false)
        await drain()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .reacquiring)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        fixture.clock.advance(to: fixture.clock.now + 10)
        await drain()
        XCTAssertEqual(fixture.coordinator.state, .failed("Person lost."))
        XCTAssertEqual(fixture.commands, 0)
        _ = await fixture.coordinator.stop()
    }

    func testFailedFinalStopAfterDeferralOutranksWaitAndBlocksNewAdmission() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.3)
        await drain()
        fixture.failStop = true
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .failed("Motor stop could not be confirmed. Motion is blocked."))
        let started = await fixture.coordinator.start()
        XCTAssertFalse(started)
        fixture.send(4, range: 1.6)
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        fixture.failStop = false
        _ = await fixture.coordinator.stop()
    }

    func testObstacleDuringReadyFeedbackWinsOverCloseClearanceDeferral() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.3)
        await drain()
        fixture.clearance = 0.44
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .failed("Obstacle detected. Motion stopped."))
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        _ = await fixture.coordinator.stop()
    }

    func testUncertainFirstSendIrrevocablyConsumesAttemptAndCannotRetryOnClearanceRecovery() async {
        let fixture = Fixture()
        fixture.failSend = true
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        XCTAssertFalse(fixture.coordinator.isActive)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "true")
        fixture.failSend = false
        fixture.send(3, range: 1.7)
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        XCTAssertNotEqual(fixture.coordinator.state, .waitingForClearance)
        _ = await fixture.coordinator.stop()
    }

    func testExpiredLatestObservationAtAdmissionEntersExistingOutagePolicyWithoutSend() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.clock.advance(to: fixture.clock.now + 0.501, wakeSleepers: false)
        fixture.captureControllerFrame(sequence: 3, timestamp: fixture.clock.now)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.perceptionIssue, .staleFrame,
                       "A delayed watchdog must not let admission hide the actual stale-frame outage")
        _ = await fixture.coordinator.stop()
    }

    func testStopAndNewGenerationFenceOldSuspendedAdmission() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.coordinator.inhibitMotion()
        let stopping = Task { await fixture.coordinator.stop() }
        await drain()
        fixture.feedback.release()
        _ = await stopping.value
        _ = await fixture.coordinator.start()
        await drain()
        fixture.send(10, range: 1.3)
        await drain()
        fixture.send(11, range: 1.3)
        await drain()
        await fixture.fresh(12, range: 1.3)
        await fixture.fresh(13, range: 1.3)
        await fixture.fresh(14, range: 1.3)
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        XCTAssertEqual(fixture.coordinator.state, .waitingForClearance)
        XCTAssertEqual(fixture.states.last?["ready_signal_attempted"], "false")
        _ = await fixture.coordinator.stop()
    }

    func testTwoEligibleFramesWhileFeedbackSuspendsOwnOnlyOnePendingRequest() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.65)
        fixture.send(4, range: 1.7)
        await drain()
        XCTAssertEqual(fixture.plans, 1)
        fixture.feedback.release()
        await drain()
        XCTAssertEqual(fixture.commands, 1)
        _ = await fixture.coordinator.stop()
    }

    func testDeferredAttemptCanStepBackGraduallyAndSignalExactlyAtInclusiveGate() async {
        let fixture = Fixture()
        await fixture.acquire()
        fixture.send(2, range: 1.6)
        await fixture.feedback.waitUntilEntered()
        fixture.send(3, range: 1.3)
        await drain()
        fixture.feedback.release()
        await drain()
        fixture.send(4, range: 1.36)
        await drain()
        XCTAssertEqual(fixture.commands, 0)
        fixture.send(5, range: 1.37)
        await drain()
        XCTAssertEqual(fixture.commands, 1, "Deferral cannot consume the one attempt; the exact gate is inclusive")
        XCTAssertEqual(fixture.coordinator.state, .signalingReady, "A final stop alone cannot establish a baseline")
        await fixture.fresh(6, range: 1.37)
        XCTAssertEqual(fixture.coordinator.state, .waitingForMovement)
        XCTAssertEqual(fixture.commands, 1)
        _ = await fixture.coordinator.stop()
    }

    @MainActor
    private final class SuspendedMission: OperatorMission {
        let stopFeedback = FollowDiagnosticSuspension()
        func handle(_ text: String) async { XCTFail("Local follow must not reach mission handling") }
        func cancelCurrentMissionAndWait() async throws { await stopFeedback.suspend() }
    }

    @MainActor
    private final class Fixture {
        let perception = FollowPerceptionFake()
        let clock = ManualFollowClock()
        var feedback = AdmissionSuspension()
        let sendFeedback = AdmissionSuspension()
        var suspendSend = false
        var holdAcknowledgement = false
        var holdReadyAcknowledgement = false
        let readyFeedback = AdmissionSuspension()
        let scanFeedback = AdmissionSuspension()
        let stopFeedback = AdmissionSuspension()
        var holdStopNumber: Int?
        var sourceContinuation: AsyncStream<NavigationPoseSample>.Continuation?
        var wheelCommands: [WheelCommand] = []
        var stops = 0
        var stopSendCounts: [Int] = []
        var synthetic = false
        var enriched = true
        var independentControllerFrame = false
        var controllerGeneration: UInt64 = 1
        var sourceSequence: UInt64 = 0
        var frameOffset: UInt64 = 0
        var frameIDs: [UInt64: UInt64] = [:]
        var storedControllerSample = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
            frameID: .init(generation: 1, sequence: 0), sourceTimestamp: 0, trackingQuality: .normal, source: "synthetic_ar")
        var controllerSourceTimestamp: Double?
        var ingestedControllerTimestamp = 0.0
        var suspendDeparture = false
        var commands = 0
        var plans = 0
        var failSend = false
        var failStop = false
        var clearance = 2.0
        var yaw = 0.0
        var turns = 0
        var turning = false
        var stall = false
        var controllerTime = Date(timeIntervalSince1970: 100)
        var states: [[String: String]] = []
        var records: [[String: Any]] = []
        var authorizedCommands: [Int] = []
        var ready = false
        var position = Vec2.zero
        var stationaryPauseSeconds = 0.0
        lazy var controller = NavigationController(currentPose: { [self] in Pose2D(position: position, yaw: yaw) },
            forwardClearance: { [self] in clearance }, plan: { [self] _, goal in ready = true; plans += 1; return [goal] },
            lastAckAt: { [self] in await acknowledgement() }, sendCommand: { [self] command in try await sendCommand(command) },
            stopRover: { [self] in try await stopRover() },
            sleep: { [self] duration in await tick(duration) }, now: { [self] in controllerTime },
            poseSample: { [self] in
                return enriched ? storedControllerSample : .legacy(storedControllerSample.pose)
            }, sourceNow: { [self] in clock.now },
            sourceEvents: { [self] in AsyncStream { sourceContinuation = $0 } },
            sourceStopSnapshot: { [self] in storedControllerSample })
        lazy var coordinator: FollowMeCoordinator = {
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = stationaryPauseSeconds
            return FollowMeCoordinator(perception: perception, motion: NavigationFollowMeMotion(navigation: controller),
                clock: clock, configuration: config, eventSink: { [self] in record($0, fields: $1) })
        }()
        func record(_ event: String, fields: [String: String]) {
            if event == "follow_state" { states.append(fields) }
            if let json = fields["payload"], let data = json.data(using: .utf8),
               let record = try? JSONSerialization.jsonObject(with: data) as? [String: Any] { records.append(record) }
            if event == "follow_ready.admission_authorized" { authorizedCommands.append(commands) }
        }
        func send(_ sequence: UInt64, range: Double, peopleAvailable: Bool = true, tracking: ARTrackingQuality? = .normal,
                  timestamp: Double? = nil, depthAvailable: Bool = true, poseAvailable: Bool = true,
                  confidence: Float = 0.99, box: CGRect = CGRect(x: 0.3, y: 0.2, width: 0.4, height: 0.6),
                  secondPersonRange: Double? = nil, generation: UInt64 = 1, personFrameID: ARFrameID? = nil) {
            let sourceID = frameIDs[sequence] ?? max(sequence + frameOffset, sourceSequence + 1)
            frameIDs[sequence] = sourceID
            let id = ARFrameID(generation: generation, sequence: sourceID)
            // Camera ingress is independent of deliberately malformed detector timestamps below.
            // These admission tests independently supply normal controller AR and possibly invalid detector batches.
            if sourceID > sourceSequence {
                sourceSequence = sourceID
                ingestedControllerTimestamp = clock.now
                storeControllerCapture()
            }
            if synthetic {
                var image: CVPixelBuffer?
                var depth: CVPixelBuffer?
                CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
                CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_DepthFloat32, nil, &depth)
                let map = depth!
                CVPixelBufferLockBaseAddress(map, [])
                let base = CVPixelBufferGetBaseAddress(map)!.assumingMemoryBound(to: Float.self)
                for row in 0..<20 { for col in 0..<20 { base[row * CVPixelBufferGetBytesPerRow(map) / 4 + col] = Float(range - position.x) } }
                CVPixelBufferUnlockBaseAddress(map, [])
                let snapshot = ARFrameSnapshot(id: id, timestamp: clock.now, image: image!,
                    cameraTransform: simd_float4x4(columns: (SIMD4<Float>(0, 0, 1, 0), SIMD4<Float>(0, 1, 0, 0),
                        SIMD4<Float>(-1, 0, 0, 0), SIMD4<Float>(Float(position.x), 0, Float(position.y), 1))),
                    cameraIntrinsics: simd_float3x3(columns: (SIMD3<Float>(10, 0, 0), SIMD3<Float>(0, 10, 0), SIMD3<Float>(10, 10, 1))),
                    imageResolution: CGSize(width: 20, height: 20), depthMap: map,
                    pose: Pose2D(position: position, yaw: yaw), trackingQuality: tracking ?? .unavailable)
                perception.send(.frame(ARFollowMePerceptionSource.batch(from: snapshot, detections: peopleAvailable ? [
                    .init(label: "person", confidence: 0.99, boundingBox: CGRect(x: 0.4, y: 0.5, width: 0.2, height: 0.3))] : [])))
                return
            }
            let person = FollowPersonObservation(frameID: personFrameID ?? id, timestamp: timestamp ?? clock.now,
                confidence: confidence, boundingBox: box,
                position: Vec2(range, 0), pose: Pose2D(position: position, yaw: yaw))
            var people = peopleAvailable ? [person] : []
            if let secondPersonRange {
                people.append(.init(frameID: id, timestamp: timestamp ?? clock.now, confidence: confidence, boundingBox: box,
                    position: Vec2(secondPersonRange, 0), pose: Pose2D(position: position, yaw: yaw)))
            }
            perception.send(.frame(.init(frameID: id, timestamp: timestamp ?? clock.now,
                pose: poseAvailable ? Pose2D(position: position, yaw: yaw) : nil, depthAvailable: depthAvailable,
                people: people, trackingQuality: tracking)))
        }
        func acquire(aligned: Bool = true) async {
            _ = await coordinator.start()
            for _ in 0..<30 { await Task.yield() }
            send(1, range: 1.6)
            for _ in 0..<100 { await Task.yield() }
            if aligned {
                // Two actual matched/source captures clear independent and controller stops.
                await fresh(2, range: 1.6)
                await fresh(3, range: 1.6)
                XCTAssertEqual(stops, 3, "Reach final alignment ACK before testing ready preflight")
                XCTAssertEqual(plans, 0, "The controller's arrival source cannot launch ready")
                // Logical test frame 2 starts at real AR frame 4; IDs are fixed for this stream.
                frameOffset = 2
                frameIDs.removeAll()
                clock.advance(to: clock.now + 0.301)
            }
        }
        func fresh(_ sequence: UInt64, range: Double, peopleAvailable: Bool = true) async {
            clock.advance(to: clock.now + 0.301)
            send(sequence, range: range, peopleAvailable: peopleAvailable)
            for _ in 0..<100 { await Task.yield() }
        }
        func sendAtIngressAndReleaseFeedback(_ sequence: UInt64, range: Double,
                                              file: StaticString = #filePath, line: UInt = #line) async {
            // Finish all producer-side clock reads before arming the one-shot probe.
            // With the controller held in acknowledgement(), the next clock read
            // is receive(event:)'s ingress read, before it schedules the processor.
            // Resuming the ACK here cannot run authorization until this synchronous
            // MainActor ingress turn has published pendingFrame.
            send(sequence, range: range)
            let ingested = XCTestExpectation(description: "Perception ingress before ACK release")
            clock.onNextRead = { [self] in
                feedback.release()
                ingested.fulfill()
            }
            let result = await XCTWaiter.fulfillment(of: [ingested], timeout: 1)
            XCTAssertEqual(result, .completed, file: file, line: line)
            XCTAssertNil(clock.onNextRead, "Ingress probe was not consumed", file: file, line: line)
            // Also release on fixture failure so a held controller cannot leak.
            clock.onNextRead = nil
            feedback.release()
        }
        func recoverReady(range: Double) async {
            send(3, range: 1.6)
            for _ in 0..<100 { await Task.yield() }
            await fresh(4, range: 1.6)
            await fresh(5, range: 1.6)
            await fresh(6, range: range)
            await feedback.waitUntilEntered()
        }
        func storeControllerCapture(tracking: ARTrackingQuality? = .normal) {
            storedControllerSample = .init(pose: .init(position: position, yaw: yaw),
                frameID: .init(generation: controllerGeneration, sequence: independentControllerFrame ? 100 : sourceSequence),
                sourceTimestamp: controllerSourceTimestamp ?? ingestedControllerTimestamp,
                trackingQuality: tracking, source: "synthetic_ar")
            sourceContinuation?.yield(storedControllerSample)
        }
        func captureControllerFrame(sequence: UInt64, timestamp: Double) {
            sourceSequence = sequence + frameOffset
            ingestedControllerTimestamp = timestamp
            storeControllerCapture()
        }
        func acknowledgement() async -> Date? {
            if holdReadyAcknowledgement && commands > 0 && !readyFeedback.entered { await readyFeedback.suspend() }
            if holdAcknowledgement && !scanFeedback.entered { await scanFeedback.suspend() }
            if ready && !feedback.entered { await feedback.suspend() }
            return controllerTime
        }
        func sendCommand(_ command: WheelCommand) async throws {
            if command.left != 0 || command.right != 0 { wheelCommands.append(command) }
            turning = command.left != command.right
            if turning { turns += 1 }
            if command.left == command.right, command.left != 0 { commands += 1 }
            if suspendSend || (suspendDeparture && plans > 1) { await sendFeedback.suspend() }
            if failSend { throw URLError(.networkConnectionLost) }
        }
        func stopRover() async throws {
            stops += 1
            stopSendCounts.append(wheelCommands.count)
            if stops == holdStopNumber { await stopFeedback.suspend() }
            if failStop { throw URLError(.cannotConnectToHost) }
        }
        func tick(_ duration: Duration) async {
            if turning { yaw = 0 } else {
                if FollowMotionTaskScope.evidence?.context.purpose == .followReady {
                    let elapsed = max(0.001, Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18)
                    controllerTime = controllerTime.addingTimeInterval(elapsed)
                    clock.advance(to: clock.now + elapsed, wakeSleepers: !stall)
                    if commands > 0 {
                        if !stall { position = Vec2(0.1, 0) }
                        sourceSequence += 1
                        ingestedControllerTimestamp = clock.now
                        storeControllerCapture()
                        if stall {
                            // A stalled chassis still produces fresh camera and
                            // detector frames. Only measured translation is zero.
                            let id = ARFrameID(generation: controllerGeneration, sequence: sourceSequence)
                            let person = FollowPersonObservation(frameID: id, timestamp: clock.now, confidence: 0.99,
                                boundingBox: CGRect(x: 0.3, y: 0.2, width: 0.4, height: 0.6), position: Vec2(1.6, 0),
                                pose: .init(position: position, yaw: yaw))
                            perception.send(.frame(.init(frameID: id, timestamp: clock.now,
                                pose: person.pose, depthAvailable: true, people: [person], trackingQuality: .normal)))
                            for _ in 0..<10 { await Task.yield() }
                            clock.advance(to: clock.now)
                        }
                    }
                }
            }
            await Task.yield()
        }
    }

    @MainActor
    private final class AdmissionSuspension {
        private(set) var entered = false
        private var waiter: CheckedContinuation<Void, Never>?
        func suspend() async { await withCheckedContinuation { entered = true; waiter = $0 } }
        func waitUntilEntered(file: StaticString = #filePath, line: UInt = #line) async {
            for _ in 0..<10000 where !entered { await Task.yield() }
            XCTAssertTrue(entered, "Fixture did not reach the intended held boundary", file: file, line: line)
        }
        func release() { waiter?.resume(); waiter = nil }
    }
}
