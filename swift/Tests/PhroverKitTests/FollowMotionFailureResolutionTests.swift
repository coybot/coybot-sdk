import XCTest
@testable import PhroverKit

final class FollowMotionFailureResolutionTests: XCTestCase {
    func testReadyGuardCauseAndMeasurementsSurviveGenericDeliveryAndStopFailureWins() {
        let request = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 4, purpose: .followReady, phase: "aligning")
        let context = FollowMotionOperationContext(request: request, controllerOperationID: 7, purpose: .followReady,
            profile: nil, failureCause: .readyLateralDeviation)
        let generic = FollowMotionOperationContext(request: request, controllerOperationID: 7, purpose: .followReady, profile: nil)
        var result = FollowMotionFailureResolution(.init(context: context, reason: .stalled,
            stopOutcome: .pending, source: .stream, turnDiagnosticFields: ["ready_lateral_m": .number(0.03)]))
        result.consume(.init(context: generic, reason: .stalled, stopOutcome: .confirmed, source: .result))
        XCTAssertEqual(result.diagnosticReason, "ready_lateral_deviation")
        XCTAssertEqual(result.turnDiagnosticFields["ready_lateral_m"], .number(0.03))
        XCTAssertEqual(result.message, "Ready signal stopped: measured sideways drift exceeded 2 cm. Stop confirmed. Restart following to try again.")
        result.consume(.init(context: generic, reason: .commandFailed, stopOutcome: .failed, source: .confirmation))
        XCTAssertEqual(result.message, "Motor stop could not be confirmed. Motion is blocked.")
        XCTAssertEqual(result.diagnosticReason, "ready_lateral_deviation")
    }

    func testEvidenceAndStaleSourceCausesSurviveBothOrdersAndFailedStop() {
        for (cause, reason) in [(FollowMotionFailureCause.calibrationEvidenceIncomplete, NavigationFailure.rotationResolutionInsufficient),
                                (.poseSourceStale, .trackingLost)] {
            let base = delivery(.stream).context
            let context = FollowMotionOperationContext(request: base.request, controllerOperationID: base.controllerOperationID,
                purpose: base.purpose, profile: base.profile, failureCause: cause)
            let specific = FollowMotionFailureDelivery(context: context, reason: reason, stopOutcome: .pending, source: .stream)
            let wrapper = delivery(.result, reason: .commandFailed)
            for order in [[specific, wrapper], [wrapper, specific]] {
                var resolution = FollowMotionFailureResolution(order[0])
                resolution.consume(order[1])
                XCTAssertEqual(resolution.primaryReason, reason)
                XCTAssertEqual(resolution.diagnosticReason, cause.rawValue)
                XCTAssertFalse(resolution.message.contains("too coarse"))
                resolution.consume(delivery(.confirmation, reason: .commandFailed, stop: .failed))
                resolution.consume(delivery(.result, reason: .cancelled, stop: .confirmed, stale: true))
                XCTAssertEqual(resolution.context.failureCause, cause)
                XCTAssertEqual(resolution.message, "Motor stop could not be confirmed. Motion is blocked.")
                XCTAssertEqual(resolution.priority, 3)
            }
        }
    }

    func testSchedulingCauseSurvivesBothDeliveryOrdersUnknownWrappersAndStickyStopFailure() {
        let unknown = delivery(.result, reason: .commandFailed)
        let base = unknown.context
        let context = FollowMotionOperationContext(request: base.request, controllerOperationID: base.controllerOperationID,
            purpose: base.purpose, profile: base.profile, failureCause: .burstPreSendExpired)
        let specific = FollowMotionFailureDelivery(context: context, reason: .rotationResolutionInsufficient,
            stopOutcome: .pending, source: .stream, turnDiagnosticFields: ["sender_outcome": .string("expired")])
        for order in [[unknown, specific], [specific, unknown]] {
            var record = FollowMotionFailureResolution(order[0])
            record.consume(order[1])
            XCTAssertEqual(record.key, .init(generation: 7, operationID: 91))
            XCTAssertEqual(record.diagnosticReason, "burst_pre_send_expired")
            XCTAssertEqual(record.turnDiagnosticFields["sender_outcome"], .string("expired"))
            XCTAssertTrue(record.message.hasSuffix("Confirming motor stop…"))
            record.consume(delivery(.confirmation, reason: .commandFailed, stop: .confirmed, stale: true))
            XCTAssertEqual(record.stopOutcome, .pending)
            record.consume(delivery(.confirmation, reason: .commandFailed, stop: .confirmed))
            XCTAssertTrue(record.message.hasSuffix("Stop confirmed. Restart following to try again."))
            record.consume(delivery(.confirmation, reason: .commandFailed, stop: .failed))
            record.consume(delivery(.result, reason: .cancelled, stop: .confirmed))
            XCTAssertEqual(record.context.failureCause, .burstPreSendExpired)
            XCTAssertEqual(record.primaryReason, .rotationResolutionInsufficient)
            XCTAssertEqual(record.message, "Motor stop could not be confirmed. Motion is blocked.")
            XCTAssertEqual(record.priority, 3)
        }
    }

    func testResolutionTelemetrySurvivesGenericWrapperAndStickyFailedStopWithoutLosingControllerPhase() {
        let context = delivery(.stream, purpose: .followAlignment).context
        let facts: [String: FollowDiagnosticValue] = ["controller_phase": .string("stopped_planning"),
            "candidate_budget_s": .number(-0.019), "retained_response_rate_rad_s": .number(2.0943951023931953),
            "post_ack_travel_confidence": .string("unknown")]
        var record = FollowMotionFailureResolution(.init(context: context, reason: .rotationResolutionInsufficient,
            stopOutcome: .pending, source: .stream, turnDiagnosticFields: facts))
        XCTAssertEqual(record.turnDiagnosticFields, facts)
        XCTAssertTrue(record.message.contains("Confirming motor stop…"))
        record.consume(.init(context: context, reason: .commandFailed, stopOutcome: .confirmed, source: .result))
        XCTAssertEqual(record.turnDiagnosticFields, facts)
        XCTAssertEqual(record.diagnosticReason, "rotation_resolution_insufficient")
        XCTAssertTrue(record.message.contains("Stop confirmed."))
        record.consume(.init(context: context, reason: .commandFailed, stopOutcome: .failed, source: .confirmation))
        record.consume(.init(context: context, reason: .cancelled, stopOutcome: .confirmed, source: .result))
        XCTAssertEqual(record.turnDiagnosticFields["controller_phase"], .string("stopped_planning"))
        XCTAssertEqual(record.priority, 3)
        XCTAssertEqual(record.message, "Motor stop could not be confirmed. Motion is blocked.")
    }

    private let pending = "Search rotation stopped: insufficient measured yaw progress. Confirming motor stop…"
    private let confirmed = "Search rotation stopped: insufficient measured yaw progress. Stop confirmed. Restart following to try again."
    private let blocked = "Motor stop could not be confirmed. Motion is blocked."

    private func delivery(_ source: FollowMotionDeliverySource, reason: NavigationFailure = .stalled,
                          stop: FollowMotionStopOutcome = .pending, stale: Bool = false,
                          purpose: FollowMotionPurpose? = .followScan) -> FollowMotionFailureDelivery {
        .init(context: .init(request: .init(sessionGeneration: 7, requestToken: 2, purpose: .followScan,
              phase: "searching"), controllerOperationID: 91, purpose: purpose,
              profile: nil), reason: reason, stopOutcome: stop, source: source, stale: stale)
    }

    func testBothOrdersRetainStallThroughWrapperCancellationAndAuthoritativeStop() {
        for order: [FollowMotionDeliverySource] in [[.stream, .result], [.result, .stream]] {
            var record = FollowMotionFailureResolution(delivery(order[0]))
            XCTAssertEqual(record.message, pending)
            record.consume(delivery(order[1], reason: .commandFailed))
            record.consume(delivery(.result, reason: .cancelled))
            XCTAssertEqual(record.primaryReason, .stalled)
            XCTAssertEqual(record.diagnosticReason, "no_yaw_progress")
            XCTAssertEqual(record.message, pending)
            record.consume(delivery(.confirmation, stop: .confirmed))
            XCTAssertEqual(record.message, confirmed)
            record.consume(delivery(.confirmation, reason: .commandFailed, stop: .failed))
            record.consume(delivery(.result, stop: .confirmed, stale: true))
            XCTAssertEqual(record.primaryReason, .stalled)
            XCTAssertEqual(record.stopOutcome, .failed)
            XCTAssertEqual(record.message, blocked)
            XCTAssertEqual(record.priority, 3)
        }
    }

    func testOperationKeyIsolationUnknownFactsAndStaleAcknowledgement() {
        let initial = delivery(.stream)
        var record = FollowMotionFailureResolution(initial)
        XCTAssertEqual(record.key, .init(generation: 7, operationID: 91))
        for (generation, operation) in [(UInt64(8), UInt64(91)), (7, 92)] {
            let foreign = FollowMotionOperationContext(request: .init(sessionGeneration: generation,
                requestToken: 2, purpose: .followAlignment, phase: "aligning"),
                controllerOperationID: operation, purpose: .followAlignment, profile: nil)
            record.consume(.init(context: foreign, reason: .commandFailed, stopOutcome: .failed, source: .stream))
            XCTAssertEqual(record.stopOutcome, .pending)
            XCTAssertEqual(record.context.request?.phase, "searching")
        }
        record.consume(delivery(.result, stop: .confirmed, stale: true))
        XCTAssertEqual(record.message, pending, "Stale success is not authoritative stop acknowledgement")
        let unknown = FollowMotionFailureResolution(delivery(.result, stop: .unknown, purpose: nil))
        XCTAssertEqual(unknown.diagnosticReason, "stalled")
        XCTAssertEqual(unknown.message, "Navigation stopped: insufficient measured progress.")
        XCTAssertNil(unknown.context.purpose)
    }

    func testLegacyRequestMappingDoesNotMergeUnknownControllerOperations() {
        let request = FollowMotionRequestContext(sessionGeneration: 7, requestToken: 2,
            purpose: .followScan, phase: "searching")
        let context = FollowMotionOperationContext(request: request, controllerOperationID: nil,
            purpose: nil, profile: nil)
        var record = FollowMotionFailureResolution(.init(context: context, reason: .stalled,
            stopOutcome: .unknown, source: .result))
        let other = FollowMotionOperationContext(request: .init(sessionGeneration: 7, requestToken: 3,
            purpose: .followReady, phase: "signalingReady"), controllerOperationID: nil, purpose: nil, profile: nil)
        record.consume(.init(context: other, reason: .commandFailed, stopOutcome: .failed, source: .stream))
        XCTAssertEqual(record.stopOutcome, .unknown)
        XCTAssertNil(record.key)
        XCTAssertEqual(record.diagnosticReason, "stalled")
    }

    func testResolutionFailureRetainsCapturedPurposeReasonAndStopPriorityInBothOrders() {
        for purpose: FollowMotionPurpose? in [.followScan, .followAlignment, nil] {
            let prefix = purpose == .followScan ? "Search rotation" : (purpose == .followAlignment ? "Person alignment" : "Turn")
            for order: [FollowMotionDeliverySource] in [[.stream, .result], [.result, .stream]] {
                var record = FollowMotionFailureResolution(delivery(order[0], reason: .rotationResolutionInsufficient,
                    stop: .unknown, purpose: purpose))
                XCTAssertEqual(record.diagnosticReason, "rotation_resolution_insufficient")
                XCTAssertEqual(record.priority, 2)
                XCTAssertEqual(record.message, "\(prefix) stopped: observed response is too coarse for the remaining angle. Confirming motor stop…")
                record.consume(delivery(order[1], reason: .commandFailed, purpose: .followReady))
                record.consume(delivery(.result, reason: .cancelled))
                record.consume(delivery(.confirmation, reason: .rotationResolutionInsufficient, stop: .confirmed, stale: true))
                XCTAssertEqual(record.stopOutcome, .pending)
                XCTAssertEqual(record.context.purpose, purpose)
                record.consume(delivery(.confirmation, reason: .rotationResolutionInsufficient, stop: .confirmed))
                XCTAssertEqual(record.primaryReason, .rotationResolutionInsufficient)
                XCTAssertEqual(record.message, "\(prefix) stopped: observed response is too coarse for the remaining angle. Stop confirmed. Restart following to try again.")
                record.consume(delivery(.confirmation, reason: .commandFailed, stop: .failed))
                record.consume(delivery(.result, reason: .rotationResolutionInsufficient, stop: .confirmed))
                XCTAssertEqual(record.message, blocked)
                XCTAssertEqual(record.priority, 3)
            }
        }
    }

    func testSpecificDeliverySuppliesCapturedPurposeAfterUnknownGenericWrapper() {
        var record = FollowMotionFailureResolution(delivery(.result, reason: .commandFailed,
            stop: .unknown, purpose: nil))
        record.consume(delivery(.stream, reason: .rotationResolutionInsufficient, purpose: .followAlignment))
        XCTAssertEqual(record.context.purpose, .followAlignment)
        XCTAssertEqual(record.primaryReason, .rotationResolutionInsufficient)
        XCTAssertEqual(record.message, "Person alignment stopped: observed response is too coarse for the remaining angle. Confirming motor stop…")
        record.consume(delivery(.confirmation, reason: .commandFailed, stop: .failed, purpose: nil))
        record.consume(delivery(.result, reason: .cancelled, stop: .confirmed, purpose: nil))
        XCTAssertEqual(record.context.purpose, .followAlignment)
        XCTAssertEqual(record.message, blocked)
    }
}
