import Foundation

/// A value-only record. The coordinator owns retention and motor-stop authority.
struct FollowMotionFailureResolution {
    private(set) var turnDiagnosticFields: [String: FollowDiagnosticValue] = [:]
    struct Key: Hashable {
        let generation: UInt64
        let operationID: UInt64
    }
    private(set) var context: FollowMotionOperationContext
    private(set) var primaryReason: NavigationFailure
    private(set) var stopOutcome: FollowMotionStopOutcome = .unknown
    private(set) var deduplicated = false
    private var sources: Set<String> = []
    var deliveriesDrained: Bool {
        sources.contains(FollowMotionDeliverySource.stream.rawValue)
            && sources.contains(FollowMotionDeliverySource.result.rawValue)
            && sources.contains(FollowMotionDeliverySource.confirmation.rawValue)
    }
    var key: Key? {
        guard let generation = context.request?.sessionGeneration,
              let operationID = context.controllerOperationID else { return nil }
        return Key(generation: generation, operationID: operationID)
    }

    init(_ delivery: FollowMotionFailureDelivery) {
        context = delivery.context
        primaryReason = delivery.reason
        consume(delivery)
    }
    mutating func consume(_ delivery: FollowMotionFailureDelivery) {
        if let key {
            guard delivery.context.request?.sessionGeneration == key.generation,
                  delivery.context.controllerOperationID == key.operationID else { return }
        }
        guard delivery.context.request?.sessionGeneration == context.request?.sessionGeneration,
              delivery.context.controllerOperationID == context.controllerOperationID else { return }
        if key == nil {
            guard delivery.context.request?.requestToken == context.request?.requestToken else { return }
        }
        let previousReason = primaryReason
        let previousCause = context.failureCause
        let previousStop = stopOutcome
        let seen = !sources.isEmpty
        if primaryReason == .commandFailed || primaryReason == .cancelled {
            if delivery.reason != .cancelled {
                primaryReason = delivery.reason
                if delivery.reason != .commandFailed { context = delivery.context }
            }
        }
        if delivery.reason == primaryReason, !delivery.turnDiagnosticFields.isEmpty {
            turnDiagnosticFields.merge(delivery.turnDiagnosticFields) { _, value in value }
        }
        if delivery.reason == primaryReason, context.failureCause == nil,
           delivery.context.failureCause != nil {
            context = delivery.context
        }
        // Pending/cancellation/unknown cannot revoke an acknowledgement. Failed is sticky.
        if stopOutcome != .failed {
            switch delivery.stopOutcome {
            case .failed: stopOutcome = .failed
            case .confirmed where !delivery.stale: stopOutcome = .confirmed
            case .pending where stopOutcome == .unknown: stopOutcome = .pending
            default: break
            }
        }
        deduplicated = seen && previousReason == primaryReason && previousStop == stopOutcome
            && previousCause == context.failureCause
        sources.insert(delivery.source.rawValue)
    }
    var diagnosticReason: String {
        if primaryReason == .stalled, context.purpose == .followReady,
           let cause = context.failureCause, cause.readyDescription != nil { return cause.rawValue }
        if primaryReason == .stalled, context.purpose == .followScan { return "no_yaw_progress" }
        switch primaryReason {
        case .noPose: return "no_pose"
        case .noPath: return "no_path"
        case .pathRejected: return "path_rejected"
        case .obstacle: return "obstacle"
        case .commsLost: return "comms_lost"
        case .tipping: return "tipping"
        case .stalled: return "stalled"
        case .rotationResolutionInsufficient: return context.failureCause?.rawValue ?? "rotation_resolution_insufficient"
        case .commandFailed: return "transport_failed"
        case .trackingLost: return context.failureCause == .poseSourceStale ? "pose_source_stale" : "tracking_lost"
        case .cancelled: return "cancelled"
        }
    }
    var priority: Int {
        stopOutcome == .failed ? 3 : (diagnosticReason == "no_yaw_progress" || primaryReason == .rotationResolutionInsufficient ? 2 : 1)
    }
    var message: String {
        if stopOutcome == .failed { return "Motor stop could not be confirmed. Motion is blocked." }
        if diagnosticReason == "no_yaw_progress" {
            return stopOutcome == .confirmed
                ? "Search rotation stopped: insufficient measured yaw progress. Stop confirmed. Restart following to try again."
                : "Search rotation stopped: insufficient measured yaw progress. Confirming motor stop…"
        }
        switch primaryReason {
        case .noPose: return "Rover pose unavailable. Wait for AR tracking to recover."
        case .trackingLost:
            return context.failureCause == .poseSourceStale
                ? "Camera pose is stale. Motion stopped; wait for fresh camera frames."
                : "AR tracking lost. Wait for tracking to recover."
        case .obstacle: return "Obstacle detected. Motion stopped."
        case .commsLost: return "Rover communication lost. Motion stopped."
        case .tipping: return "Rover tipping detected. Motion stopped."
        case .stalled:
            if context.purpose == .followReady, let description = context.failureCause?.readyDescription {
                let stop = stopOutcome == .confirmed ? "Stop confirmed. Restart following to try again." : "Confirming motor stop…"
                return "Ready signal stopped: \(description). \(stop)"
            }
            return "Navigation stopped: insufficient measured progress."
        case .rotationResolutionInsufficient:
            let turn = context.purpose == .followScan ? "Search rotation" : (context.purpose == .followAlignment ? "Person alignment" : "Turn")
            let stop = stopOutcome == .confirmed ? "Stop confirmed. Restart following to try again." : "Confirming motor stop…"
            if context.failureCause == .burstPreSendExpired {
                return "Turn not started: command scheduling exceeded burst budget. \(stop)"
            }
            if context.failureCause == .calibrationEvidenceIncomplete {
                return "\(turn) stopped: calibration pose evidence is incomplete or unreliable. \(stop)"
            }
            return "\(turn) stopped: observed response is too coarse for the remaining angle. \(stop)"
        case .noPath: return "No navigation path available."
        case .pathRejected: return "Navigation path rejected."
        case .commandFailed: return "Navigation command failed."
        case .cancelled: return "Navigation cancelled."
        }
    }
}
