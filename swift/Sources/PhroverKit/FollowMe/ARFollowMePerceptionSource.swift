import Foundation
import ARKit
import RoverNav

/// Only detections and depth belonging to a single rear-camera AR snapshot may be combined.
@MainActor
public final class ARFollowMePerceptionSource: FollowMePerception {
    private let ar: ARSessionManager
    private let detector: Detector

    public var detectorReady: Bool { detector.isLoaded }
    public var personLabelAvailable: Bool { detector.supportedCanonicalLabels.contains("person") }

    public init(ar: ARSessionManager, detector: Detector) {
        self.ar = ar
        self.detector = detector
    }

    public func events() -> AsyncStream<FollowPerceptionEvent> {
        let frames = ar.snapshots()
        let lifecycle = ar.lifecycleEvents()
        let mailbox = FollowPerceptionMailbox()
        let consumer = detector.beginFollowConsumer()
        let frameTask = Task { [detector, ar] in
            var lastLatencySummary = -Double.infinity
            var lastRequestedFrame: ARFrameID?
            for await queued in frames {
                guard !Task.isCancelled else { break }
                // Select before inference, then retain that exact image/depth/
                // pose pairing through completion. Never re-age old detections.
                let snapshot = Self.inferenceSnapshot(queued: queued, latest: ar.latestSnapshot)
                guard snapshot.id.generation == ar.sessionGeneration, snapshot.id != lastRequestedFrame else { continue }
                lastRequestedFrame = snapshot.id
                // Diagnostic reason is live; eligibility uses the chosen snapshot.
                let reason = ar.session.currentFrame.map { Self.trackingReason(from: $0.camera.trackingState) } ?? nil
                if snapshot.trackingQuality != .normal {
                    mailbox.publish(Self.batch(from: snapshot, detections: [], trackingReason: reason))
                    continue
                }
                let started = ProcessInfo.processInfo.systemUptime
                guard let receipt = await detector.evaluateForFollowAsync(snapshot), !Task.isCancelled else { continue }
                guard ar.sessionGeneration == snapshot.id.generation else { continue }
                let duration = ProcessInfo.processInfo.systemUptime - started
                mailbox.publish(Self.batch(from: snapshot, detections: receipt.frame.detections,
                    trackingReason: reason, inferenceDuration: duration,
                    inferenceStatus: receipt.status == .executed ? .executed : .failed,
                    inferenceFailureReason: receipt.failureReason,
                    personVerification: receipt.personVerification ?? []))
                let published = ProcessInfo.processInfo.systemUptime
                if published - lastLatencySummary >= 1 {
                    lastLatencySummary = published
                    RuntimeFileLog.append("follow_perception_latency", fields: [
                        "frame_id": "\(snapshot.id.generation):\(snapshot.id.sequence)",
                        "capture_to_inference_start_s": String(started - snapshot.timestamp),
                        "inference_s": String(duration),
                        "capture_to_publish_s": String(published - snapshot.timestamp),
                        "coalesced_completed_frames": String(mailbox.coalescedFrames)])
                }
            }
        }
        let lifecycleTask = Task {
            for await event in lifecycle {
                guard !Task.isCancelled else { break }
                switch event {
                case .reset, .interrupted: mailbox.finish(with: .interrupted)
                case .failed(_, let description): mailbox.finish(with: .failed(description))
                case .interruptionEnded: continue
                }
                frameTask.cancel()
                detector.endFollowConsumer(consumer)
                return
            }
        }
        return mailbox.stream { @Sendable [detector] in
            frameTask.cancel()
            lifecycleTask.cancel()
            detector.endFollowConsumer(consumer)
        }
    }

    static func inferenceSnapshot(queued: ARFrameSnapshot, latest: ARFrameSnapshot?) -> ARFrameSnapshot {
        guard queued.trackingQuality == .normal, queued.timestamp.isFinite,
              queued.pose.position.x.isFinite, queued.pose.position.y.isFinite, queued.pose.yaw.isFinite,
              let latest, latest.id.generation == queued.id.generation,
              latest.id.sequence >= queued.id.sequence,
              queued.depthMap != nil || latest.depthMap == nil else { return queued }
        return latest
    }

    public static func batch(from snapshot: ARFrameSnapshot,
                             detections: [Detector.Detection],
                             trackingReason: FollowTrackingReason? = nil,
                             inferenceDuration: TimeInterval? = nil,
                              inferenceStatus: FollowInferenceStatus = .executed,
                               inferenceFailureReason: Detector.FailureReason? = nil,
                               personVerification: [PersonBodyVerifier.Decision]? = nil) -> FollowFrameBatch {
        let rawPeople = detections.filter { $0.label.lowercased() == "person" }
        let status: FollowInferenceStatus = snapshot.trackingQuality == .normal ? inferenceStatus : .skippedTracking
        let evaluated = status != .skippedTracking && status != .failed
        let detectorCountsKnown = status == .executed
        // This public projection helper preserves trusted legacy callers. Live
        // production always supplies explicit body decisions (including none).
        let verified = personVerification.map { Set($0.filter(\.accepted).map(\.rawPersonID)) }
        let candidates: [FollowProjectionEvidence] = evaluated ? rawPeople.enumerated().filter { verified?.contains($0.offset) ?? true }.map { id, detection in
            let decision = personVerification?.first { $0.rawPersonID == id && $0.accepted }
            return FollowPersonProjection.evaluate(box: decision?.normalizedBox ?? detection.boundingBox,
                detectorConfidence: detection.confidence, rawPersonID: id, in: snapshot,
                bodyVerified: decision != nil, verifiedDepthAnchor: decision?.depthAnchor)
        } : []
        let people: [FollowPersonObservation] = candidates.compactMap { candidate in
            guard let position = candidate.position, candidate.rejection == nil,
                  position.x.isFinite, position.y.isFinite else { return nil }
            return FollowPersonObservation(frameID: snapshot.id, timestamp: snapshot.timestamp,
                                           confidence: candidate.detectorConfidence, boundingBox: candidate.box,
                                            position: position, pose: snapshot.pose, rawPersonID: candidate.rawPersonID,
                                            bodyVerified: verified?.contains(candidate.rawPersonID) == true)
        }
        let thresholds = FollowMeConfiguration()
        let acceptedRawIDs = Set(people.filter { $0.confidence >= thresholds.minimumConfidence }.compactMap(\.rawPersonID))
        let minimumPartialConfidence = thresholds.minimumPartialConfidence
        let partial: [FollowBearingObservation] = evaluated && snapshot.trackingQuality == .normal
            ? rawPeople.enumerated().compactMap { id, detection in
                guard !acceptedRawIDs.contains(id), detection.confidence >= minimumPartialConfidence,
                      let bearing = FollowPersonProjection.centerBearing(box: detection.boundingBox, in: snapshot)
                else { return nil }
                return FollowBearingObservation(frameID: snapshot.id, timestamp: snapshot.timestamp,
                    confidence: detection.confidence, boundingBox: detection.boundingBox,
                    bearing: bearing, pose: snapshot.pose, rawPersonID: id)
            } : []
        var diagnostics = FollowPerceptionDiagnostics(frameID: snapshot.id, timestamp: snapshot.timestamp,
            inferenceStatus: status, inferenceFailureReason: inferenceFailureReason,
            rawDetectorCount: detectorCountsKnown ? detections.count : nil, rawPersonCount: detectorCountsKnown ? rawPeople.count : nil,
            projectionAttemptedCount: evaluated ? candidates.count : nil,
            projectionAcceptedCount: evaluated ? people.count : nil,
            projectionRejectedCount: evaluated ? candidates.count - people.count : nil,
            projectedPersonCount: evaluated ? people.count : nil, candidates: candidates)
        diagnostics.personVerification = evaluated ? personVerification : nil
        return FollowFrameBatch(frameID: snapshot.id, timestamp: snapshot.timestamp,
                                pose: snapshot.trackingQuality == .normal ? snapshot.pose : nil,
                                depthAvailable: snapshot.depthMap != nil, people: people,
                                trackingQuality: snapshot.trackingQuality, trackingReason: trackingReason,
                                inferenceDuration: inferenceDuration, perceptionDiagnostics: diagnostics,
                                partialPeople: partial)
    }

    public static func trackingReason(from state: ARCamera.TrackingState) -> FollowTrackingReason? {
        switch state {
        case .normal: return nil
        case .notAvailable: return .notAvailable
        case .limited(let reason):
            switch reason {
            case .initializing: return .initializing
            case .excessiveMotion: return .excessiveMotion
            case .insufficientFeatures: return .insufficientFeatures
            case .relocalizing: return .relocalizing
            @unknown default: return .unknown
            }
        }
    }
}

@MainActor
public final class SystemFollowClock: FollowMeClock {
    public init() {}
    public var now: TimeInterval { ProcessInfo.processInfo.systemUptime }
    public func sleep(seconds: Double) async { try? await Task.sleep(for: .seconds(seconds)) }
}
