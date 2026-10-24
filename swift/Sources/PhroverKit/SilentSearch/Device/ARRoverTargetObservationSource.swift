import Foundation
import RoverNav

@MainActor
public final class ARRoverTargetObservationSource: SilentSearchTargetObserving {
    public typealias DetectorFunction = (ARFrameSnapshot) -> Detector.FrameDetections

    private let snapshots: () -> AsyncStream<ARFrameSnapshot>
    private let clock: any SilentSearchClock
    private let sharedFrame: SharedMissionFrame
    private let detector: DetectorFunction
    private var evaluateAsync: ((ARFrameSnapshot) async -> Detector.FollowEvaluation?)?
    private var currentGeneration: (() -> UInt64)?
    private let tracker: SilentSearchTargetTracker

    public init(sessionManager: ARSessionManager, clock: any SilentSearchClock,
                detector: Detector, canonicalLabel: String, sharedFrame: SharedMissionFrame,
                eventSink: TargetTrackerEventSink? = nil,
                events: (any SilentSearchEventSink)? = nil) {
        snapshots = { sessionManager.snapshots() }
        self.clock = clock
        self.sharedFrame = sharedFrame
        self.detector = { detector.detect($0) }
        self.evaluateAsync = { await detector.evaluateForMissionAsync($0) }
        self.currentGeneration = { sessionManager.sessionGeneration }
        tracker = SilentSearchTargetTracker(canonicalLabel: canonicalLabel, frame: sharedFrame,
                                eventSink: eventSink, events: events)
    }

    init(sessionManager: ARSessionManager, clock: any SilentSearchClock, canonicalLabel: String,
         sharedFrame: SharedMissionFrame, eventSink: TargetTrackerEventSink? = nil,
         events: (any SilentSearchEventSink)? = nil,
         detector: @escaping DetectorFunction) {
        snapshots = { sessionManager.snapshots() }
        self.clock = clock
        self.sharedFrame = sharedFrame
        self.detector = detector
        tracker = SilentSearchTargetTracker(canonicalLabel: canonicalLabel, frame: sharedFrame,
                                eventSink: eventSink, events: events)
    }

    init(clock: any SilentSearchClock, canonicalLabel: String, sharedFrame: SharedMissionFrame,
         snapshots: @escaping () -> AsyncStream<ARFrameSnapshot>,
         detector: @escaping DetectorFunction) {
        self.snapshots = snapshots
        self.clock = clock
        self.sharedFrame = sharedFrame
        self.detector = detector
        tracker = SilentSearchTargetTracker(canonicalLabel: canonicalLabel, frame: sharedFrame)
    }

    func frameObservation(in snapshot: ARFrameSnapshot) -> TargetFrameObservation? {
        guard snapshot.id.generation == sharedFrame.sessionGeneration else { return nil }
        return frameObservation(in: snapshot, result: detector(snapshot))
    }

    private func frameObservation(in snapshot: ARFrameSnapshot, result: Detector.FrameDetections) -> TargetFrameObservation? {
        guard snapshot.id.generation == sharedFrame.sessionGeneration else { return nil }
        guard result.frameID == snapshot.id,
              result.monotonicTimestamp == snapshot.timestamp else { return nil }
        let detections = result.detections.map { detection in
            let center = CGPoint(x: detection.boundingBox.midX, y: detection.boundingBox.midY)
            return TargetDetectionObservation(
                label: detection.label, confidence: Double(detection.confidence),
                normalizedCenter: Vec2(Double(center.x), Double(center.y)),
                localGroundedPoint: ARSessionManager.unproject(normalizedPoint: center, in: snapshot)
            )
        }
        return TargetFrameObservation(frameID: snapshot.id.sequence,
                                      monotonicTimestamp: snapshot.timestamp, detections: detections)
    }

    func process(_ snapshot: ARFrameSnapshot) -> SilentSearchTargetObservationResult {
        guard let observation = frameObservation(in: snapshot) else { return .pending }
        switch tracker.process(observation) {
        case .collecting: return .pending
        case let .confirmed(confirmation): return .confirmed(confirmation)
        }
    }

    public func observeNextFrame(until deadline: SilentSearchInstant) async -> SilentSearchTargetObservationResult {
        guard clock.monotonicNow < deadline else { return .pending }
        let stream = AsyncStream<ARFrameSnapshot?> { continuation in
            let frameTask = Task { @MainActor [weak self] in
                guard let self else {
                    continuation.yield(nil)
                    continuation.finish()
                    return
                }
                for await snapshot in self.snapshots() {
                    if Task.isCancelled { break }
                    continuation.yield(snapshot)
                    continuation.finish()
                    return
                }
                continuation.yield(nil)
                continuation.finish()
            }
            let timeoutTask = Task { @MainActor [weak self] in
                guard let self else { return }
                try? await self.clock.sleep(until: deadline)
                if !Task.isCancelled {
                    continuation.yield(nil)
                    continuation.finish()
                }
            }
            continuation.onTermination = { @Sendable _ in
                frameTask.cancel()
                timeoutTask.cancel()
            }
        }
        for await snapshot in stream {
            guard let snapshot, !Task.isCancelled else { return .pending }
            guard let evaluateAsync else { return process(snapshot) }
            // Native Vision remains serialized on its worker; waiting never blocks
            // MainActor, so cancellation and motor-stop work can still execute.
            guard let evaluation = await evaluateAsync(snapshot), !Task.isCancelled,
                  clock.monotonicNow < deadline,
                  evaluation.snapshot.id.generation == currentGeneration?(),
                  evaluation.snapshot.trackingQuality == .normal,
                  evaluation.receipt.status == .executed else { return .pending }
            let age = ProcessInfo.processInfo.systemUptime - evaluation.snapshot.timestamp
            guard age.isFinite, age >= 0, age <= 0.5,
                  let observation = frameObservation(in: evaluation.snapshot, result: evaluation.receipt.frame) else { return .pending }
            switch tracker.process(observation) {
            case .collecting: return .pending
            case .confirmed(let confirmation): return .confirmed(confirmation)
            }
        }
        return .pending
    }
}
