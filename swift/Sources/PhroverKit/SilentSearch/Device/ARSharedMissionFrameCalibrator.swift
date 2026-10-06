import Foundation
import RoverNav

@MainActor
public final class ARSharedMissionFrameCalibrator: SilentSearchCalibrating {
    public typealias Scanner = @Sendable (OpticalFrame) throws -> [OpticalObservation]
    typealias DetailedScanner = @Sendable (OpticalFrame) throws -> OpticalScanOutcome
    typealias Grounder = @MainActor @Sendable (
        OpticalObservation, ARFrameSnapshot, String, UInt64
    ) -> Result<SharedMissionCalibrationObservation, SilentSearchCalibrationGroundingFailure>

    @MainActor
    private final class CalibrationAttemptState {
        var lastFailure: SilentSearchCalibrationIssue?

        func markerExpired() {
            switch lastFailure {
            case .scannerFailure, .groundingFailure(.wrongMarkerID): break
            default: lastFailure = nil
            }
        }
    }

    private let sessionManager: ARSessionManager
    private let scanner: DetailedScanner
    private let grounder: Grounder
    private let eventSink: (any SilentSearchEventSink)?
    private let markerExpirySleep: @MainActor @Sendable () async throws -> Void
    private var task: Task<Void, Never>?
    private var markerExpiryTask: Task<Void, Never>?
    private var visibleMarkerContext: SilentSearchCalibrationFrameContext?
    private var calibrationAttempt: CalibrationAttemptState?

    public init(sessionManager: ARSessionManager, scanner: OpticalQRCodeScanner = OpticalQRCodeScanner(),
                events: (any SilentSearchEventSink)? = nil) {
        self.sessionManager = sessionManager
        self.scanner = { try scanner.scanDetailed($0) }
        grounder = Self.groundValidated
        self.eventSink = events
        markerExpirySleep = { try await Task.sleep(for: .milliseconds(500)) }
    }

    init(sessionManager: ARSessionManager, events: (any SilentSearchEventSink)? = nil,
         markerExpirySleep: @escaping @MainActor @Sendable () async throws -> Void = {
             try await Task.sleep(for: .milliseconds(500))
         },
         grounder: @escaping Grounder = ARSharedMissionFrameCalibrator.groundValidated,
         scanner: @escaping Scanner) {
        self.sessionManager = sessionManager
        self.scanner = { OpticalScanOutcome(observations: try scanner($0)) }
        self.grounder = grounder
        self.eventSink = events
        self.markerExpirySleep = markerExpirySleep
    }

    init(sessionManager: ARSessionManager, events: (any SilentSearchEventSink)? = nil,
         markerExpirySleep: @escaping @MainActor @Sendable () async throws -> Void = {
             try await Task.sleep(for: .milliseconds(500))
         },
         grounder: @escaping Grounder = ARSharedMissionFrameCalibrator.groundValidated,
         detailedScanner: @escaping DetailedScanner) {
        self.sessionManager = sessionManager
        self.scanner = detailedScanner
        self.grounder = grounder
        self.eventSink = events
        self.markerExpirySleep = markerExpirySleep
    }

    public func events(markerID: String, sessionGeneration: UInt64) -> AsyncStream<SilentSearchCalibrationEvent> {
        cancel()
        return AsyncStream { continuation in
            let attempt = CalibrationAttemptState()
            calibrationAttempt = attempt
            task = Task { @MainActor [sessionManager, scanner, grounder, eventSink] in
                guard let configuration = SharedMissionCalibrationConfiguration(markerID: markerID) else {
                    continuation.yield(.rejected(.invalidMarkerID))
                    continuation.finish()
                    return
                }
                var calibrator = SharedMissionCalibrator(
                    configuration: configuration, sessionGeneration: sessionGeneration, events: eventSink
                )
                for await snapshot in sessionManager.snapshots() {
                    guard !Task.isCancelled else { break }
                    let context = SilentSearchCalibrationFrameContext(
                        frameID: snapshot.id, monotonicTimestamp: snapshot.timestamp
                    )
                    guard snapshot.trackingQuality == .normal else {
                        let issue = SilentSearchCalibrationIssue.trackingNotNormal
                        if issue != attempt.lastFailure {
                            continuation.yield(.feedback(.trackingNotNormal(context: context)))
                            attempt.lastFailure = issue
                        }
                        continue
                    }
                    guard snapshot.id.generation == sessionGeneration else {
                        continuation.yield(.feedback(.groundingFailed(
                            context: context, reason: .generationMismatch
                        )))
                        continuation.yield(.rejected(.generationMismatch))
                        break
                    }
                    let opticalFrame = OpticalFrame(pixelBuffer: snapshot.image,
                        arFrameID: snapshot.id, monotonicTimestamp: snapshot.timestamp)
                    let scans: [OpticalObservation]
                    do {
                        let outcome = try scanner(opticalFrame)
                        for diagnostic in outcome.diagnostics {
                            continuation.yield(.feedback(.scannerBackendFailed(
                                context: context, diagnostic: diagnostic
                            )))
                        }
                        scans = outcome.observations
                        if scans.isEmpty, attempt.lastFailure != nil {
                            continuation.yield(.feedback(.waitingForMarker(context: context)))
                            attempt.lastFailure = nil
                        } else if scans.isEmpty, outcome.diagnostics.isEmpty {
                            continuation.yield(.feedback(.scanCompleted(context: context)))
                        }
                    } catch {
                        if attempt.lastFailure != .scannerFailure {
                            continuation.yield(.feedback(.scannerFailed(
                                context: context
                            )))
                            attempt.lastFailure = .scannerFailure
                        }
                        scans = []
                    }
                    for scan in scans {
                        let preflight = Self.preflight(
                            observation: scan, in: snapshot, expectedMarkerID: markerID,
                            sessionGeneration: sessionGeneration
                        )
                        guard case .success = preflight else {
                            guard case let .failure(reason) = preflight else { continue }
                            let issue = SilentSearchCalibrationIssue.groundingFailure(reason)
                            if issue != attempt.lastFailure {
                                continuation.yield(.feedback(.groundingFailed(
                                    context: context, reason: reason
                                )))
                                attempt.lastFailure = issue
                            }
                            continue
                        }
                        scheduleMarkerExpiry(
                            context: context, attempt: attempt, continuation: continuation
                        )
                        continuation.yield(.feedback(.expectedMarkerDetected(
                            context: context, markerID: markerID, corners: scan.corners
                        )))
                        await Task.yield()
                        let result = grounder(scan, snapshot, markerID, sessionGeneration)
                        guard case let .success(observation) = result else {
                            guard case let .failure(reason) = result else { continue }
                            let issue = SilentSearchCalibrationIssue.groundingFailure(reason)
                            if issue != attempt.lastFailure {
                                continuation.yield(.feedback(.groundingFailed(
                                    context: context, reason: reason
                                )))
                                attempt.lastFailure = issue
                            }
                            continue
                        }
                        continuation.yield(.feedback(.allCornersGrounded(
                            context: context
                        )))
                        attempt.lastFailure = nil
                        switch calibrator.observe(observation) {
                        case let .collecting(frameCount):
                            continuation.yield(.progress(
                                context: context, acceptedFrameCount: frameCount
                            ))
                        case let .rejected(diagnostic):
                            continuation.yield(.rejected(diagnostic))
                        case let .accepted(frame):
                            continuation.yield(.progress(
                                context: context, acceptedFrameCount: 3
                            ))
                            continuation.yield(.accepted(frame))
                            continuation.finish()
                            return
                        }
                    }
                }
                continuation.finish()
            }
            continuation.onTermination = { @Sendable [weak self] _ in
                Task { @MainActor in self?.cancel() }
            }
        }
    }

    public func cancel() {
        task?.cancel()
        task = nil
        markerExpiryTask?.cancel()
        markerExpiryTask = nil
        visibleMarkerContext = nil
        calibrationAttempt = nil
    }

    private func scheduleMarkerExpiry(
        context: SilentSearchCalibrationFrameContext,
        attempt: CalibrationAttemptState,
        continuation: AsyncStream<SilentSearchCalibrationEvent>.Continuation
    ) {
        visibleMarkerContext = context
        markerExpiryTask?.cancel()
        markerExpiryTask = Task { @MainActor [weak self] in
            guard let self else { return }
            do { try await self.markerExpirySleep() }
            catch { return }
            guard self.calibrationAttempt === attempt,
                  self.visibleMarkerContext == context else { return }
            self.visibleMarkerContext = nil
            self.markerExpiryTask = nil
            attempt.markerExpired()
            continuation.yield(.feedback(.qrLost(context: context)))
        }
    }

    nonisolated static func ground(
        observation: OpticalObservation,
        in snapshot: ARFrameSnapshot,
        expectedMarkerID: String,
        sessionGeneration: UInt64
    ) -> Result<SharedMissionCalibrationObservation, SilentSearchCalibrationGroundingFailure> {
        switch preflight(
            observation: observation, in: snapshot, expectedMarkerID: expectedMarkerID,
            sessionGeneration: sessionGeneration
        ) {
        case let .failure(reason): return .failure(reason)
        case .success:
            return groundValidated(
                observation, snapshot, expectedMarkerID, sessionGeneration
            )
        }
    }

    private nonisolated static func preflight(
        observation: OpticalObservation,
        in snapshot: ARFrameSnapshot,
        expectedMarkerID: String,
        sessionGeneration: UInt64
    ) -> Result<Void, SilentSearchCalibrationGroundingFailure> {
        guard snapshot.trackingQuality == .normal else { return .failure(.trackingNotNormal) }
        guard snapshot.id.generation == sessionGeneration else { return .failure(.generationMismatch) }
        guard observation.frameID == snapshot.id.sequence else { return .failure(.frameMismatch) }
        guard observation.monotonicTimestamp == snapshot.timestamp else { return .failure(.timestampMismatch) }
        guard let payload = String(data: observation.payload, encoding: .utf8),
              let markerID = SharedMissionCalibrator.markerID(fromPayload: payload) else {
            return .failure(.invalidPayload)
        }
        guard markerID == expectedMarkerID else { return .failure(.wrongMarkerID) }
        return .success(())
    }

    private nonisolated static func groundValidated(
        _ observation: OpticalObservation,
        _ snapshot: ARFrameSnapshot,
        _ expectedMarkerID: String,
        _ sessionGeneration: UInt64
    ) -> Result<SharedMissionCalibrationObservation, SilentSearchCalibrationGroundingFailure> {
        guard snapshot.depthMap != nil else { return .failure(.missingDepthMap) }
        func point(_ normalized: Vec2) -> Vec2? {
            ARSessionManager.unproject(
                normalizedPoint: CGPoint(x: normalized.x, y: normalized.y), in: snapshot
            )
        }
        guard let topLeft = point(observation.corners.topLeft) else {
            return .failure(.cornerUnavailable(.topLeft))
        }
        guard let topRight = point(observation.corners.topRight) else {
            return .failure(.cornerUnavailable(.topRight))
        }
        guard let bottomLeft = point(observation.corners.bottomLeft) else {
            return .failure(.cornerUnavailable(.bottomLeft))
        }
        guard let bottomRight = point(observation.corners.bottomRight) else {
            return .failure(.cornerUnavailable(.bottomRight))
        }
        return .success(SharedMissionCalibrationObservation(
            markerID: expectedMarkerID, sessionGeneration: snapshot.id.generation,
            frameID: snapshot.id.sequence, monotonicTimestamp: snapshot.timestamp,
            corners: OrientedMarkerCorners(topLeft: topLeft, topRight: topRight,
                                           bottomLeft: bottomLeft, bottomRight: bottomRight)
        ))
    }
}
