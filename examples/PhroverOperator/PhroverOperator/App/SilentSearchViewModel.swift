import CoreGraphics
import Observation
import PhroverKit
import RoverNav
import SwiftUI
import UIKit

enum SilentSearchOperatorPhase: Equatable {
    case setup
    case calibrating
    case readyForExchange
    case displayingQR
    case scanning
    case searching
    case returning
    case rendezvousRotating
    case intentionalWait
    case converging
    case terminalFound
    case terminalNotFound
    case terminalFailure
}

@MainActor
protocol SilentSearchViewModel: AnyObject, Observable {
    var phase: SilentSearchOperatorPhase { get }
    var role: RoverRole { get set }
    var targetLabel: String { get set }
    var durationSeconds: Int { get set }
    var supportedTargetLabels: [String] { get }
    var title: String { get }
    var detail: String { get }
    var readinessItems: [(String, Bool)] { get }
    var calibrationProgress: Int { get }
    var calibrationPreviewImage: UIImage? { get }
    var calibrationVisualState: SilentSearchCalibrationVisualState { get }
    var calibrationProjection: CalibrationViewProjection { get }
    var qrImage: CGImage? { get }
    var opticalTimedOut: Bool { get }
    var failureReason: String? { get }
    var mapState: SilentSearchMapState { get }
    var canStart: Bool { get }
    var showsStop: Bool { get }
    func refreshReadiness()
    func start()
    func completeQRPresentation()
    func retry()
    func abort()
    func stop()
    func startCalibrationPreview()
    func stopCalibrationPreview()
}

enum SilentSearchLaunchScenario: String, CaseIterable {
    case setupNotReady = "setup-not-ready"
    case calibrating
    case calibrationQRDetected = "calibration-qr-detected"
    case calibrationContextMismatch = "calibration-context-mismatch"
    case calibrationQRLost = "calibration-qr-lost"
    case calibrationGroundingFailed = "calibration-grounding-failed"
    case calibrationSampleAccepted = "calibration-sample-accepted"
    case displayOffer = "display-offer"
    case displayTimeout = "display-timeout"
    case scanTimeout = "scan-timeout"
    case searching
    case returning
    case rendezvousRotating = "rendezvous-rotating"
    case intentionalWait = "intentional-wait"
    case converging
    case notFound = "not-found"
    case found
    case navigationFailure = "navigation-failure"
}

@MainActor
@Observable
final class ScriptedSilentSearchViewModel: SilentSearchViewModel {
    var role: RoverRole = .a
    var targetLabel = "chair"
    var durationSeconds = 180
    let supportedTargetLabels = ["chair"]
    private(set) var phase: SilentSearchOperatorPhase
    private(set) var opticalTimedOut = false
    private(set) var failureReason: String?
    private(set) var calibrationVisualState = SilentSearchCalibrationVisualState()

    init(scenario: SilentSearchLaunchScenario) {
        switch scenario {
        case .setupNotReady: phase = .setup
        case .calibrating:
            phase = .calibrating
        case .calibrationQRDetected:
            phase = .calibrating
            calibrationVisualState.qrDecoded = true
            calibrationVisualState.recordDetection(.init(
                context: Self.previewFrameContext,
                markerID: "SILENT_SEARCH_01", corners: Self.previewCorners
            ))
        case .calibrationContextMismatch:
            phase = .calibrating
            calibrationVisualState.qrDecoded = true
            calibrationVisualState.recordDetection(.init(
                context: SilentSearchCalibrationFrameContext(
                    frameID: ARFrameID(generation: 1, sequence: 1), monotonicTimestamp: 1
                ),
                markerID: "SILENT_SEARCH_01", corners: Self.previewCorners
            ))
        case .calibrationQRLost:
            phase = .calibrating
            calibrationVisualState.qrDecoded = true
            calibrationVisualState.cornersGrounded = true
            calibrationVisualState.sampleAccepted = true
        case .calibrationGroundingFailed:
            phase = .calibrating
            calibrationVisualState.qrDecoded = true
            calibrationVisualState.recordDetection(.init(
                context: Self.previewFrameContext,
                markerID: "SILENT_SEARCH_01", corners: Self.previewCorners
            ))
            calibrationVisualState.currentIssue = .groundingFailure(.cornerUnavailable(.topLeft))
        case .calibrationSampleAccepted:
            phase = .calibrating
            calibrationVisualState.qrDecoded = true
            calibrationVisualState.cornersGrounded = true
            calibrationVisualState.sampleAccepted = true
            calibrationVisualState.recordDetection(.init(
                context: Self.previewFrameContext,
                markerID: "SILENT_SEARCH_01", corners: Self.previewCorners
            ))
        case .displayOffer: phase = .displayingQR
        case .displayTimeout:
            phase = .displayingQR
            opticalTimedOut = true
        case .scanTimeout:
            phase = .scanning
            opticalTimedOut = true
        case .searching: phase = .searching
        case .returning: phase = .returning
        case .rendezvousRotating: phase = .rendezvousRotating
        case .intentionalWait: phase = .intentionalWait
        case .converging: phase = .converging
        case .notFound: phase = .terminalNotFound
        case .found: phase = .terminalFound
        case .navigationFailure:
            phase = .terminalFailure
            failureReason = "Navigation failed: no path."
        }
    }

    var title: String {
        switch phase {
        case .setup: "Silent Search"
        case .calibrating: "Calibrate shared frame"
        case .readyForExchange: "Calibration accepted"
        case .displayingQR: "Display mission offer"
        case .scanning: "Scan partner QR"
        case .searching: "Searching west sector"
        case .returning: "Returning to rendezvous"
        case .rendezvousRotating: "Aligning for optical exchange"
        case .intentionalWait: "Waiting for partner"
        case .converging: "Converging on target"
        case .terminalFound: "FOUND"
        case .terminalNotFound: "NOT FOUND"
        case .terminalFailure: "Mission failed"
        }
    }

    var detail: String {
        switch phase {
        case .setup: "Not ready"
        case .calibrating: "Calibration 1 of 3"
        case .readyForExchange: "Shared marker frame is valid."
        case .displayingQR: "Position the other rover's rear camera over this code."
        case .scanning: opticalTimedOut ? "No valid QR code received within 30 seconds." : "Aim this rover's rear camera at the partner screen."
        case .searching: "Frontier frontier_4 · 118 seconds remaining"
        case .returning: "Navigating to Rover A staging pose."
        case .rendezvousRotating: "Rotating rear camera toward the partner screen."
        case .intentionalWait: "Stopped intentionally while waiting for the other rover."
        case .converging: "Target chair · final stand-off 0.60 m"
        case .terminalFound: "Both position and heading tolerances passed."
        case .terminalNotFound: "Both rovers completed the acknowledged search."
        case .terminalFailure: failureReason ?? "Mission failed."
        }
    }

    var readinessItems: [(String, Bool)] {
        [("Normal tracking", false), ("LiDAR depth", false), ("Target detector", true), ("Rover command link", false)]
    }

    var calibrationProgress: Int { phase == .calibrating ? 1 : 0 }
    var calibrationPreviewImage: UIImage? {
        guard phase == .calibrating else { return nil }
        return UIImage(systemName: "camera.metering.center.weighted")
    }
    var calibrationProjection: CalibrationViewProjection {
        CalibrationViewProjection(
            state: calibrationVisualState,
            latestRenderedFrameContext: Self.previewFrameContext
        )
    }

    var qrImage: CGImage? {
        guard phase == .displayingQR else { return nil }
        return try? OpticalQRCodeRenderer().render(payload: Data("PHROVER-UI-TEST-OFFER".utf8), moduleScale: 7)
    }

    var mapState: SilentSearchMapState { .preview }
    var canStart: Bool { phase == .readyForExchange }
    var showsStop: Bool { [.searching, .returning, .rendezvousRotating, .converging].contains(phase) }

    func refreshReadiness() {}
    func start() {}
    func completeQRPresentation() { phase = .scanning }
    func retry() { opticalTimedOut = false }
    func abort() {
        calibrationVisualState = SilentSearchCalibrationVisualState()
        phase = .terminalFailure
        failureReason = "Mission aborted by operator."
    }
    func stop() {
        calibrationVisualState = SilentSearchCalibrationVisualState()
        phase = .terminalFailure
        failureReason = "Mission stopped by operator."
    }
    func startCalibrationPreview() {}
    func stopCalibrationPreview() {}

    private static let previewCorners = OrientedMarkerCorners(
        topLeft: Vec2(0.2, 0.8), topRight: Vec2(0.8, 0.8),
        bottomLeft: Vec2(0.2, 0.2), bottomRight: Vec2(0.8, 0.2)
    )
    private static let previewFrameContext = SilentSearchCalibrationFrameContext(
        frameID: ARFrameID(generation: 1, sequence: 2), monotonicTimestamp: 2
    )
}

@MainActor
@Observable
final class LiveSilentSearchViewModel: SilentSearchViewModel {
    var role: RoverRole = .a
    var targetLabel: String {
        didSet {
            environment.targetLabel = targetLabel
            targetObserver.setTargetLabel(targetLabel)
            coordinator.refreshReadiness()
        }
    }
    var durationSeconds = 180

    private let coordinator: SilentSearchCoordinator
    private let environment: SilentSearchDeviceEnvironment
    private let presentation: LiveOpticalPresentation
    private let explorer: LiveSectorExplorer
    private let motion: LiveSilentSearchMotion
    private let targetObserver: LiveTargetObserver
    private let calibrationPreview: CalibrationPreviewModel

    init(coordinator: SilentSearchCoordinator, environment: SilentSearchDeviceEnvironment,
         presentation: LiveOpticalPresentation, explorer: LiveSectorExplorer,
         motion: LiveSilentSearchMotion, targetObserver: LiveTargetObserver,
         calibrationPreview: CalibrationPreviewModel,
         targetLabel: String) {
        self.targetLabel = targetLabel
        self.coordinator = coordinator
        self.environment = environment
        self.presentation = presentation
        self.explorer = explorer
        self.motion = motion
        self.targetObserver = targetObserver
        self.calibrationPreview = calibrationPreview
        coordinator.missionDidChange = { [weak self] mission in
            guard let self else { return }
            self.role = mission.role
            self.targetLabel = mission.targetLabel
            self.durationSeconds = Int(mission.searchDurationSeconds)
        }
    }

    static func compose(ar: ARSessionManager, control: RoverControl,
                        navigation: NavigationController, detector: Detector) -> LiveSilentSearchViewModel {
        let clock = RuntimeSilentSearchClock()
        let targetLabel = detector.supportedCanonicalLabels.sorted().first ?? ""
        let environment = SilentSearchDeviceEnvironment(
            ar: ar, detector: detector, control: control, targetLabel: targetLabel
        )
        let presentation = LiveOpticalPresentation()
        let optical = AROpticalExchangeService(sessionManager: ar, clock: clock) { payload in
            if let payload {
                try await presentation.present(payload)
            } else {
                presentation.cancel()
            }
        }
        let frameProvider = LiveSharedFrameProvider()
        let events = RuntimeSilentSearchEventSink()
        let explorer = LiveSectorExplorer(ar: ar, frameProvider: frameProvider, events: events)
        let motion = LiveSilentSearchMotion(ar: ar, navigation: navigation, frameProvider: frameProvider)
        let target = LiveTargetObserver(
            ar: ar, clock: clock, detector: detector, frameProvider: frameProvider,
            targetLabel: targetLabel, events: events
        )
        let coordinator = SilentSearchCoordinator(dependencies: SilentSearchDependencies(
            clock: clock,
            readiness: environment,
            calibration: ARSharedMissionFrameCalibrator(sessionManager: ar, events: events),
            opticalExchange: optical,
            explorer: explorer,
            targetObserver: target,
            motion: motion,
            safety: LiveSilentSearchSafetyMonitor(ar: ar, navigation: navigation, control: control),
            events: events
        ))
        frameProvider.coordinator = coordinator
        return LiveSilentSearchViewModel(
            coordinator: coordinator, environment: environment, presentation: presentation,
            explorer: explorer, motion: motion, targetObserver: target,
            calibrationPreview: CalibrationPreviewModel(frames: { ar.snapshots() }),
            targetLabel: targetLabel
        )
    }

    var phase: SilentSearchOperatorPhase {
        switch coordinator.phase {
        case .setup: .setup
        case .calibrating: .calibrating
        case .handshake(.ready): .readyForExchange
        case .handshake(.presenting), .rendezvous(.presenting): .displayingQR
        case .handshake(.scanning), .rendezvous(.scanning): .scanning
        case .rendezvous(.rotating): .rendezvousRotating
        case .waitingForSearch, .rendezvous(.ready), .rendezvous(.waiting), .waitingForConvergence:
            .intentionalWait
        case .searching: .searching
        case .returning: .returning
        case .converging: .converging
        case .terminal(.success): .terminalFound
        case .terminal(.notFound): .terminalNotFound
        case .terminal: .terminalFailure
        }
    }

    var title: String {
        switch phase {
        case .setup: "Silent Search"
        case .calibrating: "Calibrate shared frame"
        case .readyForExchange: "Calibration accepted"
        case .displayingQR: "Show QR to partner"
        case .scanning: "Scan partner QR"
        case .searching: "Searching \(role.searchSector.rawValue) sector"
        case .returning: "Returning to rendezvous"
        case .rendezvousRotating: "Aligning for optical exchange"
        case .intentionalWait: "Waiting"
        case .converging: "Converging on target"
        case .terminalFound: "FOUND"
        case .terminalNotFound: "NOT FOUND"
        case .terminalFailure: "Mission failed"
        }
    }

    var detail: String {
        switch phase {
        case .setup: readinessItems.allSatisfy(\.1) ? "Ready" : "Not ready"
        case .calibrating: "Calibration \(calibrationProgress) of 3"
        case .readyForExchange: "Shared marker frame is valid."
        case .displayingQR: "Position the other rover's rear camera over this code."
        case .scanning:
            opticalTimedOut ? "No valid QR code received within 30 seconds." : "Aim this rover's rear camera at the partner screen."
        case .searching: "Following eligible frontiers with sector policy enforced."
        case .returning: "Navigating to the fixed \(role.rawValue.uppercased()) rendezvous pose."
        case .rendezvousRotating: "Rotating rear camera toward the partner screen."
        case .intentionalWait: "Stopped intentionally while waiting for the other rover."
        case .converging: "Using unrestricted convergence after acknowledged release."
        case .terminalFound: "Both position and heading tolerances passed."
        case .terminalNotFound: "Both rovers completed the acknowledged search."
        case .terminalFailure: failureReason ?? "Mission failed."
        }
    }

    var readinessItems: [(String, Bool)] {
        let readiness = coordinator.readiness
        let trackingReady: Bool = if case .normal = readiness.tracking { true } else { false }
        return [
            ("Normal tracking", trackingReady),
            ("LiDAR depth", readiness.lidarAvailable && readiness.generationValid),
            ("Target detector", readiness.detectorLoaded && readiness.detectorLabelAvailable),
            ("Rover command link", readiness.commandLinkAvailable),
        ]
    }

    var calibrationProgress: Int { coordinator.calibrationProgress }
    var calibrationPreviewImage: UIImage? { calibrationPreview.image }
    var calibrationVisualState: SilentSearchCalibrationVisualState { coordinator.calibrationVisualState }
    var calibrationProjection: CalibrationViewProjection {
        CalibrationViewProjection(
            state: calibrationVisualState,
            expectedMarkerID: coordinator.mission?.markerID ?? "SILENT_SEARCH_01",
            latestRenderedFrameContext: calibrationPreview.latestRenderedFrameContext
        )
    }
    var supportedTargetLabels: [String] { environment.supportedTargetLabels }
    var qrImage: CGImage? {
        guard phase == .displayingQR, let payload = presentation.payload else { return nil }
        return try? OpticalQRCodeRenderer().render(payload: payload, moduleScale: 7)
    }
    var opticalTimedOut: Bool { coordinator.diagnostic == .opticalTimedOut }
    var failureReason: String? {
        guard case let .terminal(result) = coordinator.phase else { return nil }
        return Self.failureMessage(result)
    }
    var mapState: SilentSearchMapState {
        let target = coordinator.targetConfirmation?.coordinate
        let standOff = target.flatMap { target in
            MissionPoint(x: target.x + (role == .a ? -SilentSearchGeometry.targetOffset : SilentSearchGeometry.targetOffset),
                         y: target.y)
        }
        return SilentSearchMapState(
            role: role,
            rover: motion.currentMissionPose,
            frontiers: explorer.candidates.map {
                let status: SilentSearchMapFrontierStatus = switch $0.status {
                case .available: .available
                case .visited: .visited
                case .rejected: .rejected
                }
                return SilentSearchMapFrontier(
                    id: $0.stableID,
                    point: CGPoint(x: $0.missionCentroid.x, y: $0.missionCentroid.y),
                    status: status
                )
            },
            path: motion.currentMissionPath,
            target: target,
            standOff: standOff
        )
    }
    var canStart: Bool {
        switch phase {
        case .setup: coordinator.readiness.missingRequirements.isEmpty
        case .readyForExchange: true
        default: false
        }
    }
    var showsStop: Bool { [.searching, .returning, .rendezvousRotating, .converging].contains(phase) }

    func refreshReadiness() {
        Task {
            await environment.refreshCommandLink()
            coordinator.refreshReadiness()
        }
    }

    func start() {
        switch phase {
        case .setup:
            Task {
                await environment.refreshCommandLink()
                coordinator.refreshReadiness()
                guard let mission = SilentSearchMission(
                    role: role, targetLabel: targetLabel,
                    searchDurationSeconds: UInt32(durationSeconds), markerID: "SILENT_SEARCH_01"
                ) else { return }
                coordinator.configure(mission)
                _ = coordinator.startCalibration()
            }
        case .readyForExchange:
            _ = coordinator.startHandshake()
        default: break
        }
    }

    func completeQRPresentation() { presentation.complete() }
    func retry() { _ = coordinator.retryOpticalExchange() }
    func abort() {
        calibrationPreview.stop()
        Task { await coordinator.abort() }
    }
    func stop() {
        calibrationPreview.stop()
        Task { await coordinator.stop() }
    }
    func startCalibrationPreview() {
        guard phase == .calibrating else { return }
        calibrationPreview.start()
    }
    func stopCalibrationPreview() { calibrationPreview.stop() }

    private static func failureMessage(_ result: SilentSearchTerminalResult) -> String? {
        switch result {
        case .success, .notFound: nil
        case .operatorStopped: "Mission stopped by operator."
        case .operatorAborted: "Mission aborted by operator."
        case .partnerTimeout: "Partner did not complete rendezvous within 60 seconds."
        case .calibrationInvalidated: "Shared calibration was invalidated."
        case .safetyFailure(.transport): "Safety failure: rover transport lost."
        case .safetyFailure(.reactiveSafety): "Safety failure: reactive safety stopped the rover."
        case let .motionFailure(failure): motionFailureMessage(failure)
        case let .protocolFailure(rejection): "Optical protocol failed: \(protocolName(rejection))."
        }
    }

    private static func motionFailureMessage(_ failure: SilentSearchMotionFailure) -> String {
        switch failure {
        case .noPose: "Navigation failed: rover pose unavailable."
        case .noPath: "Navigation failed: no path."
        case .pathRejected: "Navigation failed: path left the allowed sector."
        case .obstacle: "Navigation failed: obstacle blocked motion."
        case .commandLink: "Navigation failed: rover command link lost."
        case .tipping: "Navigation failed: tipping risk detected."
        case .stalled: "Navigation failed: rover stalled."
        case .tracking: "Navigation failed: AR tracking lost."
        case .positionToleranceExceeded: "Navigation failed: position tolerance not reached."
        case .headingToleranceExceeded: "Navigation failed: heading tolerance not reached."
        }
    }

    private static func protocolName(_ rejection: OpticalProtocolRejection) -> String {
        switch rejection {
        case .codec: "invalid QR payload"
        case .unexpectedPhase: "message received in the wrong phase"
        case .wrongMission: "mission identifier mismatch"
        case .wrongMarker: "marker identifier mismatch"
        case .wrongRole: "rover role mismatch"
        case .sequenceNotIncreasing: "message sequence was not increasing"
        case .clockDisagreement: "rover clocks disagree"
        case .invalidSchedule: "invalid mission schedule"
        case .invalidLinkedHash: "acknowledgement hash mismatch"
        case .invalidDecision: "invalid rendezvous decision"
        case .convergenceAfterConflict: "convergence forbidden after conflicting reports"
        case .noOutgoingMessage: "no QR message available to retry"
        }
    }
}

@MainActor
@Observable
final class LiveOpticalPresentation {
    private(set) var payload: Data?
    @ObservationIgnored private var continuation: CheckedContinuation<Void, Error>?

    func present(_ payload: Data) async throws {
        self.payload = payload
        try await withTaskCancellationHandler {
            try await withCheckedThrowingContinuation { continuation in
                self.continuation = continuation
            }
        } onCancel: {
            Task { @MainActor [weak self] in self?.cancel() }
        }
    }

    func complete() {
        payload = nil
        continuation?.resume()
        continuation = nil
    }

    func cancel() {
        payload = nil
        continuation?.resume(throwing: CancellationError())
        continuation = nil
    }
}

@MainActor
final class LiveSharedFrameProvider {
    weak var coordinator: SilentSearchCoordinator?
    var frame: SharedMissionFrame? { coordinator?.sharedFrame }
    var role: RoverRole? { coordinator?.mission?.role }
}

@MainActor
final class LiveSilentSearchMotion: SilentSearchMotion {
    private let ar: ARSessionManager
    private let navigation: NavigationController
    private let frameProvider: LiveSharedFrameProvider
    private var adapter: NavigationSilentSearchMotion?
    private var adapterFrame: SharedMissionFrame?

    init(ar: ARSessionManager, navigation: NavigationController, frameProvider: LiveSharedFrameProvider) {
        self.ar = ar
        self.navigation = navigation
        self.frameProvider = frameProvider
    }

    var currentMissionPose: MissionPose? { currentAdapter()?.currentMissionPose }
    var currentMissionPath: [MissionPoint] { currentAdapter()?.currentMissionPath ?? [] }
    func navigate(to target: MissionPoint, policy: SilentSearchMotionPolicy) async -> SilentSearchMotionResult {
        guard let adapter = currentAdapter() else { return .failed(.noPose) }
        return await adapter.navigate(to: target, policy: policy)
    }
    func rotate(to heading: Double, tolerance: Double) async -> SilentSearchMotionResult {
        guard let adapter = currentAdapter() else { return .failed(.noPose) }
        return await adapter.rotate(to: heading, tolerance: tolerance)
    }
    func stop() async { await navigation.cancelAndWait() }

    private func currentAdapter() -> NavigationSilentSearchMotion? {
        guard let frame = frameProvider.frame else { return nil }
        if adapterFrame != frame {
            adapterFrame = frame
            adapter = NavigationSilentSearchMotion(navigation: navigation, ar: ar, frame: frame)
        }
        return adapter
    }
}

@MainActor
final class LiveSectorExplorer: SilentSearchExploring {
    private let ar: ARSessionManager
    private let frameProvider: LiveSharedFrameProvider
    private var explorer: SectorExplorer?
    private var explorerFrame: SharedMissionFrame?
    private let events: any SilentSearchEventSink
    private(set) var candidates: [SectorFrontierCandidate] = []

    init(ar: ARSessionManager, frameProvider: LiveSharedFrameProvider,
         events: any SilentSearchEventSink) {
        self.ar = ar
        self.frameProvider = frameProvider
        self.events = events
    }

    func nextCandidate() async -> SectorExplorerSelection {
        guard let frame = frameProvider.frame, let role = frameProvider.role, let start = ar.pose?.position else {
            return .exhausted
        }
        if explorerFrame != frame {
            explorerFrame = frame
            explorer = SectorExplorer(
                frame: frame, sector: role.searchSector,
                policy: SectorPathPolicy(sector: role.searchSector, frame: frame), events: events
            )
        }
        guard let explorer else { return .exhausted }
        let current = CostmapBuilder.buildWithObserved(from: ar.meshAnchors, center: start)
        explorer.rebuild(costmap: current.map, observed: current.observed, from: start)
        candidates = explorer.candidates
        return explorer.nextCandidate()
    }

    func markVisited(_ stableID: String) {
        explorer?.markVisited(stableID)
        candidates = explorer?.candidates ?? []
    }
    func markRejected(_ stableID: String, reason: SectorFrontierRejectionReason) {
        explorer?.markRejected(stableID, reason: reason)
        candidates = explorer?.candidates ?? []
    }
}

@MainActor
final class LiveTargetObserver: SilentSearchTargetObserving {
    private let ar: ARSessionManager
    private let clock: any SilentSearchClock
    private let detector: Detector
    private let frameProvider: LiveSharedFrameProvider
    private var targetLabel: String
    private let events: any SilentSearchEventSink
    private var source: ARRoverTargetObservationSource?
    private var sourceFrame: SharedMissionFrame?

    init(ar: ARSessionManager, clock: any SilentSearchClock, detector: Detector,
         frameProvider: LiveSharedFrameProvider, targetLabel: String,
         events: any SilentSearchEventSink) {
        self.ar = ar
        self.clock = clock
        self.detector = detector
        self.frameProvider = frameProvider
        self.targetLabel = targetLabel
        self.events = events
    }

    func observeNextFrame(until deadline: SilentSearchInstant) async -> SilentSearchTargetObservationResult {
        guard let frame = frameProvider.frame else { return .pending }
        if sourceFrame != frame {
            sourceFrame = frame
            source = ARRoverTargetObservationSource(
                sessionManager: ar, clock: clock, detector: detector,
                canonicalLabel: targetLabel, sharedFrame: frame, events: events
            )
        }
        return await source?.observeNextFrame(until: deadline) ?? .pending
    }

    func setTargetLabel(_ label: String) {
        guard targetLabel != label else { return }
        targetLabel = label
        source = nil
        sourceFrame = nil
    }
}

@MainActor
final class LiveSilentSearchSafetyMonitor: SilentSearchSafetyMonitoring {
    private let snapshots: () -> AsyncStream<ARFrameSnapshot>
    private let lifecycleEvents: () -> AsyncStream<ARSessionLifecycleEvent>
    private let navigationStates: () async -> AsyncStream<NavigationSafetyState>
    private let commandLinkReadiness: () async -> AsyncStream<RoverCommandLinkReadiness>

    init(ar: ARSessionManager, navigation: NavigationController, control: RoverControl) {
        snapshots = { ar.snapshots() }
        lifecycleEvents = { ar.lifecycleEvents() }
        navigationStates = { navigation.safetyStates() }
        commandLinkReadiness = { await control.commandLinkReadiness() }
    }

    init(
        navigationStates: @escaping () async -> AsyncStream<NavigationSafetyState>,
        commandLinkReadiness: @escaping () async -> AsyncStream<RoverCommandLinkReadiness>
    ) {
        snapshots = { AsyncStream { $0.finish() } }
        lifecycleEvents = { AsyncStream { $0.finish() } }
        self.navigationStates = navigationStates
        self.commandLinkReadiness = commandLinkReadiness
    }

    static func safetyEvent(for lifecycle: ARSessionLifecycleEvent) -> SilentSearchSafetyEvent? {
        switch lifecycle {
        case .reset, .failed: .generationChanged
        case let .interrupted(generation): .trackingLimited(generation: generation)
        case .interruptionEnded: nil
        }
    }

    func events() -> AsyncStream<SilentSearchSafetyEvent> {
        AsyncStream { continuation in
            var emittedTransportFailure = false
            var emittedReactiveSafetyFailure = false
            func yield(_ event: SilentSearchSafetyEvent) {
                if event == .transportFailed {
                    guard !emittedTransportFailure else { return }
                    emittedTransportFailure = true
                } else if event == .reactiveSafetyFailed {
                    guard !emittedReactiveSafetyFailure else { return }
                    emittedReactiveSafetyFailure = true
                }
                continuation.yield(event)
            }
            let snapshots = Task { @MainActor [snapshots] in
                for await snapshot in snapshots() {
                    switch snapshot.trackingQuality {
                    case .normal: yield(.trackingNormal(generation: snapshot.id.generation))
                    case .limited, .unavailable:
                        yield(.trackingLimited(generation: snapshot.id.generation))
                    }
                }
            }
            let lifecycle = Task { @MainActor [lifecycleEvents] in
                for await event in lifecycleEvents() {
                    if let event = Self.safetyEvent(for: event) { yield(event) }
                }
            }
            let navigation = Task { @MainActor [navigationStates] in
                for await state in await navigationStates() {
                    guard case let .failed(failure) = state else { continue }
                    switch failure {
                    case .commsLost, .commandFailed: yield(.transportFailed)
                    case .obstacle, .tipping, .stalled: yield(.reactiveSafetyFailed)
                    case .noPose, .noPath, .pathRejected, .trackingLost, .cancelled: break
                    }
                }
            }
            let commandLink = Task { @MainActor [commandLinkReadiness] in
                for await readiness in await commandLinkReadiness() where readiness == .unavailable {
                    yield(.transportFailed)
                }
            }
            continuation.onTermination = { @Sendable _ in
                snapshots.cancel()
                lifecycle.cancel()
                navigation.cancel()
                commandLink.cancel()
            }
        }
    }
}
