import ARKit
import Foundation

@MainActor
public final class SilentSearchDeviceEnvironment: SilentSearchReadinessChecking {
    private static let probeFreshness: TimeInterval = 2

    public var targetLabel: String
    private let tracking: () -> ARTrackingQuality
    private let generation: () -> UInt64
    private let lidarSupported: () -> Bool
    private let detectorLoaded: () -> Bool
    private let detectorLabels: () -> Set<String>
    private let now: () -> Date
    private let probeLink: () async throws -> Void
    private var lastSuccessfulProbe: Date?

    public init(ar: ARSessionManager, detector: Detector, control: RoverControl, targetLabel: String) {
        self.targetLabel = targetLabel
        tracking = { ar.trackingQuality }
        generation = { ar.sessionGeneration }
        lidarSupported = {
            ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) &&
                ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
        }
        detectorLoaded = { detector.isLoaded }
        detectorLabels = { detector.supportedCanonicalLabels }
        now = Date.init
        probeLink = { try await control.probeLink() }
    }

    init(
        targetLabel: String,
        tracking: @escaping () -> ARTrackingQuality,
        generation: @escaping () -> UInt64,
        lidarSupported: @escaping () -> Bool,
        detectorLoaded: @escaping () -> Bool,
        detectorLabels: @escaping () -> Set<String>,
        now: @escaping () -> Date,
        probeLink: @escaping () async throws -> Void
    ) {
        self.targetLabel = targetLabel
        self.tracking = tracking
        self.generation = generation
        self.lidarSupported = lidarSupported
        self.detectorLoaded = detectorLoaded
        self.detectorLabels = detectorLabels
        self.now = now
        self.probeLink = probeLink
    }

    public var snapshot: SilentSearchReadiness {
        let currentGeneration = generation()
        let trackingReadiness: SilentSearchTrackingReadiness = switch tracking() {
        case .unavailable: .unavailable
        case .limited: .limited(sessionGeneration: currentGeneration)
        case .normal: .normal(sessionGeneration: currentGeneration)
        }
        let linkFresh = lastSuccessfulProbe.map {
            now().timeIntervalSince($0) >= 0 && now().timeIntervalSince($0) <= Self.probeFreshness
        } ?? false
        return SilentSearchReadiness(
            tracking: trackingReadiness,
            lidarAvailable: lidarSupported(),
            generationValid: currentGeneration > 0,
            detectorLoaded: detectorLoaded(),
            detectorLabelAvailable: detectorLabels().contains(targetLabel),
            commandLinkAvailable: linkFresh
        )
    }

    public var supportedTargetLabels: [String] { detectorLabels().sorted() }

    public func refreshCommandLink() async {
        do {
            try await probeLink()
            lastSuccessfulProbe = now()
        } catch {
            lastSuccessfulProbe = nil
        }
    }
}
