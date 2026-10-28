import Foundation
import ARKit
import CoreMotion
import ImageIO
import RoverNav

public struct ARFrameID: Hashable, Sendable {
    public let generation: UInt64
    public let sequence: UInt64

    public init(generation: UInt64, sequence: UInt64) {
        self.generation = generation
        self.sequence = sequence
    }
}

public enum ARTrackingQuality: Equatable, Sendable {
    case unavailable
    case limited
    case normal
}

public enum ARDepthSource: String, Sendable {
    case smoothedSceneDepth, sceneDepth
}

/// One coherent AR frame: image, camera, depth and pose captured together, so consumers
/// that run asynchronously (Silent Search) never pair one frame's detections with another
/// frame's camera or depth.
public struct ARFrameSnapshot: @unchecked Sendable {
    public let id: ARFrameID
    public let timestamp: TimeInterval
    public let image: CVPixelBuffer
    public let cameraTransform: simd_float4x4
    public let cameraIntrinsics: simd_float3x3
    public let imageResolution: CGSize
    public let depthMap: CVPixelBuffer?
    public let depthConfidenceMap: CVPixelBuffer?
    public let depthSource: ARDepthSource?
    public let pose: Pose2D
    public let trackingQuality: ARTrackingQuality
    /// Which way up `image` is (see `ARSessionManager.imageOrientation`).
    public let imageOrientation: CGImagePropertyOrientation
    /// Floor estimate captured with this image for coherent non-LiDAR grounding.
    public let floorHeight: Float?

    public init(id: ARFrameID, timestamp: TimeInterval, image: CVPixelBuffer,
                cameraTransform: simd_float4x4, cameraIntrinsics: simd_float3x3,
                imageResolution: CGSize, depthMap: CVPixelBuffer?, pose: Pose2D,
                trackingQuality: ARTrackingQuality,
                depthConfidenceMap: CVPixelBuffer? = nil,
                depthSource: ARDepthSource? = nil,
                imageOrientation: CGImagePropertyOrientation = .right,
                floorHeight: Float? = nil) {
        self.id = id
        self.timestamp = timestamp
        self.image = image
        self.cameraTransform = cameraTransform
        self.cameraIntrinsics = cameraIntrinsics
        self.imageResolution = imageResolution
        self.depthMap = depthMap
        self.depthConfidenceMap = depthConfidenceMap
        self.depthSource = depthSource
        self.pose = pose
        self.trackingQuality = trackingQuality
        self.imageOrientation = imageOrientation
        self.floorHeight = floorHeight
    }
}

public enum ARSessionLifecycleEvent: Equatable, Sendable {
    case reset(generation: UInt64)
    case interrupted(generation: UInt64)
    case interruptionEnded(generation: UInt64)
    case failed(generation: UInt64, description: String)
}

/// Owns the ARKit session and is the rover's **primary odometry + mapping** source
/// (the WAVE ROVER base has no wheel encoders). Provides:
///   • 6DoF pose flattened to the nav ground plane (`Pose2D`)
///   • LiDAR scene mesh anchors → obstacles for the costmap
///   • live LiDAR depth → reactive obstacle avoidance
///
/// Nav frame convention (matches RoverNav.Geometry): x = ARKit world X, y = ARKit world Z.
@Observable
@MainActor
public final class ARSessionManager: NSObject, @preconcurrency ARSessionDelegate {
    public let session = ARSession()
    private let motionManager = CMMotionManager()
    private let relativeHeadingOperationQueue: OperationQueue = {
        let queue = OperationQueue()
        queue.name = "bot.coy.phrover.relative-heading"
        queue.maxConcurrentOperationCount = 1
        queue.qualityOfService = .userInteractive
        return queue
    }()

    public private(set) var pose: Pose2D?
    public var meshAnchors: [ARMeshAnchor] { meshStore.anchors }
    private var meshStore = MeshAnchorStore<ARMeshAnchor>()
    /// Nearest obstacle distance (m) in a forward cone from the latest depth frame.
    private(set) var latestDepthSafetySnapshot: DepthSafetySnapshot? {
        didSet {
            // Publication is synchronous with assignment, independent of pose
            // delivery. Snapshot callbacks tolerate unsubscribe/revoke reentrancy.
            let published = latestDepthSafetySnapshot
            for observer in Array(depthSafetyObservers.values) { observer(published) }
        }
    }
    @ObservationIgnored private var depthSafetyObservers: [UUID: (DepthSafetySnapshot?) -> Void] = [:]
    private struct WeakSearchClearance { weak var value: FollowSearchClearanceGrant? }
    @ObservationIgnored private var followSearchClearances: [WeakSearchClearance] = []

    func registerFollowSearchClearance(_ grant: FollowSearchClearanceGrant) {
        followSearchClearances.removeAll { $0.value == nil || $0.value?.revoked == true }
        followSearchClearances.append(.init(value: grant))
    }

    private func revokeFollowSearchClearances() {
        let grants = followSearchClearances.compactMap(\.value)
        followSearchClearances.removeAll()
        for grant in grants { grant.revoke(cause: .searchClearanceSourceInvalid, detail: "producer_invalidated") }
    }

    @ObservationIgnored private let depthProjection: @Sendable (DepthSafetyProcessor.Input) -> DepthSafetySnapshot
    @ObservationIgnored private lazy var depthProcessor = DepthSafetyProcessor(evaluate: depthProjection) { [weak self] snapshot in
        guard let self, self.acceptsObservations else { return }
        self.latestDepthSafetySnapshot = snapshot
        self.depthSnapshotVersion &+= 1
    }
    public private(set) var depthSnapshotVersion: UInt64 = 0
    public var depthSnapshotTimestamp: TimeInterval? { latestDepthSafetySnapshot?.timestamp }
    public private(set) var trackingState: ARCamera.TrackingState = .notAvailable
    public private(set) var trackingQuality: ARTrackingQuality = .unavailable
    /// Latest coherent frame, `nil` until the first accepted frame after a reset.
    public private(set) var latestSnapshot: ARFrameSnapshot?
    private(set) var sourceHighWater: FollowTurnSourceHighWater?
    private var turnEvidence = FollowTurnPoseEvidenceArchive()
    private var snapshotContinuations: [UUID: AsyncStream<ARFrameSnapshot>.Continuation] = [:]
    private var lifecycleContinuations: [UUID: AsyncStream<ARSessionLifecycleEvent>.Continuation] = [:]
    private var hasResetTracking = false

    /// Latest RGB frame, for `Detector` to run inference on.
    public private(set) var latestPixelBuffer: CVPixelBuffer?
    /// Monotonic camera-frame counter used to prevent perception from reusing a frame
    /// captured before a scan turn completed.
    public private(set) var frameSequence: UInt64 = 0
    public private(set) var latestObservation: PoseObservation?
    public private(set) var sessionGeneration: UInt64 = 0
    private var planningReadinessTracker = PlanningReadinessTracker()
    var planningReadiness: PlanningReadinessSnapshot { planningReadinessTracker.snapshot }
    public var observationHandler: ((PoseObservation) -> Void)?
    public var onReset: ((UInt64) -> Void)?
    private var acceptsObservations = false
    var acceptsFollowSearchClearance: Bool { acceptsObservations }
    private var lastObservationTimestamp: TimeInterval?
    nonisolated private let relativeHeadingStore = RelativeHeadingTrackerStore()
    private var loggedRelativeHeadingReliability: RelativeHeadingReliability = .unreliable(.notStarted)
    private let relativeHeadingTelemetry: RoomTopologyTelemetrySink
    /// Gyroscope-fused rover heading. Unlike ARKit world yaw, this does not jump when
    /// visual tracking relocalizes, so scan turns use it for relative-angle completion.
    func beginRelativeHeadingMeasurement() {
        relativeHeadingStore.beginMeasurement()
        loggedRelativeHeadingReliability = .unreliable(.notStarted)
        relativeHeadingTelemetry("relative_heading_measurement_started", [:])
    }

    func endRelativeHeadingMeasurement() {
        relativeHeadingStore.endMeasurement()
        loggedRelativeHeadingReliability = .unreliable(.notStarted)
        relativeHeadingTelemetry("relative_heading_measurement_ended", [:])
    }

    @discardableResult
    func ingestRelativeHeadingSample(_ sample: RelativeHeadingSample) -> Bool {
        guard let result = relativeHeadingStore.ingest(sample) else { return false }
        reportRelativeHeadingIngest(result, timestamp: sample.timestamp)
        return result.accepted
    }

    func relativeHeadingMeasurement(
        at timestamp: TimeInterval = ProcessInfo.processInfo.systemUptime
    ) -> RelativeHeadingMeasurement {
        let measurement = relativeHeadingStore.measurement(at: timestamp)
        if measurement.reliability != loggedRelativeHeadingReliability {
            reportRelativeHeadingReliabilityTransition(to: measurement.reliability)
        }
        return measurement
    }

    /// Scan turns only consume pose samples while ARKit has a reliable world transform.
    public var isTrackingNormal: Bool {
        latestObservation?.trackingQuality == .normal
    }
    /// Latest camera (intrinsics + transform + raw sensor `imageResolution`), retained so
    /// `unproject(normalizedPoint:)` can back-project a detection into the world.
    public private(set) var latestCamera: ARCamera?
    /// Latest LiDAR depth map (meters, aligned to `latestCamera.imageResolution`'s aspect).
    public private(set) var latestDepthMap: CVPixelBuffer?
    /// Which way up the raw camera buffer is, derived from gravity each frame. Everything
    /// that hands the image to a model (`Detector`, `FrameEncoder`) rotates by this so the
    /// model sees the scene upright, and `unproject` inverts the same rotation — so the
    /// phone can be mounted portrait, landscape either way, or upside down.
    public private(set) var imageOrientation: CGImagePropertyOrientation = .right
    /// World height (ARKit y) of the floor, from the lowest horizontal plane ARKit has
    /// found below the camera. What a phone with no LiDAR places people and objects on.
    public private(set) var floorHeight: Float?
    /// Whether this device produces a LiDAR depth map at all (Pro iPhones and iPads).
    public var hasLiDAR: Bool { ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) }
    /// Nearest-surface distance (m) in a fixed central window of the latest depth frame;
    /// `.infinity` when there is no depth. Image-space, not floor-aware, so navigation does
    /// not gate on it (`DepthSafetyEvaluator` does); kept as `FollowController`'s
    /// forward-obstacle signal via `LocalRoverDrive`.
    public private(set) var forwardClearance: Double = .infinity
    private var lastRejectedMeshLogAt = Date.distantPast

    public override init() {
        depthProjection = DepthSafetyProcessor.project
        relativeHeadingTelemetry = { event, fields in
            RuntimeFileLog.append(event, fields: fields)
        }
        super.init()
        session.delegate = self
    }

    init(relativeHeadingTelemetry: @escaping RoomTopologyTelemetrySink,
         depthProjection: @escaping @Sendable (DepthSafetyProcessor.Input) -> DepthSafetySnapshot = DepthSafetyProcessor.project) {
        self.depthProjection = depthProjection
        self.relativeHeadingTelemetry = relativeHeadingTelemetry
        super.init()
        session.delegate = self
    }

    private func invalidateRelativeHeadingMeasurement(
        reason: RelativeHeadingReliability.UnreliableReason
    ) {
        let timestamp = ProcessInfo.processInfo.systemUptime
        let result = relativeHeadingStore.invalidate(reason: reason, at: timestamp)
        guard result.wasActive else {
            loggedRelativeHeadingReliability = .unreliable(.notStarted)
            return
        }
        relativeHeadingTelemetry(
            "relative_heading_measurement_invalidated",
            fieldsForRelativeHeading(
                result.measurement,
                timestamp: timestamp
            )
        )
        reportRelativeHeadingReliabilityTransition(to: result.measurement.reliability)
    }

    private func reportRelativeHeadingIngest(
        _ result: RelativeHeadingIngestResult,
        timestamp: TimeInterval
    ) {
        let measurement = result.measurement
        guard measurement.reliability != loggedRelativeHeadingReliability else { return }
        relativeHeadingTelemetry(
            result.accepted ? "relative_heading_sample_accepted" : "relative_heading_sample_rejected",
            fieldsForRelativeHeading(measurement, timestamp: timestamp)
        )
        reportRelativeHeadingReliabilityTransition(to: measurement.reliability)
    }

    nonisolated private func ingestRelativeHeadingMotionSample(_ sample: RelativeHeadingSample) {
        guard let result = relativeHeadingStore.ingest(sample) else { return }
        Task { @MainActor [weak self] in
            self?.reportRelativeHeadingIngest(result, timestamp: sample.timestamp)
        }
    }

    nonisolated func makeRelativeHeadingMotionHandler() -> CMDeviceMotionHandler {
        { [weak self] motion, _ in
            guard let motion else { return }
            self?.ingestRelativeHeadingMotionSample(RelativeHeadingSample(
                timestamp: motion.timestamp,
                rotationRate: SIMD3(
                    motion.rotationRate.x,
                    motion.rotationRate.y,
                    motion.rotationRate.z
                ),
                gravity: SIMD3(
                    motion.gravity.x,
                    motion.gravity.y,
                    motion.gravity.z
                )
            ))
        }
    }

    private func reportRelativeHeadingReliabilityTransition(
        to reliability: RelativeHeadingReliability
    ) {
        let previous = Self.relativeHeadingReliabilityDescription(loggedRelativeHeadingReliability)
        let current = Self.relativeHeadingReliabilityDescription(reliability)
        loggedRelativeHeadingReliability = reliability
        relativeHeadingTelemetry("relative_heading_reliability_changed", [
            "from": previous,
            "to": current,
        ])
    }

    private func fieldsForRelativeHeading(
        _ measurement: RelativeHeadingMeasurement,
        timestamp: TimeInterval
    ) -> [String: String] {
        [
            "timestamp": String(format: "%.3f", timestamp),
            "accumulated_angle": String(format: "%.4f", measurement.accumulatedAngle),
            "sample_age": measurement.sampleAge.map { String(format: "%.3f", $0) } ?? "none",
            "reliability": Self.relativeHeadingReliabilityDescription(measurement.reliability),
        ]
    }

    private static func relativeHeadingReliabilityDescription(
        _ reliability: RelativeHeadingReliability
    ) -> String {
        switch reliability {
        case .reliable:
            return "reliable"
        case .unreliable(let reason):
            return reason.rawValue
        }
    }

    public func start() {
        resetTracking(generation: sessionGeneration + 1)
    }

    public func resetTracking(generation: UInt64) {
        resetTracking(generation: generation, runSession: true)
    }

    func resetTracking(generation: UInt64, runSession: Bool) {
        revokeFollowSearchClearances()
        depthProcessor.invalidate()
        hasResetTracking = true
        acceptsObservations = false
        sessionGeneration = generation
        planningReadinessTracker.reset(sessionGeneration: generation)
        pose = nil
        meshStore.removeAll()
        latestDepthSafetySnapshot = nil
        trackingState = .notAvailable
        latestPixelBuffer = nil
        frameSequence = 0
        latestObservation = nil
        lastObservationTimestamp = nil
        invalidateRelativeHeadingMeasurement(reason: .sessionGenerationChanged)
        latestCamera = nil
        latestDepthMap = nil
        latestSnapshot = nil
        sourceHighWater = nil
        turnEvidence.clear()
        trackingQuality = .unavailable
        forwardClearance = .infinity
        onReset?(generation)
        publishLifecycle(.reset(generation: generation))

        if motionManager.isDeviceMotionAvailable, !motionManager.isDeviceMotionActive {
            motionManager.deviceMotionUpdateInterval = RoverConfig.relativeHeadingUpdateInterval
            motionManager.startDeviceMotionUpdates(
                using: .xArbitraryZVertical,
                to: relativeHeadingOperationQueue,
                withHandler: makeRelativeHeadingMotionHandler()
            )
        }
        guard runSession else {
            acceptsObservations = true
            return
        }
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics.insert(.smoothedSceneDepth)
        }
        // Without LiDAR there is no depth map; the floor plane is what everything is
        // placed on instead (see `groundPoint(boundingBox:label:)`).
        if !hasLiDAR {
            config.planeDetection = [.horizontal]
        }
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        acceptsObservations = true
    }

    public func pause() {
        acceptsObservations = false
        revokeFollowSearchClearances()
        depthProcessor.invalidate()
        latestDepthSafetySnapshot = nil
        session.pause()
        motionManager.stopDeviceMotionUpdates()
    }

    /// Every accepted frame as one coherent snapshot. Each subscriber gets its own stream,
    /// buffering only the newest frame.
    public func snapshots() -> AsyncStream<ARFrameSnapshot> {
        let id = UUID()
        return AsyncStream(bufferingPolicy: .bufferingNewest(1)) { continuation in
            snapshotContinuations[id] = continuation
            continuation.onTermination = { @Sendable [weak self] _ in
                Task { @MainActor in self?.snapshotContinuations.removeValue(forKey: id) }
            }
        }
    }

    /// Keep compact pose ingress evidence even when image consumers skip frames.
    func turnPoseEvidence(from first: ARFrameID, through last: ARFrameID) -> [FollowTurnBurstPlanner.Sample]? {
        turnEvidence.evidence(from: first, through: last)
    }

    /// Session resets, interruptions and failures, unbuffered-loss-free per subscriber.
    public func lifecycleEvents() -> AsyncStream<ARSessionLifecycleEvent> {
        let id = UUID()
        return AsyncStream(bufferingPolicy: .unbounded) { continuation in
            lifecycleContinuations[id] = continuation
            continuation.onTermination = { @Sendable [weak self] _ in
                Task { @MainActor in self?.lifecycleContinuations.removeValue(forKey: id) }
            }
        }
    }

    private func publishLifecycle(_ event: ARSessionLifecycleEvent) {
        for continuation in lifecycleContinuations.values { continuation.yield(event) }
    }

    private func publishSnapshot(image: CVPixelBuffer, timestamp: TimeInterval,
                                 cameraTransform: simd_float4x4, intrinsics: simd_float3x3,
                                 imageResolution: CGSize, depthMap: CVPixelBuffer?,
                                 trackingQuality: ARTrackingQuality,
                                 depthConfidenceMap: CVPixelBuffer? = nil,
                                 depthSource: ARDepthSource? = nil) {
        guard let pose else { return }
        let snapshot = ARFrameSnapshot(
            id: ARFrameID(generation: sessionGeneration, sequence: frameSequence), timestamp: timestamp,
            image: image, cameraTransform: cameraTransform, cameraIntrinsics: intrinsics,
            imageResolution: imageResolution, depthMap: depthMap, pose: pose,
            trackingQuality: trackingQuality, depthConfidenceMap: depthConfidenceMap,
            depthSource: depthSource, imageOrientation: imageOrientation, floorHeight: floorHeight
        )
        latestSnapshot = snapshot
        turnEvidence.record(.init(snapshot: snapshot), at: ProcessInfo.processInfo.systemUptime)
        sourceHighWater = .init(frameID: snapshot.id, sourceTimestamp: timestamp.isFinite
            ? max(sourceHighWater?.sourceTimestamp ?? timestamp, timestamp) : sourceHighWater?.sourceTimestamp)
        self.trackingQuality = trackingQuality
        for continuation in snapshotContinuations.values { continuation.yield(snapshot) }
    }

    // MARK: - ARSessionDelegate

    func ingestDepthSafety(rawDepthMap: CVPixelBuffer,
                           intrinsics: simd_float3x3,
                           intrinsicsImageSize: CGSize? = nil,
                           cameraTransform: simd_float4x4,
                           timestamp: TimeInterval) {
        latestDepthSafetySnapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: rawDepthMap,
            intrinsics: intrinsics,
            intrinsicsImageSize: intrinsicsImageSize,
            cameraTransform: cameraTransform,
            timestamp: timestamp,
            calibration: RoverConfig.cameraMountCalibration,
            geometry: RoverConfig.collisionGeometry
        )
        depthSnapshotVersion &+= 1
    }

    func enqueueDepthSafety(rawDepthMap: CVPixelBuffer,
                            intrinsics: simd_float3x3,
                            intrinsicsImageSize: CGSize? = nil,
                            cameraTransform: simd_float4x4,
                            timestamp: TimeInterval) {
        depthProcessor.submit(.init(rawDepthMap: rawDepthMap, intrinsics: intrinsics,
            imageResolution: intrinsicsImageSize, cameraTransform: cameraTransform, timestamp: timestamp))
    }

    func observeDepthSafety(_ observer: @escaping (DepthSafetySnapshot?) -> Void) -> () -> Void {
        let id = UUID()
        depthSafetyObservers[id] = observer
        return { [weak self] in self?.depthSafetyObservers.removeValue(forKey: id) }
    }

    func depthSafetyObservation(
        for command: WheelCommand,
        at timestamp: TimeInterval = ProcessInfo.processInfo.systemUptime
    ) -> DepthSafetyObservation {
        guard let latestDepthSafetySnapshot else {
            return .unavailable(
                .missingRawDepth,
                sampleAge: .infinity,
                motionClass: DepthSafetyMotionClass.classify(command)
            )
        }
        return DepthSafetyEvaluator.evaluate(
            latestDepthSafetySnapshot,
            command: command,
            now: timestamp,
            operatorClearedRotation: FollowSearchRotationScope.authorized,
            boundedReadyMotion: ReadyMotionDepthScope.active
        )
    }

    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let trackingQuality = Self.trackingQuality(frame.camera.trackingState)
        let observation = PoseObservation(
            pose: Self.groundPose(from: frame.camera.transform),
            frameSequence: frameSequence &+ 1,
            timestamp: frame.timestamp,
            trackingQuality: trackingQuality,
            sessionGeneration: sessionGeneration
        )
        guard ingest(observation) else { return }
        trackingState = frame.camera.trackingState
        latestPixelBuffer = frame.capturedImage
        latestCamera = frame.camera
        let orientation = Self.imageOrientation(cameraTransform: frame.camera.transform, previous: imageOrientation)
        if orientation != imageOrientation {
            imageOrientation = orientation
            RuntimeFileLog.append("image_orientation", fields: ["exif": "\(orientation.rawValue)"])
        }
        if let rawDepth = frame.sceneDepth {
            enqueueDepthSafety(
                rawDepthMap: rawDepth.depthMap,
                intrinsics: frame.camera.intrinsics,
                intrinsicsImageSize: frame.camera.imageResolution,
                cameraTransform: frame.camera.transform,
                timestamp: frame.timestamp
            )
        } else {
            depthProcessor.invalidate()
            latestDepthSafetySnapshot = nil
        }
        let depth = frame.smoothedSceneDepth ?? frame.sceneDepth
        if let depth {
            forwardClearance = Self.forwardClearance(from: depth)
            latestDepthMap = depth.depthMap
        } else {
            forwardClearance = .infinity
            latestDepthMap = nil
        }
        publishSnapshot(image: frame.capturedImage, timestamp: frame.timestamp,
                        cameraTransform: frame.camera.transform, intrinsics: frame.camera.intrinsics,
                        imageResolution: frame.camera.imageResolution, depthMap: latestDepthMap,
                        trackingQuality: Self.arTrackingQuality(trackingQuality),
                        depthConfidenceMap: depth?.confidenceMap,
                        depthSource: frame.smoothedSceneDepth != nil ? .smoothedSceneDepth
                            : (depth != nil ? .sceneDepth : nil))
    }

    @discardableResult
    func ingest(_ observation: PoseObservation) -> Bool {
        guard acceptsObservations,
              observation.sessionGeneration == sessionGeneration,
              observation.frameSequence > frameSequence,
              lastObservationTimestamp.map({ observation.timestamp > $0 }) ?? true else {
            return false
        }
        let observedAt = ProcessInfo.processInfo.systemUptime
        for grant in followSearchClearances.compactMap(\.value) { grant.observeIngress(observation, now: observedAt) }
        followSearchClearances.removeAll { $0.value == nil || $0.value?.revoked == true }
        latestObservation = observation
        lastObservationTimestamp = observation.timestamp
        pose = observation.pose
        frameSequence = observation.frameSequence
        planningReadinessTracker.ingest(observation)
        observationHandler?(observation)
        return true
    }

    private func suspendObservations() {
        revokeFollowSearchClearances()
        depthProcessor.invalidate()
        turnEvidence.clear()
        acceptsObservations = false
        planningReadinessTracker.suspend()
        pose = nil
        latestDepthSafetySnapshot = nil
        trackingState = .notAvailable
        latestPixelBuffer = nil
        latestObservation = nil
        invalidateRelativeHeadingMeasurement(reason: .trackingInterrupted)
        latestCamera = nil
        latestDepthMap = nil
        forwardClearance = .infinity
        latestSnapshot = nil
        trackingQuality = .unavailable
    }

    public func sessionWasInterrupted(_ session: ARSession) {
        suspendObservations()
        publishLifecycle(.interrupted(generation: sessionGeneration))
    }

    public func sessionInterruptionEnded(_ session: ARSession) {
        acceptsObservations = true
        publishLifecycle(.interruptionEnded(generation: sessionGeneration))
    }

    public func session(_ session: ARSession, didFailWithError error: Error) {
        suspendObservations()
        publishLifecycle(.failed(generation: sessionGeneration, description: error.localizedDescription))
    }

    private static func arTrackingQuality(_ quality: PoseTrackingQuality) -> ARTrackingQuality {
        switch quality {
        case .normal: .normal
        case .limited: .limited
        case .unavailable: .unavailable
        }
    }

    // MARK: - Test seams (Silent Search)

    func resetForTesting() {
        resetTracking(generation: sessionGeneration + 1, runSession: false)
    }

    func ingestForTesting(image: CVPixelBuffer, timestamp: TimeInterval,
                          cameraTransform: simd_float4x4, intrinsics: simd_float3x3,
                          imageResolution: CGSize, depthMap: CVPixelBuffer?,
                          trackingQuality: ARTrackingQuality,
                          depthConfidenceMap: CVPixelBuffer? = nil,
                          depthSource: ARDepthSource? = nil,
                          imageOrientation: CGImagePropertyOrientation? = nil) {
        // Silent Search's fixtures feed a never-started manager at generation 0; outside
        // tests, observations are only accepted after `start()`/`resetTracking`.
        if !hasResetTracking { acceptsObservations = true }
        let poseQuality: PoseTrackingQuality = switch trackingQuality {
        case .normal: .normal
        case .limited: .limited
        case .unavailable: .unavailable
        }
        guard ingest(PoseObservation(
            pose: Self.groundPose(from: cameraTransform),
            frameSequence: frameSequence &+ 1,
            timestamp: timestamp,
            trackingQuality: poseQuality,
            sessionGeneration: sessionGeneration
        )) else { return }
        latestPixelBuffer = image
        if let imageOrientation { self.imageOrientation = imageOrientation }
        latestDepthMap = depthMap
        forwardClearance = depthMap.map(Self.forwardClearance(fromDepthMap:)) ?? .infinity
        publishSnapshot(image: image, timestamp: timestamp, cameraTransform: cameraTransform,
                        intrinsics: intrinsics, imageResolution: imageResolution, depthMap: depthMap,
                        trackingQuality: trackingQuality, depthConfidenceMap: depthConfidenceMap,
                        depthSource: depthSource)
    }

    func interruptionBeganForTesting() { sessionWasInterrupted(session) }
    func interruptionEndedForTesting() { sessionInterruptionEnded(session) }
    func failureForTesting(description: String) {
        suspendObservations()
        publishLifecycle(.failed(generation: sessionGeneration, description: description))
    }

    private static func trackingQuality(_ state: ARCamera.TrackingState) -> PoseTrackingQuality {
        switch state {
        case .normal: .normal
        case .limited: .limited
        case .notAvailable: .unavailable
        }
    }

    public func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        collectMesh(anchors)
        collectFloor(anchors)
    }
    public func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        collectMesh(anchors)
        collectFloor(anchors)
    }

    public func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        // Removal is valid even during tracking loss. It is not a new trusted
        // geometry observation and must not satisfy a recovery refresh gate.
        let removed = meshStore.remove(identifiers: Set(anchors.map(\.identifier)))
        if removed > 0 {
            RuntimeFileLog.append("nav_mesh_removed", fields: [
                "removed_count": String(removed),
                "remaining_count": String(meshAnchors.count),
                "session_generation": String(sessionGeneration)
            ])
        }
    }

    private func collectFloor(_ anchors: [ARAnchor]) {
        guard let cameraY = latestCamera?.transform.columns.3.y else { return }
        let heights = anchors
            .compactMap { $0 as? ARPlaneAnchor }
            .filter { $0.alignment == .horizontal }
            .map { $0.transform.columns.3.y }
            // A table top is a horizontal plane too; the floor is below the camera.
            .filter { $0 < cameraY - 0.05 }
        guard let lowest = heights.min() else { return }
        floorHeight = min(floorHeight ?? lowest, lowest)
    }

    private func collectMesh(_ anchors: [ARAnchor]) {
        let mesh = anchors.compactMap { $0 as? ARMeshAnchor }
        guard !mesh.isEmpty else { return }
        guard planningReadiness.isPoseReady else {
            let now = Date()
            if now.timeIntervalSince(lastRejectedMeshLogAt) >= 1 {
                lastRejectedMeshLogAt = now
                RuntimeFileLog.append("nav_mesh_update_rejected", fields: [
                    "reason": "tracking_unstable",
                    "pose_streak": "\(planningReadiness.normalObservationStreak)",
                    "session_generation": "\(sessionGeneration)"
                ], now: now)
            }
            return
        }
        meshStore.upsert(mesh)
        planningReadinessTracker.recordTrustedMeshUpdate()
    }

    private func trackingStateDescription(_ state: ARCamera.TrackingState) -> String {
        switch state {
        case .normal: return "normal"
        case .limited: return "limited"
        case .notAvailable: return "notAvailable"
        @unknown default: return "unknown"
        }
    }

    // MARK: - Object grounding

    /// Back-projects a point in normalized Vision coordinates (bottom-left origin, y-up)
    /// of the *upright* image — the frame `Detector` boxes and the cloud brain's image
    /// points are both in, because both see the buffer rotated by `imageOrientation` — to
    /// a world point on the nav plane, by sampling the aligned LiDAR depth map and
    /// unprojecting through the camera intrinsics. Returns `nil` if there's no
    /// camera/depth yet or the sampled depth is invalid.
    ///
    /// `ARCamera` has no public initializer, so the math is split into the static helpers
    /// `sensorPixel`/`sampleDepth`/`unprojectPoint` so it can be exercised in tests against
    /// synthetic intrinsics/transforms/depth.
    /// `unproject(normalizedPoint:)` against one coherent snapshot instead of the latest
    /// camera/depth, so a detection is only ever grounded with its own frame's geometry.
    nonisolated public static func unproject(normalizedPoint: CGPoint, in snapshot: ARFrameSnapshot) -> Vec2? {
        guard let depthMap = snapshot.depthMap,
              let depth = sampleDepth(depthMap, atVisionNormalizedPoint: normalizedPoint,
                                      imageSize: snapshot.imageResolution,
                                      orientation: snapshot.imageOrientation) else {
            return nil
        }
        return unprojectPoint(normalizedPoint, imageSize: snapshot.imageResolution,
                              orientation: snapshot.imageOrientation,
                              intrinsics: snapshot.cameraIntrinsics,
                              cameraTransform: snapshot.cameraTransform, depth: depth)
    }

    nonisolated public static func unproject(normalizedBoundingBox: CGRect, fallbackPoint: CGPoint,
                                           in snapshot: ARFrameSnapshot) -> Vec2? {
        guard let depthMap = snapshot.depthMap else { return nil }
        guard let sample = sampleObjectDepth(depthMap, inVisionNormalizedBoundingBox: normalizedBoundingBox,
            imageSize: snapshot.imageResolution, orientation: snapshot.imageOrientation) else {
            return unproject(normalizedPoint: fallbackPoint, in: snapshot)
        }
        return unprojectPoint(sample.point, imageSize: snapshot.imageResolution,
            orientation: snapshot.imageOrientation, intrinsics: snapshot.cameraIntrinsics,
            cameraTransform: snapshot.cameraTransform, depth: sample.depth)
    }

    public static func groundPoint(boundingBox box: CGRect, label: String,
                                   in snapshot: ARFrameSnapshot) -> Vec2? {
        if snapshot.depthMap != nil {
            return unproject(normalizedBoundingBox: box, fallbackPoint: CGPoint(x: box.midX, y: box.midY), in: snapshot)
        }
        if let floorHeight = snapshot.floorHeight {
            let ray = cameraRay(through: CGPoint(x: box.midX, y: box.minY), imageSize: snapshot.imageResolution,
                orientation: snapshot.imageOrientation, intrinsics: snapshot.cameraIntrinsics,
                cameraTransform: snapshot.cameraTransform)
            if let hit = floorIntersection(origin: ray.origin, direction: ray.direction, floorY: floorHeight) {
                return Vec2(Double(hit.x), Double(hit.z))
            }
        }
        guard MissionAgent.visualQueryMatchScore(query: "person", label: label) > 0,
              let depth = depthFromApparentHeight(box, imageSize: snapshot.imageResolution,
                orientation: snapshot.imageOrientation, intrinsics: snapshot.cameraIntrinsics) else { return nil }
        return unprojectPoint(CGPoint(x: box.midX, y: box.midY), imageSize: snapshot.imageResolution,
            orientation: snapshot.imageOrientation, intrinsics: snapshot.cameraIntrinsics,
            cameraTransform: snapshot.cameraTransform, depth: Float(depth))
    }

    public func unproject(normalizedPoint: CGPoint) -> Vec2? {
        guard let snapshot = latestSnapshot else { return nil }
        return Self.unproject(normalizedPoint: normalizedPoint, in: snapshot)
    }

    /// Where a detected object stands on the nav plane. With LiDAR, the nearest supported
    /// depth cluster across the box (`unproject(normalizedBoundingBox:fallbackPoint:)`).
    /// Without it, where the ray through the bottom-centre of its box — a person's feet —
    /// meets the floor ARKit has found; before a floor is found, and only for a person,
    /// from how tall they look against an assumed 1.7 m.
    public func groundPoint(boundingBox box: CGRect, label: String) -> Vec2? {
        if latestDepthMap != nil {
            return unproject(normalizedBoundingBox: box, fallbackPoint: CGPoint(x: box.midX, y: box.midY))
        }
        guard let camera = latestCamera else { return nil }
        let imageSize = camera.imageResolution
        if let floorHeight {
            let ray = Self.cameraRay(through: CGPoint(x: box.midX, y: box.minY), imageSize: imageSize,
                                     orientation: imageOrientation, intrinsics: camera.intrinsics,
                                     cameraTransform: camera.transform)
            if let hit = Self.floorIntersection(origin: ray.origin, direction: ray.direction, floorY: floorHeight) {
                return Vec2(Double(hit.x), Double(hit.z))
            }
        }
        guard MissionAgent.visualQueryMatchScore(query: "person", label: label) > 0,
              let depth = Self.depthFromApparentHeight(box, imageSize: imageSize,
                                                        orientation: imageOrientation,
                                                        intrinsics: camera.intrinsics)
        else { return nil }
        return Self.unprojectPoint(CGPoint(x: box.midX, y: box.midY), imageSize: imageSize,
                                   orientation: imageOrientation, intrinsics: camera.intrinsics,
                                   cameraTransform: camera.transform, depth: Float(depth))
    }

    /// Back-projects a detected object's bounds using several aligned depth samples.
    /// A single center pixel is unreliable on reflective or partially occluded objects,
    /// while the overall box median can be dominated by background around the object.
    /// Prefer the nearest supported depth cluster and keep the center point as a fallback
    /// when the detector did not provide usable bounds.
    public func unproject(normalizedBoundingBox: CGRect, fallbackPoint: CGPoint) -> Vec2? {
        guard let camera = latestCamera, let depthMap = latestDepthMap else { return nil }
        let imageSize = camera.imageResolution
        guard let sample = Self.sampleObjectDepth(
            depthMap,
            inVisionNormalizedBoundingBox: normalizedBoundingBox,
            imageSize: imageSize,
            orientation: imageOrientation
        ) else {
            return unproject(normalizedPoint: fallbackPoint)
        }
        RuntimeFileLog.append("object_depth_sampled", fields: [
            "depth": String(format: "%.2f", sample.depth),
            "support_count": "\(sample.supportCount)",
            "valid_count": "\(sample.validSampleCount)",
            "box_x": String(format: "%.2f", normalizedBoundingBox.minX),
            "box_y": String(format: "%.2f", normalizedBoundingBox.minY),
            "box_width": String(format: "%.2f", normalizedBoundingBox.width),
            "box_height": String(format: "%.2f", normalizedBoundingBox.height),
        ])
        return Self.unprojectPoint(
            sample.point,
            imageSize: imageSize,
            orientation: imageOrientation,
            intrinsics: camera.intrinsics,
            cameraTransform: camera.transform,
            depth: sample.depth
        )
    }

    /// The world ray through a Vision-normalized image point.
    nonisolated static func cameraRay(through point: CGPoint, imageSize: CGSize, orientation: CGImagePropertyOrientation,
                          intrinsics: simd_float3x3, cameraTransform: simd_float4x4)
        -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let sensor = sensorPixel(forVisionNormalizedPoint: point, imageSize: imageSize, orientation: orientation)
        let fx = Double(intrinsics[0][0]), fy = Double(intrinsics[1][1])
        let cx = Double(intrinsics[2][0]), cy = Double(intrinsics[2][1])
        // Camera space at depth 1: +X right, +Y up, looking down -Z (as in unprojectPoint).
        let local = SIMD4<Float>(Float((sensor.x - cx) / fx), Float(-(sensor.y - cy) / fy), -1, 0)
        let world = cameraTransform * local
        let origin = cameraTransform.columns.3
        return (SIMD3(origin.x, origin.y, origin.z), simd_normalize(SIMD3(world.x, world.y, world.z)))
    }

    /// Where a ray meets the horizontal plane y = `floorY`, if it points down at it and
    /// lands within `maxRange` metres (a nearly level ray "hits" the floor absurdly far
    /// away, which is worse than no answer).
    nonisolated static func floorIntersection(origin: SIMD3<Float>, direction: SIMD3<Float>, floorY: Float,
                                  maxRange: Float = 12) -> SIMD3<Float>? {
        guard direction.y < -1e-3, origin.y > floorY else { return nil }
        let t = (floorY - origin.y) / direction.y
        let hit = origin + t * direction
        let horizontal = simd_length(SIMD2(hit.x - origin.x, hit.z - origin.z))
        return horizontal <= maxRange ? hit : nil
    }

    /// Range (m, along the optical axis) to a person from how tall their box is, assuming
    /// `assumedHeight`. Rough — a child or someone sitting reads far — so it is only the
    /// fallback until ARKit has a floor. `nil` when the box touches the top or bottom of
    /// the frame, since a cut-off person looks shorter and so farther than they are.
    nonisolated static func depthFromApparentHeight(_ box: CGRect, imageSize: CGSize, orientation: CGImagePropertyOrientation,
                                        intrinsics: simd_float3x3, assumedHeight: Double = 1.7) -> Double? {
        guard box.minY > 0.02, box.maxY < 0.98 else { return nil }
        let top = sensorPixel(forVisionNormalizedPoint: CGPoint(x: box.midX, y: box.maxY),
                              imageSize: imageSize, orientation: orientation)
        let bottom = sensorPixel(forVisionNormalizedPoint: CGPoint(x: box.midX, y: box.minY),
                                 imageSize: imageSize, orientation: orientation)
        let pixels = hypot(top.x - bottom.x, top.y - bottom.y)
        guard pixels > 1 else { return nil }
        let focal = Double(intrinsics[0][0] + intrinsics[1][1]) / 2
        return focal * assumedHeight / pixels
    }

    /// Undoes the rotation a Vision request (or `CIImage.oriented`) applied for
    /// `orientation`, landing back in the raw sensor pixel space (top-left origin)
    /// `intrinsics`/depth are calibrated against.
    nonisolated static func sensorPixel(forVisionNormalizedPoint p: CGPoint, imageSize: CGSize,
                            orientation: CGImagePropertyOrientation = .right) -> CGPoint {
        // Upright image, top-left origin, normalized.
        let u = p.x, v = 1 - p.y
        let raw: (u: Double, v: Double)
        switch orientation {
        case .up: raw = (u, v)
        case .left: raw = (1 - v, u)        // displayed after a 90° counter-clockwise turn
        case .down: raw = (1 - u, 1 - v)    // 180°
        default: raw = (v, 1 - u)           // .right, 90° clockwise; mirrored never occurs
        }
        return CGPoint(x: raw.u * imageSize.width, y: raw.v * imageSize.height)
    }

    /// Minimum lead the gravity component on a new image axis needs over the current one
    /// before the orientation flips. About 12° either side of 45°, so a mount that sits
    /// near the diagonal, or a chassis rocking over a threshold, doesn't flicker.
    nonisolated static let orientationHysteresis = 0.2

    /// Which EXIF orientation makes the raw buffer upright, from where world-up falls in
    /// the camera's image plane. ARKit's camera +X runs along the raw image toward its right
    /// edge and +Y toward its top, so world-up's components on those axes say which raw
    /// edge is up: left edge up is the ordinary portrait hold (`.right`), top edge up is
    /// landscape with the raw buffer already upright (`.up`). Holds `previous` when the
    /// camera looks nearly straight up or down, where the roll is undefined.
    nonisolated static func imageOrientation(cameraTransform t: simd_float4x4,
                                 previous: CGImagePropertyOrientation) -> CGImagePropertyOrientation {
        let upX = Double(t.columns.0.y), upY = Double(t.columns.1.y)
        func lift(_ o: CGImagePropertyOrientation) -> Double {
            switch o {
            case .up: return upY
            case .left: return upX
            case .down: return -upY
            default: return -upX
            }
        }
        let best = [CGImagePropertyOrientation.right, .up, .left, .down].max { lift($0) < lift($1) }!
        guard best != previous, lift(best) > 0.3,
              lift(best) > lift(previous) + orientationHysteresis else { return previous }
        return best
    }

    /// Fraction of the depth map's smaller side sampled either side of the point. Small
    /// enough to stay on the object, large enough to span the gaps a single pixel falls
    /// through (see `sampleDepth`).
    nonisolated static let depthSampleWindowFraction = 0.03

    /// Samples the LiDAR depth map (meters) at the raw-sensor-space point corresponding to
    /// a Vision-normalized point. The depth map is lower-res than the color camera but
    /// aligned to the same field of view, so the fractional position carries over directly.
    ///
    /// Reads a small window and takes its 25th percentile rather than the single pixel
    /// under the point. A detection's centre is not guaranteed to land on the object: on a
    /// standing person it routinely falls between torso and arm and reads the wall metres
    /// behind them, which puts the unprojected world point well past the target. Preferring
    /// the nearer quartile of a local window biases toward the surface facing the camera —
    /// the same reasoning as `forwardClearance(fromDepthMap:)`'s low percentile, over a
    /// local window instead of the whole driving corridor. Erring near also errs safe: a
    /// goal short of the object is a stop, one behind it is a collision.
    nonisolated static func sampleDepth(_ depthMap: CVPixelBuffer, atVisionNormalizedPoint p: CGPoint, imageSize: CGSize,
                            orientation: CGImagePropertyOrientation = .right) -> Float? {
        guard imageSize.width > 0, imageSize.height > 0 else { return nil }
        let sensor = sensorPixel(forVisionNormalizedPoint: p, imageSize: imageSize, orientation: orientation)

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        let dw = CVPixelBufferGetWidth(depthMap), dh = CVPixelBufferGetHeight(depthMap)
        guard let base = CVPixelBufferGetBaseAddress(depthMap), dw > 0, dh > 0 else { return nil }
        let stride = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
        let ptr = base.assumingMemoryBound(to: Float32.self)
        let dx = min(dw - 1, max(0, Int((sensor.x / imageSize.width) * Double(dw))))
        let dy = min(dh - 1, max(0, Int((sensor.y / imageSize.height) * Double(dh))))

        let half = max(1, Int(Double(min(dw, dh)) * depthSampleWindowFraction))
        var samples: [Float] = []
        samples.reserveCapacity((2 * half + 1) * (2 * half + 1))
        for y in max(0, dy - half)...min(dh - 1, dy + half) {
            for x in max(0, dx - half)...min(dw - 1, dx + half) {
                let d = ptr[y * stride + x]
                if d > 0.05 && d.isFinite { samples.append(d) }
            }
        }
        guard !samples.isEmpty else { return nil }
        samples.sort()
        let index = min(samples.count - 1, max(0, Int(Double(samples.count - 1) * 0.25)))
        return samples[index]
    }

    struct ObjectDepthSample: Equatable {
        let point: CGPoint
        let depth: Float
        let supportCount: Int
        let validSampleCount: Int
    }

    nonisolated static func objectDepthSamplePoints(in boundingBox: CGRect) -> [CGPoint] {
        let unitBox = boundingBox.standardized.intersection(CGRect(x: 0, y: 0, width: 1, height: 1))
        guard !unitBox.isNull, unitBox.width > 0, unitBox.height > 0 else { return [] }
        let fractions: [CGFloat] = [0.25, 0.5, 0.75]
        return fractions.flatMap { yFraction in
            fractions.map { xFraction in
                CGPoint(
                    x: unitBox.minX + unitBox.width * xFraction,
                    y: unitBox.minY + unitBox.height * yFraction
                )
            }
        }
    }

    nonisolated static func sampleObjectDepth(_ depthMap: CVPixelBuffer,
                                  inVisionNormalizedBoundingBox boundingBox: CGRect,
                                  imageSize: CGSize,
                                  orientation: CGImagePropertyOrientation = .right) -> ObjectDepthSample? {
        let samples = objectDepthSamplePoints(in: boundingBox).compactMap { point -> ObjectDepthSample? in
            guard let depth = sampleDepth(
                depthMap,
                atVisionNormalizedPoint: point,
                imageSize: imageSize,
                orientation: orientation
            ) else { return nil }
            return ObjectDepthSample(
                point: point,
                depth: depth,
                supportCount: 1,
                validSampleCount: 1
            )
        }
        guard !samples.isEmpty else { return nil }
        return nearestSupportedObjectDepthSample(from: samples)
    }

    nonisolated static func nearestSupportedObjectDepthSample(
        from samples: [ObjectDepthSample],
        clusterTolerance: Float = 0.35
    ) -> ObjectDepthSample? {
        guard !samples.isEmpty else { return nil }
        let sorted = samples.sorted { $0.depth < $1.depth }
        let minimumSupport = min(2, sorted.count)
        var clusters: [[ObjectDepthSample]] = []

        for sample in sorted {
            if let lastDepth = clusters.last?.last?.depth,
               sample.depth - lastDepth <= clusterTolerance {
                clusters[clusters.count - 1].append(sample)
            } else {
                clusters.append([sample])
            }
        }

        let cluster = clusters.first(where: { $0.count >= minimumSupport }) ?? sorted
        let selected = cluster[cluster.count / 2]
        return ObjectDepthSample(
            point: selected.point,
            depth: selected.depth,
            supportCount: cluster.count,
            validSampleCount: sorted.count
        )
    }

    /// Back-projects a raw-sensor-space point at a known depth through the camera
    /// intrinsics and pose into a world-plane point (matches `groundPose`'s x/world-x,
    /// y/world-z convention). Pure math — testable with synthetic intrinsics/transform.
    nonisolated static func unprojectPoint(_ visionNormalizedPoint: CGPoint, imageSize: CGSize,
                               orientation: CGImagePropertyOrientation = .right,
                               intrinsics: simd_float3x3, cameraTransform: simd_float4x4, depth: Float) -> Vec2 {
        let sensor = sensorPixel(forVisionNormalizedPoint: visionNormalizedPoint, imageSize: imageSize,
                                 orientation: orientation)
        let fx = Double(intrinsics[0][0]), fy = Double(intrinsics[1][1])
        let cx = Double(intrinsics[2][0]), cy = Double(intrinsics[2][1])
        let d = Double(depth)

        // Camera space: +X right, +Y up, camera looks down -Z (same convention as groundPose).
        let xCam = (sensor.x - cx) / fx * d
        let yCam = -(sensor.y - cy) / fy * d
        let zCam = -d

        let world = cameraTransform * SIMD4<Float>(Float(xCam), Float(yCam), Float(zCam), 1)
        return Vec2(Double(world.x), Double(world.z))
    }

    // MARK: - Geometry helpers

    /// Flatten an ARKit camera transform to a ground-plane pose. Camera looks down -Z.
    nonisolated static func groundPose(from t: simd_float4x4) -> Pose2D {
        let p = t.columns.3
        // Device forward in world = -(third basis column).
        let fwd = -SIMD3<Float>(t.columns.2.x, t.columns.2.y, t.columns.2.z)
        let yaw = atan2(Double(fwd.z), Double(fwd.x))
        return Pose2D(position: Vec2(Double(p.x), Double(p.z)), yaw: yaw)
    }

    static func forwardClearance(from depth: ARDepthData) -> Double {
        forwardClearance(fromDepthMap: depth.depthMap)
    }

    /// Robust near depth (m) sampled from the driving corridor of the LiDAR depth map.
    nonisolated static func forwardClearance(fromDepthMap map: CVPixelBuffer) -> Double {
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        let w = CVPixelBufferGetWidth(map), h = CVPixelBufferGetHeight(map)
        guard let base = CVPixelBufferGetBaseAddress(map), w > 0, h > 0 else { return .infinity }
        let rowBytes = CVPixelBufferGetBytesPerRow(map)
        let ptr = base.assumingMemoryBound(to: Float32.self)
        let stride = rowBytes / MemoryLayout<Float32>.size

        var depths: [Float] = []
        depths.reserveCapacity((h / 3) * (w / 2))
        // Sample a wider central driving corridor. A wall slightly off-center in the
        // mounted phone's view still needs to stop the rover before contact.
        for y in (h / 3)..<(h * 2 / 3) {
            for x in (w / 4)..<(w * 3 / 4) {
                let d = ptr[y * stride + x]
                if d > 0.05 && d.isFinite { depths.append(d) }
            }
        }
        guard !depths.isEmpty else { return .infinity }
        depths.sort()
        let index = min(depths.count - 1, max(0, Int(Double(depths.count - 1) * 0.10)))
        return Double(depths[index])
    }

}

extension ARSessionManager: RoomSessionARManaging {}
