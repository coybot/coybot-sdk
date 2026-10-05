import Foundation

@MainActor
public protocol RoomSessionARManaging: AnyObject {
    var latestObservation: PoseObservation? { get }
    var observationHandler: ((PoseObservation) -> Void)? { get set }
    var onReset: ((UInt64) -> Void)? { get set }
    func resetTracking(generation: UInt64)
}

@MainActor
public final class RoomSessionCoordinator {
    public private(set) var sessionGeneration: UInt64

    private let motion: RoverMotion
    private let topology: RoomTopologyManaging
    private let arSession: RoomSessionARManaging
    private var hasStarted = false
    private var hasStartedTopology = false
    private var isSessionActive = false
    private var isResettingAR = false
    private var lastFrameSequence: UInt64 = 0
    private var lastTimestamp: TimeInterval = 0

    public init(motion: RoverMotion,
                topology: RoomTopologyManaging,
                arSession: RoomSessionARManaging,
                initialGeneration: UInt64 = 0) {
        self.motion = motion
        self.topology = topology
        self.arSession = arSession
        self.sessionGeneration = initialGeneration
        arSession.observationHandler = { [weak self] observation in
            self?.observe(observation)
        }
        arSession.onReset = { [weak self] generation in
            self?.handleARReset(generation: generation)
        }
    }

    public func startIfNeeded() async {
        guard !hasStarted else { return }
        hasStarted = true
        await resetSession()
    }

    @discardableResult
    public func resetSession() async -> UInt64 {
        hasStarted = true
        isSessionActive = false
        await motion.stopAndWait()
        topology.abandonTransition()
        sessionGeneration += 1
        hasStartedTopology = false
        lastFrameSequence = 0
        lastTimestamp = 0
        topology.reset(forSessionGeneration: sessionGeneration)
        isResettingAR = true
        arSession.resetTracking(generation: sessionGeneration)
        isResettingAR = false
        isSessionActive = true
        return sessionGeneration
    }

    private func handleARReset(generation: UInt64) {
        guard !isResettingAR else { return }
        hasStarted = true
        isSessionActive = false
        topology.abandonTransition()
        sessionGeneration = generation
        hasStartedTopology = false
        lastFrameSequence = 0
        lastTimestamp = 0
        topology.reset(forSessionGeneration: generation)
        isSessionActive = true
    }

    private func observe(_ observation: PoseObservation) {
        guard isSessionActive,
              observation.sessionGeneration == sessionGeneration,
              observation.frameSequence > lastFrameSequence,
              observation.timestamp > lastTimestamp else {
            return
        }
        lastFrameSequence = observation.frameSequence
        lastTimestamp = observation.timestamp
        guard observation.trackingQuality == .normal else { return }
        if hasStartedTopology {
            topology.ingestStablePose(observation.pose, sessionGeneration: sessionGeneration)
        } else {
            hasStartedTopology = true
            topology.startSession(generation: sessionGeneration, initialPose: observation.pose)
        }
    }
}
