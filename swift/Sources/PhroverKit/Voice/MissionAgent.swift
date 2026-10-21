import Foundation
import CoreGraphics
import UIKit
import RoverNav

public enum NavigationGoalRejection: String, Equatable, Sendable, CaseIterable {
    case missingPose = "missing_pose"
    case trackingUnstable = "tracking_unstable"
    case startOutsideMap = "start_outside_map"
    case goalOutsideMap = "goal_outside_map"
    case startBlocked = "start_blocked"
    case goalBlocked = "goal_blocked"
    case noConnectedPath = "no_connected_path"
}

public enum PlanningRecoveryOutcome: String, Equatable, Sendable {
    case ready
    case trackingTimeout = "tracking_timeout"
    case meshTimeout = "mesh_timeout"
    case cancelled
    case sessionGenerationChanged = "session_generation_changed"
}

public enum PlanningContextRequirement: String, Equatable, Sendable {
    case stablePose = "stable_pose"
    case refreshedTrustedMesh = "refreshed_trusted_mesh"
}

public struct NavigationGoalAssessment: Equatable, Sendable {
    public let goal: Vec2
    public let isReachable: Bool
    public let pathDistance: Double
    public let rejectionReason: NavigationGoalRejection?

    public init(goal: Vec2,
                isReachable: Bool,
                pathDistance: Double,
                rejectionReason: NavigationGoalRejection? = nil) {
        self.goal = goal
        self.isReachable = isReachable
        self.pathDistance = pathDistance
        self.rejectionReason = rejectionReason
    }
}

/// Motion surface `MissionAgent` drives. A separate protocol from the concrete
/// `NavigationController` (rather than depending on it directly) so the mission loop can be
/// tested without a live ARKit session.
@MainActor
public protocol RoverMotion: AnyObject {
    var state: NavigationController.State { get }
    func navigate(to goal: Vec2)
    func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double)
    func rotate(by angle: Double) async
    func rotateForScan(by angle: Double) async
    func assessGoal(_ goal: Vec2) -> NavigationGoalAssessment
    func preparePlanningContext(
        requiring requirement: PlanningContextRequirement
    ) async -> PlanningRecoveryOutcome
    func recoverPlanningContext() async -> PlanningRecoveryOutcome
    func stopAndWait() async
    func cancel()
    func stopAndConfirm() async throws
}

extension RoverMotion {
    public func stopAndConfirm() async throws { cancel() }
    public func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) {
        navigate(to: goal)
    }

    public func rotateForScan(by angle: Double) async {
        await rotate(by: angle)
    }

    public func assessGoal(_ goal: Vec2) -> NavigationGoalAssessment {
        NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: 0)
    }

    public func preparePlanningContext(
        requiring requirement: PlanningContextRequirement
    ) async -> PlanningRecoveryOutcome { .ready }

    public func recoverPlanningContext() async -> PlanningRecoveryOutcome {
        await preparePlanningContext(requiring: .refreshedTrustedMesh)
    }

    public func stopAndWait() async {
        cancel()
    }
}

extension NavigationController: RoverMotion {}

/// Perception surface `MissionAgent` reads. A separate protocol from `ARSessionManager` +
/// `Detector` so the mission loop can be tested by scripting what's "visible" without a
/// live ARKit session or the bundled CoreML model.
@MainActor
public protocol RoverPerception: AnyObject {
    var pose: Pose2D? { get }
    var latestObservation: PoseObservation? { get }
    /// Monotonic camera-frame sequence, when the perception source can provide one.
    /// `nil` keeps non-camera and test implementations backward compatible.
    var frameSequence: UInt64? { get }
    func detectObjects() -> [PerceivedObject]
    func refreshDetections() async
    /// Project a point produced by the latest local detection receipt.
    func unprojectDetectedObject(normalizedPoint: CGPoint) -> Vec2?
    func unproject(normalizedPoint: CGPoint) -> Vec2?
    func unproject(object: PerceivedObject) -> Vec2?
    func capturedFrameJPEG() -> Data?
    /// Resolve a free-text description ("the green chair") to a normalized point in the
    /// current view, or `nil` if nothing matches. Has a default (substring-match)
    /// implementation below; conform your own for open-vocabulary/attribute grounding.
    func groundObject(query: String) -> CGPoint?
    /// Where a detected object stands on the nav plane. The default unprojects its
    /// centre point through depth; a phone without LiDAR has no depth and places it by
    /// its base on the floor instead (`ARPerceptionSource`).
    func groundPoint(of object: PerceivedObject) -> Vec2?
    /// Openings into unexplored space (frontier detection over the scene mesh). Default
    /// implementation returns [] for perception sources with no mapping capability.
    func explorationFrontiers() -> [Frontier]
}

extension RoverPerception {
    public var latestObservation: PoseObservation? { nil }
    public var frameSequence: UInt64? { nil }

    public func refreshDetections() async {}

    public func unprojectDetectedObject(normalizedPoint: CGPoint) -> Vec2? {
        unproject(normalizedPoint: normalizedPoint)
    }

    public func unproject(object: PerceivedObject) -> Vec2? {
        unproject(normalizedPoint: object.normalizedPoint)
    }

    /// Default grounding: case-insensitive substring match against `detectObjects()`
    /// labels, picking the highest-confidence match. No attribute/color understanding —
    /// "green chair" matches the same as "chair". Override for anything smarter.
    public func groundObject(query: String) -> CGPoint? {
        let q = query.lowercased()
        return detectObjects()
            .filter { q.contains($0.label.lowercased()) || $0.label.lowercased().contains(q) }
            .max { $0.confidence < $1.confidence }?
            .normalizedPoint
    }

    public func explorationFrontiers() -> [Frontier] { [] }

    public func groundPoint(of object: PerceivedObject) -> Vec2? {
        unproject(normalizedPoint: object.normalizedPoint)
    }
}

/// Default `RoverPerception`: on-device COCO detection over the live ARKit frame.
@MainActor
public final class ARPerceptionSource: RoverPerception {
    private let ar: ARSessionManager
    private let detector: Detector?
    private let colorAnalyzer = LocalObjectColorAnalyzer()
    private var detectionFrame: ARFrameSnapshot?
    private var detections: [Detector.Detection] = []

    public func refreshDetections() async {
        guard let detector, let snapshot = ar.latestSnapshot else {
            detectionFrame = nil
            detections = []
            return
        }
        guard let evaluation = await detector.evaluateForMissionAsync(snapshot),
              !Task.isCancelled, evaluation.snapshot.id.generation == ar.sessionGeneration else {
            detectionFrame = nil
            detections = []
            return
        }
        detectionFrame = evaluation.snapshot
        detections = evaluation.receipt.frame.detections
    }

    private var freshDetectionFrame: ARFrameSnapshot? {
        guard let frame = detectionFrame, frame.id.generation == ar.sessionGeneration,
              ar.trackingQuality == .normal, frame.trackingQuality == .normal else { return nil }
        let age = ProcessInfo.processInfo.systemUptime - frame.timestamp
        return age.isFinite && age >= 0 && age <= 0.5 ? frame : nil
    }

    public init(ar: ARSessionManager, detector: Detector?) {
        self.ar = ar
        self.detector = detector
    }

    public var pose: Pose2D? { ar.pose }
    public var latestObservation: PoseObservation? { ar.latestObservation }
    public var frameSequence: UInt64? { ar.frameSequence }

    public func detectObjects() -> [PerceivedObject] {
        guard let frame = freshDetectionFrame else { return [] }
        return detections.map { detection in
            PerceivedObject(
                label: detection.label,
                confidence: detection.confidence,
                normalizedPoint: CGPoint(
                    x: detection.boundingBox.midX,
                    y: detection.boundingBox.midY
                ),
                normalizedBoundingBox: detection.boundingBox,
                colorEvidence: colorAnalyzer.analyze(
                    pixelBuffer: frame.image,
                    normalizedBoundingBox: rawBufferBox(detection.boundingBox, in: frame)
                )
            )
        }
    }

    public func unproject(normalizedPoint: CGPoint) -> Vec2? {
        ar.unproject(normalizedPoint: normalizedPoint)
    }

    public func unprojectDetectedObject(normalizedPoint: CGPoint) -> Vec2? {
        guard let frame = freshDetectionFrame else { return nil }
        return ARSessionManager.unproject(normalizedPoint: normalizedPoint, in: frame)
    }

    public func groundPoint(of object: PerceivedObject) -> Vec2? {
        guard let frame = freshDetectionFrame else { return nil }
        guard let box = object.normalizedBoundingBox else {
            return ARSessionManager.unproject(normalizedPoint: object.normalizedPoint, in: frame)
        }
        return ARSessionManager.groundPoint(boundingBox: box, label: object.label, in: frame)
    }

    public func unproject(object: PerceivedObject) -> Vec2? {
        guard let frame = freshDetectionFrame else { return nil }
        guard let box = object.normalizedBoundingBox else {
            return ARSessionManager.unproject(normalizedPoint: object.normalizedPoint, in: frame)
        }
        return ARSessionManager.unproject(normalizedBoundingBox: box,
                                         fallbackPoint: object.normalizedPoint, in: frame)
    }

    /// Detector boxes are in the upright frame (the buffer rotated by
    /// `ar.imageOrientation`); `LocalObjectColorAnalyzer` reads the raw buffer, so map the
    /// box back into raw-buffer Vision coordinates (bottom-left origin) first.
    private func rawBufferBox(_ box: CGRect, in frame: ARFrameSnapshot) -> CGRect {
        let unit = CGSize(width: 1, height: 1)
        let a = ARSessionManager.sensorPixel(forVisionNormalizedPoint: CGPoint(x: box.minX, y: box.minY),
                                             imageSize: unit, orientation: frame.imageOrientation)
        let b = ARSessionManager.sensorPixel(forVisionNormalizedPoint: CGPoint(x: box.maxX, y: box.maxY),
                                             imageSize: unit, orientation: frame.imageOrientation)
        // sensorPixel is top-left origin; flip back to Vision's bottom-left origin.
        return CGRect(x: min(a.x, b.x), y: 1 - max(a.y, b.y),
                      width: abs(a.x - b.x), height: abs(a.y - b.y))
    }

    public func capturedFrameJPEG() -> Data? {
        FrameEncoder.jpeg(ar.latestPixelBuffer, orientation: ar.imageOrientation)
    }

    public func explorationFrontiers() -> [Frontier] {
        guard let center = ar.pose?.position else { return [] }
        let (map, observed) = CostmapBuilder.buildWithObserved(from: ar.meshAnchors, center: center)
        return FrontierFinder.candidates(costmap: map, observed: observed)
    }
}

/// Voice surface `MissionAgent` drives. A separate protocol from `SpeechOut`/`SpeechIn` so
/// the mission loop can be tested by scripting operator replies without real audio I/O.
@MainActor
public protocol RoverVoice: AnyObject {
    func speak(_ text: String)
    /// Ask a question and wait for a reply, honoring `timeout`. `nil` on timeout or no
    /// usable reply — the caller proceeds best-effort.
    func ask(_ question: String, timeout: TimeInterval) async -> String?
}

/// Battery surface `MissionAgent` reads — optional (nil if the platform doesn't expose
/// one). A separate protocol from `RoverPerception` since it's orthogonal to vision/nav:
/// self-model ("how much runway do I have left") doesn't need ARKit/detector plumbing to
/// fake, and a brain that ignores battery entirely still works with `percent` always nil.
@MainActor
public protocol RoverBattery: AnyObject {
    var percent: Double? { get }
}

/// Default `RoverBattery`: the iPhone's own battery level — the phone is the rover's
/// brain, so its battery is what capability "self-model and calibrated uncertainty"
/// reasons about (the WAVE ROVER chassis itself exposes no battery telemetry today).
@MainActor
public final class DeviceBattery: RoverBattery {
    public init() {
        UIDevice.current.isBatteryMonitoringEnabled = true
    }

    public var percent: Double? {
        let level = UIDevice.current.batteryLevel
        return level < 0 ? nil : Double(level) * 100.0
    }
}

/// Team-mesh surface `MissionAgent` reads/writes — optional (nil for a solo mission). A
/// separate protocol from the others since it's about *other rovers*, not this one's own
/// sensing/motion/speech: capability "collaboration" (shared intent, market allocation,
/// survivor robustness) is the brain's own reasoning over `currentTeamContext()`, using
/// `broadcastClaim` to act on it — `MissionAgent` only relays, same as `RoverBattery`.
@MainActor
public protocol RoverTeamRadio: AnyObject {
    /// Current known team state (rooms + who's claimed/alive), or `nil` if unavailable
    /// this tick (e.g. mesh not yet joined).
    func currentTeamContext() -> TeamContext?
    /// Announce a claim on a room/area to the rest of the team.
    func broadcastClaim(_ roomId: String)
}

/// Follow-mode surface `MissionAgent` drives — optional (nil if the app wires no
/// follower). A separate protocol from `RoverMotion` because following is not a trip: it
/// starts, runs between ticks, and ends on its own terms, so it cannot be expressed through
/// a `navigate`-shaped call that the mission loop waits on.
@MainActor
public protocol RoverFollowing: AnyObject {
    /// What is being followed right now, or `nil` if nothing is.
    var followingSpec: TargetSpec? { get }
    var followState: FollowController.State { get }
    func follow(_ spec: TargetSpec, seed: Vec2?)
    func stopFollowing()
}

/// Default `RoverVoice`: on-device TTS/STT.
@MainActor
public final class SpeechRoverVoice: RoverVoice {
    private let out: SpeechOut
    private let speechIn: SpeechIn?

    public init(out: SpeechOut = SpeechOut(), speechIn: SpeechIn? = nil) {
        self.out = out
        self.speechIn = speechIn
    }

    public func speak(_ text: String) { out.speak(text) }

    public func ask(_ question: String, timeout: TimeInterval) async -> String? {
        out.speak(question)
        guard let speechIn else { return nil }
        return await speechIn.listenOnce(timeout: timeout)
    }
}

/// The mission loop: gather context, ask the current `RoverBrain` for the next action,
/// execute it, and repeat — until the brain says `.stop`/`.done`, or it asks a question
/// that goes unanswered and has nothing left to try.
///
/// Normal missions remain brain-driven. When every configured brain fails before a first
/// decision, a bounded deterministic parser handles the supported offline object-navigation
/// contract, including optional color, other-room search, and explicitly requested return.
@Observable
@MainActor
public final class MissionAgent {
    public enum Phase: Equatable { case idle, thinking, acting, waitingForAnswer }
    private enum VisualTargetScanResult {
        case found(Vec2)
        case notFound
        case failed(String)
        case cancelled
        case sessionGenerationChanged
    }

    private enum ReturnLegOutcome {
        case arrived
        case failed
        case cancelled
        case sessionGenerationChanged
    }

    private enum BlockedHeadingRecoveryOutcome: Equatable {
        case completed
        case cancelled
        case sessionGenerationChanged
    }

    private enum RoomTraversalOutcome {
        case crossed(DoorwayRouteStep)
        case exhausted(String)
        case cancelled
    }

    private enum VisualTargetApproachOutcome {
        case arrived
        case failed(String)
        case cancelled
        case sessionGenerationChanged
    }

    private enum VisualNavigationGoalResult {
        case reachable(Vec2)
        case rejected(NavigationGoalRejection)
    }

    private struct VisualTargetStagingCandidate {
        enum Kind: String {
            case mapOpening = "map_opening"
            case incrementalVisualRay = "incremental_visual_ray"
        }

        let candidate: ExplorationCandidate
        let assessment: NavigationGoalAssessment
        let targetDistanceBefore: Double
        let targetDistanceAfter: Double
        let kind: Kind

        var targetProgress: Double { targetDistanceBefore - targetDistanceAfter }
    }

    public private(set) var phase: Phase = .idle {
        didSet {
            guard phase != oldValue else { return }
            phaseDidChange?(phase)
        }
    }
    public private(set) var memory = MissionMemory()
    /// The mission plan as last written by a brain (see `BrainOutput.updatedPlan`).
    public private(set) var plan: String?
    /// Openings into unexplored space, with stable ids and visited status maintained
    /// across ticks. Visited candidates are kept (they're memory — "already checked, it
    /// was a hallway"); unexplored ones that stop being frontiers (e.g. seen through
    /// without visiting) are dropped so the brain isn't offered stale openings.
    public private(set) var explorationCandidates: [ExplorationCandidate] = []

    private let motion: RoverMotion
    private let perception: RoverPerception
    private let voice: RoverVoice
    private let follower: RoverFollowing?
    private let battery: RoverBattery?
    private let teamRadio: RoverTeamRadio?
    private let roomTopology: RoomTopologyManaging?
    private let doorwayEvidenceProvider: DoorwayEvidenceProviding?
    private let roomTransitionPollInterval: TimeInterval
    private let askTimeout: TimeInterval
    private let brainDecisionTimeout: TimeInterval
    private let blockedHeadingRecoveryAngle: Double
    private let blockedHeadingRecoveryTimeout: TimeInterval
    private let visualTargetConfidenceThreshold: Float
    private let visualTargetScanAngle: Double
    private let visualTargetScanDelay: TimeInterval
    private let maxVisualTargetScanSteps: Int
    private let phaseDidChange: ((Phase) -> Void)?
    private let roomTransitionStateDidChange: ((RoomTransitionDebugState) -> Void)?
    private let roomTransitionTelemetry: RoomTopologyTelemetrySink
    private let commandStatusDidChange: ((MissionCommandStatus) -> Void)?
    private let currentBrain: () -> RoverBrain?
    private let brainErrorLogger: (Error, MissionContext) -> Void
    private let missionTelemetry: MissionTelemetrySink

    private var roomTransitionDebugState: RoomTransitionDebugState = .idle
    private var roomTraversalExhaustionFields: [String: String] = [:]
    private var lastAnswerWasInconclusive = false
    private var nextCandidateNumber = 1
    private var isHandlingMission = false
    private var stopUnconfirmed = false
    private var missionGeneration = 0
    private var nextCommandID: MissionCommandID = 0
    private var commandByMissionID: [Int: (id: MissionCommandID, text: String)] = [:]
    private var terminalCommandIDs: Set<MissionCommandID> = []
    /// Ring buffer of "action → outcome" lines fed to the brain as `recentActions` (see
    /// `makeContext`) so it can notice it's repeating itself — persists across `handle()`
    /// calls within one mission agent, same as `memory`, so a multi-turn mission ("search,
    /// then go back to the ladder") keeps continuity.
    private var recentActionLines: [String] = []
    private let recentActionsLimit = 8
    private var lastDecision: RoverDecision?
    /// Consecutive ticks where the decision matched the previous one, pose barely moved,
    /// and nothing new was remembered — the signal a scripted loop (not the brain) would
    /// catch instantly but an LLM given only the current frame cannot.
    private var consecutiveNoOpTicks = 0
    /// A frontier within this distance of a known candidate is the same opening.
    private let candidateMatchRadius = 1.0
    /// Getting this close to a candidate marks it visited.
    private let visitedRadius = 1.0

    /// How long a running follow is left alone between brain look-ins. Long enough that a
    /// multi-minute follow fits inside `maxTicksPerUtterance`, short enough that the brain
    /// can still break off promptly.
    private let followReviewInterval: TimeInterval = 10

    /// Hard cap on think-ticks per utterance so a brain that never emits `.stop`/`.done`
    /// can't loop forever (matters most for scripted/fake brains in tests).
    private let maxTicksPerUtterance: Int

    public init(motion: RoverMotion,
                perception: RoverPerception,
                voice: RoverVoice,
                follower: RoverFollowing? = nil,
                battery: RoverBattery? = nil,
                teamRadio: RoverTeamRadio? = nil,
                askTimeout: TimeInterval = 8,
                brainDecisionTimeout: TimeInterval = 12,
                maxTicksPerUtterance: Int = 25,
                blockedHeadingRecoveryAngle: Double = .pi / 6,
                blockedHeadingRecoveryTimeout: TimeInterval = RoverConfig.blockedHeadingRecoveryTimeout,
                visualTargetConfidenceThreshold: Float = 0.90,
                visualTargetScanAngle: Double = .pi / 6,
                visualTargetScanDelay: TimeInterval = 1,
                maxVisualTargetScanSteps: Int = 12,
                roomTopology: RoomTopologyManaging? = nil,
                doorwayEvidenceProvider: DoorwayEvidenceProviding? = nil,
                roomTransitionPollInterval: TimeInterval = 0.05,
                phaseDidChange: ((Phase) -> Void)? = nil,
                roomTransitionStateDidChange: ((RoomTransitionDebugState) -> Void)? = nil,
                roomTransitionTelemetry: @escaping RoomTopologyTelemetrySink = { event, fields in
                    RuntimeFileLog.append(event, fields: fields)
                },
                commandStatusDidChange: ((MissionCommandStatus) -> Void)? = nil,
                brainErrorLogger: @escaping (Error, MissionContext) -> Void = { error, context in
                    BrainErrorFileLog.append(error: error, context: context)
                },
                missionTelemetry: @escaping MissionTelemetrySink = { event, fields in
                    RuntimeFileLog.append(event, fields: fields)
                },
                currentBrain: @escaping () -> RoverBrain?) {
        self.motion = motion
        self.perception = perception
        self.voice = voice
        self.follower = follower
        self.battery = battery
        self.teamRadio = teamRadio
        self.roomTopology = roomTopology
        self.doorwayEvidenceProvider = doorwayEvidenceProvider
        self.roomTransitionPollInterval = roomTransitionPollInterval
        self.askTimeout = askTimeout
        self.brainDecisionTimeout = brainDecisionTimeout
        self.maxTicksPerUtterance = maxTicksPerUtterance
        self.blockedHeadingRecoveryAngle = blockedHeadingRecoveryAngle
        self.blockedHeadingRecoveryTimeout = blockedHeadingRecoveryTimeout
        self.visualTargetConfidenceThreshold = visualTargetConfidenceThreshold
        self.visualTargetScanAngle = visualTargetScanAngle
        self.visualTargetScanDelay = visualTargetScanDelay
        self.maxVisualTargetScanSteps = maxVisualTargetScanSteps
        self.phaseDidChange = phaseDidChange
        self.roomTransitionStateDidChange = roomTransitionStateDidChange
        self.roomTransitionTelemetry = roomTransitionTelemetry
        self.commandStatusDidChange = commandStatusDidChange
        self.brainErrorLogger = brainErrorLogger
        self.missionTelemetry = missionTelemetry
        self.currentBrain = currentBrain
    }

    /// Handle one operator utterance end-to-end.
    public func handle(_ utterance: String) async {
        let trimmedUtterance = utterance.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedUtterance.isEmpty else {
            phase = .idle
            RuntimeFileLog.append("voice_command_ignored", fields: ["reason": "blank_utterance"])
            return
        }

        nextCommandID += 1
        let commandID = nextCommandID
        commandStatusDidChange?(.recognized(id: commandID, command: trimmedUtterance))

        RuntimeFileLog.append("voice_command_received", fields: ["utterance": trimmedUtterance])
        if isEmergencyStopUtterance(trimmedUtterance) {
            commandStatusDidChange?(.working(id: commandID, command: trimmedUtterance))
            do {
                try await cancelCurrentMissionAndWait()
            } catch {
                RuntimeFileLog.append("voice_command_stop_failed", fields: ["error": error.localizedDescription])
                publishTerminal(.failed(id: commandID, command: trimmedUtterance,
                                        message: "Rover stop could not be confirmed."))
                return
            }
            RuntimeFileLog.append("voice_command_stop", fields: ["utterance": trimmedUtterance])
            publishTerminal(.cancelled(id: commandID, command: trimmedUtterance))
            return
        }

        guard !stopUnconfirmed else {
            voice.speak("Rover stop could not be confirmed.")
            publishTerminal(.failed(id: commandID, command: trimmedUtterance,
                                    message: "Rover stop could not be confirmed."))
            return
        }
        guard !isHandlingMission else {
            voice.speak("I'm still working on the previous command. Say stop if you want me to cancel it.")
            RuntimeFileLog.append("voice_command_busy", fields: ["utterance": trimmedUtterance])
            publishTerminal(
                .failed(
                    id: commandID,
                    command: trimmedUtterance,
                    message: "I’m still working on the previous command."
                )
            )
            return
        }
        if motion.state != .idle {
            RuntimeFileLog.append("mission_motion_state_reset", fields: [
                "previous_state": motion.state.description
            ])
            await motion.stopAndWait()
        }
        guard let pose = perception.pose else {
            voice.speak("I don't have my bearings yet — give me a moment to look around.")
            RuntimeFileLog.append("voice_command_rejected", fields: [
                "utterance": trimmedUtterance,
                "reason": "missing_pose"
            ])
            publishTerminal(
                .failed(id: commandID, command: trimmedUtterance, message: "I don’t have my bearings yet.")
            )
            return
        }

        isHandlingMission = true
        missionGeneration += 1
        let missionID = missionGeneration
        commandByMissionID[missionID] = (commandID, trimmedUtterance)
        commandStatusDidChange?(.working(id: commandID, command: trimmedUtterance))
        defer {
            if missionGeneration == missionID {
                isHandlingMission = false
            }
        }

        plan = nil
        recentActionLines.removeAll(keepingCapacity: true)
        lastDecision = nil
        consecutiveNoOpTicks = 0
        memory.beginMission(utterance: trimmedUtterance, at: pose)
        lastAnswerWasInconclusive = false
        RuntimeFileLog.append("mission_started", fields: [
            "mission": "\(missionID)",
            "start_x": String(format: "%.2f", pose.position.x),
            "start_y": String(format: "%.2f", pose.position.y),
            "return_requested": Self.hasReturnIntent(trimmedUtterance) ? "true" : "false"
        ])
        publishRoomTransitionState(.idle)
        if RoomTransitionIntent.matches(trimmedUtterance) {
            await runRoomTransitionMission(missionID: missionID)
        } else {
            await runLoop(firstUtterance: trimmedUtterance, missionID: missionID)
        }
        if !terminalCommandIDs.contains(commandID) {
            if !isCurrentMission(missionID) || Task.isCancelled {
                publishTerminal(.cancelled(id: commandID, command: trimmedUtterance))
            } else if case .failed(let reason) = motion.state {
                publishTerminal(
                    .failed(id: commandID, command: trimmedUtterance, message: reason)
                )
            } else {
                publishTerminal(
                    .succeeded(
                        id: commandID,
                        command: trimmedUtterance,
                        message: "Command completed."
                    )
                )
            }
        }
        commandByMissionID[missionID] = nil
    }

    private func publishTerminal(_ status: MissionCommandStatus) {
        guard status.isTerminal, terminalCommandIDs.insert(status.id).inserted else { return }
        commandStatusDidChange?(status)
    }

    private func publishMissionTerminal(
        _ makeStatus: (MissionCommandID, String) -> MissionCommandStatus,
        missionID: Int
    ) {
        guard let command = commandByMissionID[missionID] else { return }
        publishTerminal(makeStatus(command.id, command.text))
    }

    private func publishRoomTransitionState(_ state: RoomTransitionDebugState) {
        guard state != roomTransitionDebugState else { return }
        roomTransitionDebugState = state
        roomTransitionStateDidChange?(state)
    }

    private func runRoomTransitionMission(missionID: Int) async {
        let sessionGeneration = roomTopology?.snapshot.sessionGeneration
        switch await traverseNextDoorway(missionID: missionID) {
        case .crossed(let step):
            let candidateID = roomTopology?.snapshot.doorways.first {
                $0.id == step.doorwayID
            }?.candidateID
            roomTransitionTelemetry("room_transition_mission_completed", [
                "candidate_id": candidateID?.rawValue ?? "unknown",
                "doorway_id": step.doorwayID.rawValue,
                "room_id": step.toRoomID.rawValue,
                "mission_id": String(missionID),
                "session_generation": sessionGeneration.map(String.init) ?? "none",
            ])
            publishRoomTransitionState(.completed(
                doorwayID: step.doorwayID,
                roomID: step.toRoomID
            ))
            publishMissionTerminal(
                { .succeeded(id: $0, command: $1, message: "Entered another room.") },
                missionID: missionID
            )
            setPhase(.idle, missionID: missionID)
        case .exhausted(let reason):
            let fields = roomTraversalExhaustionFields
            roomTraversalExhaustionFields = [:]
            if reason == "search_exhausted" || reason == "missing_topology" {
                finishRoomTransitionExhausted(
                    missionID: missionID,
                    sessionGeneration: sessionGeneration,
                    fields: fields
                )
            } else {
                let message = reason == "missing_pose"
                    ? "I don’t have my bearings yet."
                    : reason
                finishRoomTransitionFailed(
                    reason,
                    message: message,
                    missionID: missionID,
                    sessionGeneration: sessionGeneration,
                    fields: fields
                )
            }
        case .cancelled:
            setPhase(.idle, missionID: missionID)
        }
    }

    private func traverseNextDoorway(
        preferredDoorwayID: DoorwayID? = nil,
        excluding initiallyExcludedCandidateIDs: Set<DoorwayCandidateID> = [],
        missionID: Int
    ) async -> RoomTraversalOutcome {
        guard let roomTopology else {
            return .exhausted("missing_topology")
        }

        let totalScanSteps = 12
        let sessionGeneration = roomTopology.snapshot.sessionGeneration
        let missionFields: [String: String] = [
            "mission_id": String(missionID),
            "session_generation": sessionGeneration.map(String.init) ?? "none",
        ]
        var excludedCandidateIDs = initiallyExcludedCandidateIDs
        var remainingScanSteps = totalScanSteps
        var candidateAttempts = 0
        roomTraversalExhaustionFields = [:]

        while isCurrentMission(missionID), !Task.isCancelled, candidateAttempts < 3 {
            guard roomTopology.snapshot.sessionGeneration == sessionGeneration else {
                await motion.stopAndWait()
                roomTopology.abandonTransition()
                return .exhausted("session_generation_changed")
            }
            let scanStep = totalScanSteps - remainingScanSteps
            let frontiers = perception.explorationFrontiers()
            guard let referencePose = perception.pose else {
                return .exhausted("missing_pose")
            }
            let candidates = roomTopology.refreshCandidates(
                from: frontiers,
                referencePose: referencePose
            )
            roomTransitionTelemetry("room_transition_scan_step", missionFields.merging([
                "scan_step": String(scanStep),
                "total_scan_steps": String(totalScanSteps),
                "frontier_count": String(frontiers.count),
                "candidate_count": String(candidates.count),
                "excluded_candidate_count": String(excludedCandidateIDs.count),
            ]) { _, new in new })
            publishRoomTransitionState(.scanning(
                step: scanStep,
                total: totalScanSteps,
                openingCount: frontiers.count,
                candidateCount: candidates.count
            ))
            let assessments = candidates.map { candidate in
                let goal = candidate.beyondPlaneGoal()
                let assessment = motion.assessGoal(goal)
                return DoorwayCandidateAssessment(
                    candidateID: candidate.id,
                    isReachable: assessment.isReachable,
                    beyondPlaneGoal: assessment.goal,
                    pathDistance: assessment.pathDistance
                )
            }
            var visualBoosts: [DoorwayCandidateID: Double] = [:]
            if let doorwayEvidenceProvider,
               let frame = perception.capturedFrameJPEG(),
               !candidates.isEmpty {
                visualBoosts = await doorwayEvidenceProvider.boostValues(
                    forFrame: frame,
                    candidates: candidates
                )
            }
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_after_doorway_evidence")
                return .cancelled
            }
            let ranked = roomTopology.rankedCandidates(
                assessments: assessments,
                visualBoosts: visualBoosts,
                excluding: excludedCandidateIDs
            )
            for (rank, item) in ranked.enumerated() {
                roomTransitionTelemetry("doorway_candidate_assessed", missionFields.merging([
                    "scan_step": String(scanStep),
                    "candidate_id": item.candidate.id.rawValue,
                    "goal_x": String(format: "%.2f", item.assessment.beyondPlaneGoal.x),
                    "goal_y": String(format: "%.2f", item.assessment.beyondPlaneGoal.y),
                    "reachable": String(item.assessment.isReachable),
                    "path_distance": String(format: "%.2f", item.assessment.pathDistance),
                    "rank": String(rank + 1),
                ]) { _, new in new })
            }

            let selected: RankedDoorwayCandidate?
            if let preferredDoorwayID {
                selected = ranked.first {
                    $0.assessment.isReachable
                        && $0.candidate.doorwayID == preferredDoorwayID
                }
            } else {
                selected = ranked.first { $0.assessment.isReachable }
            }
            if selected == nil {
                for item in ranked where !item.assessment.isReachable {
                    excludedCandidateIDs.insert(item.candidate.id)
                    roomTransitionTelemetry("doorway_candidate_unreachable", missionFields.merging([
                        "scan_step": String(scanStep),
                        "candidate_id": item.candidate.id.rawValue,
                    ]) { _, new in new })
                    publishRoomTransitionState(.candidateFound(
                        id: item.candidate.id,
                        reachable: false
                    ))
                    publishRoomTransitionState(.unreachable(id: item.candidate.id))
                }
            }
            guard var selected else {
                let reason: String
                if frontiers.isEmpty {
                    reason = "no_openings"
                } else if candidates.isEmpty {
                    reason = "no_admitted_candidates"
                } else if !ranked.isEmpty {
                    reason = preferredDoorwayID == nil
                        ? "no_reachable_candidates"
                        : "preferred_doorway_unavailable"
                } else {
                    reason = "no_ranked_candidates"
                }
                roomTransitionTelemetry("room_transition_scan_continued", missionFields.merging([
                    "scan_step": String(scanStep),
                    "reason": reason,
                    "remaining_scan_steps": String(remainingScanSteps),
                ]) { _, new in new })
                guard remainingScanSteps > 0 else { break }
                remainingScanSteps -= 1
                setPhase(.acting, missionID: missionID)
                let frameBeforeTurn = perception.frameSequence
                switch await rotateForScanRespectingCancellation(
                    missionID: missionID, by: .pi / 6,
                    expectedSessionGeneration: sessionGeneration
                ) {
                case .completed:
                    break
                case .failed(let reason):
                    roomTraversalExhaustionFields = ["scan_step": String(scanStep)]
                    return .exhausted(reason)
                case .cancelled:
                    return .cancelled
                case .sessionGenerationChanged:
                    roomTraversalExhaustionFields = ["scan_step": String(scanStep)]
                    return .exhausted("session_generation_changed")
                }
                if case .failed(let reason) = motion.state {
                    roomTraversalExhaustionFields = ["scan_step": String(scanStep)]
                    return .exhausted(reason)
                }
                switch await waitForFreshTrackedPerceptionFrame(
                    newerThan: frameBeforeTurn,
                    expectedSessionGeneration: sessionGeneration,
                    missionID: missionID
                ) {
                case .ready:
                    break
                case .cancelled:
                    return .cancelled
                case .sessionGenerationChanged:
                    roomTraversalExhaustionFields = ["scan_step": String(scanStep)]
                    return .exhausted("session_generation_changed")
                case .timedOut:
                    roomTraversalExhaustionFields = ["scan_step": String(scanStep)]
                    return .exhausted("fresh_frame_timeout")
                }
                continue
            }

            candidateAttempts += 1
            excludedCandidateIDs.insert(selected.candidate.id)
            roomTransitionTelemetry("doorway_candidate_selected", missionFields.merging([
                "scan_step": String(scanStep),
                "candidate_id": selected.candidate.id.rawValue,
                "reachable": "true",
                "goal_x": String(format: "%.2f", selected.assessment.beyondPlaneGoal.x),
                "goal_y": String(format: "%.2f", selected.assessment.beyondPlaneGoal.y),
            ]) { _, new in new })
            publishRoomTransitionState(.candidateFound(
                id: selected.candidate.id,
                reachable: true
            ))
            guard let approachPose = perception.pose else {
                roomTraversalExhaustionFields = [
                    "candidate_id": selected.candidate.id.rawValue,
                ]
                return .exhausted("missing_pose")
            }
            var startResult = roomTopology.beginTransition(
                candidateID: selected.candidate.id,
                approachPose: approachPose
            )
            if case .correctedOrientation(let correctedCandidate) = startResult {
                let correctedGoal = correctedCandidate.beyondPlaneGoal()
                let correctedAssessment = motion.assessGoal(correctedGoal)
                roomTransitionTelemetry("room_transition_goal_reassessed", missionFields.merging([
                    "candidate_id": correctedCandidate.id.rawValue,
                    "goal_x": String(format: "%.2f", correctedGoal.x),
                    "goal_y": String(format: "%.2f", correctedGoal.y),
                    "reachable": String(correctedAssessment.isReachable),
                ]) { _, new in new })
                guard correctedAssessment.isReachable else {
                    publishRoomTransitionFailure(
                        "corrected_goal_unreachable",
                        missionID: missionID,
                        sessionGeneration: sessionGeneration,
                        fields: ["candidate_id": correctedCandidate.id.rawValue]
                    )
                    continue
                }
                selected = RankedDoorwayCandidate(
                    candidate: correctedCandidate,
                    assessment: DoorwayCandidateAssessment(
                        candidateID: correctedCandidate.id,
                        isReachable: true,
                        beyondPlaneGoal: correctedAssessment.goal,
                        pathDistance: correctedAssessment.pathDistance
                    ),
                    visualBoost: selected.visualBoost
                )
                startResult = roomTopology.beginTransition(
                    candidateID: correctedCandidate.id,
                    approachPose: approachPose
                )
            }
            guard startResult == .started else {
                let reason: String
                if case .rejected(let rejection) = startResult {
                    reason = rejection.rawValue
                } else {
                    reason = "orientation_changed_repeatedly"
                }
                publishRoomTransitionFailure(
                    reason,
                    missionID: missionID,
                    sessionGeneration: sessionGeneration,
                    fields: ["candidate_id": selected.candidate.id.rawValue]
                )
                continue
            }

            roomTransitionTelemetry("room_transition_approach_started", missionFields.merging([
                "candidate_id": selected.candidate.id.rawValue,
                "goal_x": String(format: "%.2f", selected.assessment.beyondPlaneGoal.x),
                "goal_y": String(format: "%.2f", selected.assessment.beyondPlaneGoal.y),
            ]) { _, new in new })
            publishRoomTransitionState(.approaching(id: selected.candidate.id))
            setPhase(.acting, missionID: missionID)
            motion.navigate(to: selected.assessment.beyondPlaneGoal)
            var lastFrameSequence: UInt64?
            var trackingRecoveryDeadline: Date?
            while isCurrentMission(missionID), !Task.isCancelled {
                guard roomTopology.snapshot.sessionGeneration == sessionGeneration else {
                    await motion.stopAndWait()
                    roomTopology.abandonTransition()
                    return .exhausted("session_generation_changed")
                }
                if let observation = perception.latestObservation,
                   observation.sessionGeneration == roomTopology.snapshot.sessionGeneration,
                   lastFrameSequence.map({ observation.frameSequence > $0 }) ?? true {
                    lastFrameSequence = observation.frameSequence
                    if observation.trackingQuality != .normal {
                        if trackingRecoveryDeadline == nil {
                            await motion.stopAndWait()
                            trackingRecoveryDeadline = Date().addingTimeInterval(
                                RoverConfig.scanFrameFreshnessTimeout
                            )
                        }
                        continue
                    }
                    if trackingRecoveryDeadline != nil {
                        trackingRecoveryDeadline = nil
                        publishRoomTransitionState(.approaching(id: selected.candidate.id))
                        motion.navigate(to: selected.assessment.beyondPlaneGoal)
                    }
                    if roomTopology.observeTransition(observation.transitionObservation)
                        == .readyForConfirmation {
                        roomTransitionTelemetry(
                            "room_transition_crossing_confirmation_started",
                            missionFields.merging([
                                "candidate_id": selected.candidate.id.rawValue,
                                "frame_sequence": String(observation.frameSequence),
                            ]) { _, new in new }
                        )
                        publishRoomTransitionState(.confirmingCrossing(id: selected.candidate.id))
                        await motion.stopAndWait()
                        guard isCurrentMission(missionID), !Task.isCancelled else {
                            if isCurrentMission(missionID) {
                                roomTopology.abandonTransition()
                                publishRoomTransitionState(.idle)
                            }
                            return .cancelled
                        }
                        if let roomID = roomTopology.confirmTransition(),
                           let doorwayID = roomTopology.snapshot.doorways.first(where: {
                               $0.candidateID == selected.candidate.id
                           })?.id {
                            guard let fromRoomID = roomTopology.snapshot.doorways.first(where: {
                                $0.id == doorwayID
                            }).flatMap({ doorway in
                                doorway.firstRoomID == roomID
                                    ? doorway.secondRoomID
                                    : doorway.firstRoomID
                            }) else {
                                publishRoomTransitionFailure(
                                    "crossing_route_step_failed",
                                    missionID: missionID,
                                    sessionGeneration: sessionGeneration,
                                    fields: ["candidate_id": selected.candidate.id.rawValue]
                                )
                                break
                            }
                            return .crossed(
                                DoorwayRouteStep(
                                    doorwayID: doorwayID,
                                    fromRoomID: fromRoomID,
                                    toRoomID: roomID
                                )
                            )
                        }
                        publishRoomTransitionFailure(
                            "crossing_confirmation_failed",
                            missionID: missionID,
                            sessionGeneration: sessionGeneration,
                            fields: ["candidate_id": selected.candidate.id.rawValue]
                        )
                        break
                    }
                }

                if let trackingRecoveryDeadline {
                    if Date() >= trackingRecoveryDeadline {
                        roomTopology.rejectTransition(reason: "tracking_unavailable")
                        publishRoomTransitionFailure(
                            "tracking_unavailable",
                            missionID: missionID,
                            sessionGeneration: sessionGeneration,
                            fields: ["candidate_id": selected.candidate.id.rawValue]
                        )
                        break
                    }
                    if roomTransitionPollInterval > 0 {
                        try? await Task.sleep(for: .seconds(roomTransitionPollInterval))
                    } else {
                        await Task.yield()
                    }
                    continue
                }
                if case .failed(let reason) = motion.state {
                    await motion.stopAndWait()
                    roomTopology.rejectTransition(reason: reason)
                    publishRoomTransitionFailure(
                        reason,
                        missionID: missionID,
                        sessionGeneration: sessionGeneration,
                        fields: ["candidate_id": selected.candidate.id.rawValue]
                    )
                    break
                }
                if motion.state == .arrived || motion.state == .idle {
                    roomTopology.rejectTransition(reason: "crossing_not_confirmed")
                    publishRoomTransitionFailure(
                        "crossing_not_confirmed",
                        missionID: missionID,
                        sessionGeneration: sessionGeneration,
                        fields: ["candidate_id": selected.candidate.id.rawValue]
                    )
                    break
                }
                if roomTransitionPollInterval > 0 {
                    try? await Task.sleep(for: .seconds(roomTransitionPollInterval))
                } else {
                    await Task.yield()
                }
            }
        }

        if !isCurrentMission(missionID) || Task.isCancelled {
            if isCurrentMission(missionID) {
                motion.cancel()
                roomTopology.abandonTransition()
                publishRoomTransitionState(.idle)
                setPhase(.idle, missionID: missionID)
            }
            return .cancelled
        }
        await motion.stopAndWait()
        guard isCurrentMission(missionID), !Task.isCancelled else { return .cancelled }
        roomTopology.abandonTransition()
        roomTraversalExhaustionFields = [
            "candidate_attempts": String(candidateAttempts),
            "scan_steps_used": String(totalScanSteps - remainingScanSteps),
        ]
        return .exhausted("search_exhausted")
    }

    private func publishRoomTransitionFailure(
        _ reason: String,
        missionID: Int,
        sessionGeneration: UInt64? = nil,
        fields: [String: String] = [:]
    ) {
        publishRoomTransitionState(.failed(reason: reason))
        roomTransitionTelemetry("room_transition_failed", fields.merging([
            "reason": reason,
            "mission_id": String(missionID),
            "session_generation": sessionGeneration.map(String.init) ?? "none",
        ]) { _, new in new })
    }

    private func finishRoomTransitionFailed(
        _ reason: String,
        message: String,
        missionID: Int,
        sessionGeneration: UInt64? = nil,
        fields: [String: String] = [:]
    ) {
        guard isCurrentMission(missionID) else { return }
        publishRoomTransitionFailure(
            reason,
            missionID: missionID,
            sessionGeneration: sessionGeneration,
            fields: fields
        )
        publishMissionTerminal(
            { .failed(id: $0, command: $1, message: message) },
            missionID: missionID
        )
        setPhase(.idle, missionID: missionID)
    }

    private func finishRoomTransitionExhausted(
        missionID: Int,
        sessionGeneration: UInt64? = nil,
        fields: [String: String] = [:]
    ) {
        guard isCurrentMission(missionID) else { return }
        publishRoomTransitionState(.exhausted)
        roomTransitionTelemetry("room_transition_exhausted", fields.merging([
            "mission_id": String(missionID),
            "session_generation": sessionGeneration.map(String.init) ?? "none",
        ]) { _, new in new })
        voice.speak("I couldn’t find a safe route into another room.")
        publishMissionTerminal(
            { .failed(id: $0, command: $1, message: "I couldn’t find a safe route into another room.") },
            missionID: missionID
        )
        setPhase(.idle, missionID: missionID)
    }

    public func cancelCurrentMissionAndWait() async throws {
        missionGeneration += 1
        let generation = missionGeneration
        isHandlingMission = false
        phase = .acting
        stopUnconfirmed = true
        follower?.stopFollowing()
        try await motion.stopAndConfirm()
        if missionGeneration == generation {
            stopUnconfirmed = false
            roomTopology?.abandonTransition()
            publishRoomTransitionState(.idle)
            phase = .idle
        }
    }

    // MARK: - Loop

    /// Async work may outlive acknowledged cancellation and a subsequent mission.
    /// Fence phase mutation and its UI callback at the source, including idle updates.
    private func setPhase(_ value: Phase, missionID: Int) {
        guard missionID == missionGeneration else { return }
        phase = value
    }

    private func runLoop(firstUtterance: String?, missionID: Int) async {
        var nextUtterance = firstUtterance
        let missionUtterance = firstUtterance ?? ""
        let missionVisualIntent = OfflineObjectMissionIntentParser.parse(missionUtterance)
        var lockedVisualQuery = missionVisualIntent?.objectQuery
        var visualTargetScanSteps = 0
        if let missionVisualIntent {
            RuntimeFileLog.append("mission_target_locked", fields: [
                "mission": "\(missionID)",
                "target": missionVisualIntent.objectQuery,
                "target_label": missionVisualIntent.targetLabel,
                "requested_colors": Self.requestedColorsDescription(missionVisualIntent.requestedColors),
                "source": "operator_command",
                "threshold": String(format: "%.2f", visualTargetConfidenceThreshold)
            ])
        }
        // Distinguishes "the brain itself failed" (missing/timed out/errored) from
        // genuinely exhausting every tick without the brain ever stopping — only the
        // latter should get the "used up my time" wrap-up; the brain-failure paths
        // already spoke their own message and shouldn't pile a second one on top.
        var brainFailed = false
        var hasUsableBrainOutput = false

        for tick in 0..<maxTicksPerUtterance {
            guard !missionCancellationDetected(missionID) else {
                setPhase(.idle, missionID: missionID)
                RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                return
            }
            // A plain follow request needs no model to understand, and a running follow
            // needs none to continue — which is what makes "follow me" work on a phone
            // with no Apple Intelligence and no signal.
            let directFollow = tick == 0 ? firstUtterance.flatMap(Self.directFollowQuery) : nil
            let brain = currentBrain()
            if brain == nil, directFollow == nil || follower == nil, !isFollowing {
                if followEndedWithNothingElseToDo { setPhase(.idle, missionID: missionID); return }
                RuntimeFileLog.append("mission_failed", fields: [
                    "mission": "\(missionID)",
                    "reason": "missing_brain"
                ])
                if !hasUsableBrainOutput {
                    await attemptOfflineObjectFallback(
                        utterance: missionUtterance,
                        missionID: missionID,
                        brainFailureReason: "missing_brain",
                        failureMessage: "Sorry, I can’t think right now."
                    )
                    return
                }
                voice.speak("Sorry, I can't think right now.")
                publishMissionTerminal(
                    { .failed(id: $0, command: $1, message: "Sorry, I can’t think right now.") },
                    missionID: missionID
                )
                RuntimeFileLog.append("mission_offline_fallback_skipped", fields: [
                    "mission": "\(missionID)",
                    "reason": "brain_output_already_produced"
                ])
                brainFailed = true
                break
            }

            setPhase(.thinking, missionID: missionID)
            await perception.refreshDetections()
            guard isCurrentMission(missionID), !Task.isCancelled else { return }
            let rememberedCountBefore = memory.rememberedObjects.count
            updateWorldModel()
            let newObjects = memory.rememberedObjects.count - rememberedCountBefore
            let ctx = makeContext(utterance: nextUtterance, missionID: missionID)
            RuntimeFileLog.append("mission_thinking", fields: [
                "mission": "\(missionID)",
                "tick": "\(tick)",
                "nav": ctx.navState.description,
                "visible": "\(ctx.visibleObjects.count)",
                "objects": visibleObjectsLogSummary(ctx.visibleObjects),
                "openings": "\(ctx.explorationCandidates.count)"
            ])
            nextUtterance = nil

            let output: BrainOutput
            if let directFollow, follower != nil {
                output = BrainOutput(decision: .follow(.visualQuery(directFollow)))
                RuntimeFileLog.append("mission_follow_direct", fields: [
                    "mission": "\(missionID)", "query": directFollow
                ])
            } else if let brain {
                do {
                    output = try await nextBrainAction(brain, context: ctx)
                } catch let error as CancellationError {
                    if missionCancellationDetected(missionID) {
                        cancelActiveMotion(missionID: missionID, reason: "brain_decision_cancelled")
                        return
                    }
                    if let keep = keepFollowingOutput(missionID: missionID, error: error, context: ctx) {
                        output = keep
                    } else {
                        brainErrorLogger(error, ctx)
                        RuntimeFileLog.append("mission_brain_error", fields: [
                            "mission": "\(missionID)",
                            "error": error.localizedDescription
                        ])
                        if followEndedWithNothingElseToDo { setPhase(.idle, missionID: missionID); return }
                        if !hasUsableBrainOutput {
                            await attemptOfflineObjectFallback(
                                utterance: missionUtterance,
                                missionID: missionID,
                                brainFailureReason: "brain_cancellation_error",
                                failureMessage: "Sorry, I’m having trouble thinking right now."
                            )
                            return
                        }
                        voice.speak("Sorry, I'm having trouble thinking right now.")
                        publishMissionTerminal(
                            { .failed(id: $0, command: $1, message: "Sorry, I’m having trouble thinking right now.") },
                            missionID: missionID
                        )
                        RuntimeFileLog.append("mission_offline_fallback_skipped", fields: [
                            "mission": "\(missionID)",
                            "reason": "brain_output_already_produced"
                        ])
                        brainFailed = true
                        break
                    }
                } catch is BrainDecisionTimeoutError {
                    if let keep = keepFollowingOutput(missionID: missionID) {
                        output = keep
                    } else {
                        guard isCurrentMission(missionID) else {
                            setPhase(.idle, missionID: missionID)
                            RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                            return
                        }
                        RuntimeFileLog.append("mission_brain_timeout", fields: [
                            "mission": "\(missionID)",
                            "timeout": String(format: "%.2f", brainDecisionTimeout)
                        ])
                        if followEndedWithNothingElseToDo { setPhase(.idle, missionID: missionID); return }
                        if !hasUsableBrainOutput {
                            await attemptOfflineObjectFallback(
                                utterance: missionUtterance,
                                missionID: missionID,
                                brainFailureReason: "brain_timeout",
                                failureMessage: "Sorry, I’m having trouble thinking right now."
                            )
                            return
                        }
                        voice.speak("Sorry, I'm having trouble thinking right now.")
                        publishMissionTerminal(
                            { .failed(id: $0, command: $1, message: "Sorry, I’m having trouble thinking right now.") },
                            missionID: missionID
                        )
                        RuntimeFileLog.append("mission_offline_fallback_skipped", fields: [
                            "mission": "\(missionID)",
                            "reason": "brain_output_already_produced"
                        ])
                        brainFailed = true
                        break
                    }
                } catch {
                    if let keep = keepFollowingOutput(missionID: missionID, error: error, context: ctx) {
                        output = keep
                    } else {
                        guard isCurrentMission(missionID) else {
                            setPhase(.idle, missionID: missionID)
                            RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                            return
                        }
                        brainErrorLogger(error, ctx)
                        RuntimeFileLog.append("mission_brain_error", fields: [
                            "mission": "\(missionID)",
                            "error": error.localizedDescription
                        ])
                        if followEndedWithNothingElseToDo { setPhase(.idle, missionID: missionID); return }
                        if !hasUsableBrainOutput {
                            await attemptOfflineObjectFallback(
                                utterance: missionUtterance,
                                missionID: missionID,
                                brainFailureReason: "brain_error",
                                failureMessage: "Sorry, I’m having trouble thinking right now."
                            )
                            return
                        }
                        voice.speak("Sorry, I'm having trouble thinking right now.")
                        publishMissionTerminal(
                            { .failed(id: $0, command: $1, message: "Sorry, I’m having trouble thinking right now.") },
                            missionID: missionID
                        )
                        RuntimeFileLog.append("mission_offline_fallback_skipped", fields: [
                            "mission": "\(missionID)",
                            "reason": "brain_output_already_produced"
                        ])
                        brainFailed = true
                        break
                    }
                }
            } else if let keep = keepFollowingOutput(missionID: missionID) {
                output = keep
            } else {
                brainFailed = true
                break
            }
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_after_brain_decision")
                return
            }
            hasUsableBrainOutput = true
            if let updated = output.updatedPlan, !updated.isEmpty {
                plan = updated
                RuntimeFileLog.append("mission_plan_updated", fields: [
                    "mission": "\(missionID)",
                    "plan": updated
                ])
            }
            RuntimeFileLog.append("mission_decision", fields: [
                "mission": "\(missionID)",
                "tick": "\(tick)",
                "decision": decisionDescription(output.decision)
            ])

            setPhase(.acting, missionID: missionID)
            let decision = output.decision
            let poseBefore = perception.pose?.position
            var outcome = ""

            switch decision {
            case .navigate(let target):
                stopFollowingBeforeOtherMotion(decision, missionID: missionID)
                let effectiveTarget = effectiveNavigationTarget(target,
                                                                lockedVisualQuery: &lockedVisualQuery,
                                                                missionID: missionID)
                guard let goal = resolve(effectiveTarget, missionID: missionID) else {
                    switch await scanForUnresolvedVisualTarget(effectiveTarget,
                                                               missionID: missionID,
                                                               scanSteps: &visualTargetScanSteps) {
                    case .found(let scannedGoal):
                        visualTargetScanSteps = 0
                        if navigate(to: scannedGoal, for: effectiveTarget) {
                            await waitForMotionToSettle(missionID: missionID)
                        } else {
                            switch await approachVisualTarget(
                                effectiveTarget,
                                initialGoal: scannedGoal,
                                missionID: missionID,
                                expectedSessionGeneration: roomTopology?.snapshot.sessionGeneration
                            ) {
                            case .arrived:
                                break
                            case .failed(let reason):
                                failTargetNavigationMission(missionID: missionID, reason: reason)
                                return
                            case .cancelled:
                                setPhase(.idle, missionID: missionID)
                                RuntimeFileLog.append("mission_cancelled", fields: [
                                    "mission": "\(missionID)",
                                    "reason": "cancelled_during_target_approach"
                                ])
                                return
                            case .sessionGenerationChanged:
                                setPhase(.idle, missionID: missionID)
                                RuntimeFileLog.append("mission_cancelled", fields: [
                                    "mission": "\(missionID)",
                                    "reason": "session_generation_changed"
                                ])
                                return
                            }
                        }
                        guard isCurrentMission(missionID) else {
                            setPhase(.idle, missionID: missionID)
                            RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                            return
                        }
                        if await recoverVisualNavigation(effectiveTarget,
                                                         missionID: missionID,
                                                         scanSteps: &visualTargetScanSteps,
                                                         missionUtterance: missionUtterance) {
                            return
                        }
                        if await recoverOrStopMissionIfMotionFailed(missionID: missionID,
                                                                    recoverFromBlockedHeading: true) { return }
                        if await completeReturnLegIfNeeded(after: effectiveTarget,
                                                           missionID: missionID,
                                                           missionUtterance: missionUtterance,
                                                           lockedVisualQuery: &lockedVisualQuery) {
                            return
                        }
                        if finishMissionIfVisualTargetArrived(effectiveTarget,
                                                              missionID: missionID,
                                                              missionUtterance: missionUtterance) {
                            return
                        }
                        outcome = describeMotionOutcome(label: "navigate", poseBefore: poseBefore)
                        if recordTick(decision: decision,
                                      poseBefore: poseBefore,
                                      newObjects: newObjects,
                                      outcome: outcome) {
                            setPhase(.idle, missionID: missionID)
                            return
                        }
                        continue
                    case .failed(let reason):
                        failScanRotationMission(missionID: missionID, reason: reason)
                        return
                    case .cancelled:
                        setPhase(.idle, missionID: missionID)
                        RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                        return
                    case .sessionGenerationChanged:
                        setPhase(.idle, missionID: missionID)
                        RuntimeFileLog.append("mission_cancelled", fields: [
                            "mission": "\(missionID)",
                            "reason": "session_generation_changed"
                        ])
                        return
                    case .notFound:
                        updateWorldModel()
                        if let candidate = nextUnexploredCandidate() {
                            RuntimeFileLog.append("mission_target_frontier_selected", fields: [
                                "mission": "\(missionID)",
                                "target": lockedVisualQuery ?? missionUtterance,
                                "candidate": candidate.id,
                                "width": String(format: "%.2f", candidate.widthMeters),
                                "goal_x": String(format: "%.2f", candidate.worldPoint.x),
                                "goal_y": String(format: "%.2f", candidate.worldPoint.y),
                            ])
                            visualTargetScanSteps = 0
                            motion.navigate(to: candidate.worldPoint)
                            await waitForMotionToSettle(missionID: missionID)
                            let reachedCandidate = motion.state == .arrived
                            if await recoverOrStopMissionIfMotionFailed(
                                missionID: missionID,
                                recoverFromBlockedHeading: true,
                                blockedCandidateId: candidate.id
                            ) {
                                return
                            }
                            if reachedCandidate {
                                markExplorationCandidateVisited(
                                    candidate.id,
                                    reason: "searched frontier for \(lockedVisualQuery ?? missionUtterance)",
                                    event: "mission_exploration_candidate_visited"
                                )
                            }
                            outcome = describeMotionOutcome(
                                label: "search(\(candidate.id))",
                                poseBefore: poseBefore
                            )
                            if recordTick(decision: decision,
                                          poseBefore: poseBefore,
                                          newObjects: newObjects,
                                          outcome: outcome) {
                                setPhase(.idle, missionID: missionID)
                                return
                            }
                            continue
                        }
                    }
                    guard isCurrentMission(missionID) else {
                        setPhase(.idle, missionID: missionID)
                        RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                        return
                    }
                    voice.speak("I couldn't quite figure out where that is.")
                    if recordTick(decision: decision, poseBefore: poseBefore, newObjects: newObjects,
                                   outcome: "navigate → couldn't resolve target") {
                        setPhase(.idle, missionID: missionID)
                        return
                    }
                    continue
                }
                visualTargetScanSteps = 0
                if navigate(to: goal, for: effectiveTarget) {
                    guard await waitForMotionToSettle(
                        missionID: missionID,
                        expectedSessionGeneration: roomTopology?.snapshot.sessionGeneration
                    ) else {
                        setPhase(.idle, missionID: missionID)
                        return
                    }
                } else {
                    switch await approachVisualTarget(
                        effectiveTarget,
                        initialGoal: goal,
                        missionID: missionID,
                        expectedSessionGeneration: roomTopology?.snapshot.sessionGeneration
                    ) {
                    case .arrived:
                        break
                    case .failed(let reason):
                        failTargetNavigationMission(missionID: missionID, reason: reason)
                        return
                    case .cancelled:
                        setPhase(.idle, missionID: missionID)
                        RuntimeFileLog.append("mission_cancelled", fields: [
                            "mission": "\(missionID)",
                            "reason": "cancelled_during_target_approach"
                        ])
                        return
                    case .sessionGenerationChanged:
                        setPhase(.idle, missionID: missionID)
                        RuntimeFileLog.append("mission_cancelled", fields: [
                            "mission": "\(missionID)",
                            "reason": "session_generation_changed"
                        ])
                        return
                    }
                }
                if await recoverVisualNavigation(effectiveTarget,
                                                  missionID: missionID,
                                                  scanSteps: &visualTargetScanSteps,
                                                  missionUtterance: missionUtterance) {
                    return
                }
                if await recoverOrStopMissionIfMotionFailed(missionID: missionID,
                                                            recoverFromBlockedHeading: true) { return }
                if await completeReturnLegIfNeeded(after: effectiveTarget,
                                                   missionID: missionID,
                                                   missionUtterance: missionUtterance,
                                                   lockedVisualQuery: &lockedVisualQuery) {
                    return
                }
                if finishMissionIfVisualTargetArrived(effectiveTarget,
                                                       missionID: missionID,
                                                       missionUtterance: missionUtterance) {
                    return
                }
                outcome = describeMotionOutcome(label: "navigate", poseBefore: poseBefore)

            case .explore(let candidateId):
                stopFollowingBeforeOtherMotion(decision, missionID: missionID)
                guard let requestedCandidate = explorationCandidates.first(where: { $0.id == candidateId }) else {
                    voice.speak("I'm not sure which opening that is anymore.")
                    if recordTick(decision: decision, poseBefore: poseBefore, newObjects: newObjects,
                                   outcome: "explore(\(candidateId)) → unknown opening id") {
                        setPhase(.idle, missionID: missionID)
                        return
                    }
                    continue
                }
                let candidate: ExplorationCandidate
                if requestedCandidate.status == .visited {
                    guard let unexplored = nextUnexploredCandidate() else {
                        RuntimeFileLog.append("mission_exploration_candidate_rejected", fields: [
                            "candidate": candidateId,
                            "reason": "already_visited",
                            "fallback": "scan_30deg",
                        ])
                        await motion.rotateForScan(by: blockedHeadingRecoveryAngle)
                        if await recoverOrStopMissionIfMotionFailed(missionID: missionID) { return }
                        outcome = "explore(\(candidateId)) → already visited; scanned instead"
                        break
                    }
                    candidate = unexplored
                    RuntimeFileLog.append("mission_exploration_candidate_rerouted", fields: [
                        "requested": candidateId,
                        "selected": candidate.id,
                        "reason": "requested_candidate_visited",
                    ])
                } else {
                    candidate = requestedCandidate
                }
                motion.navigate(to: candidate.worldPoint)
                await waitForMotionToSettle(missionID: missionID)
                if await recoverOrStopMissionIfMotionFailed(missionID: missionID,
                                                            recoverFromBlockedHeading: true,
                                                            blockedCandidateId: candidate.id) { return }
                outcome = describeMotionOutcome(label: "explore(\(candidate.id))", poseBefore: poseBefore)

            case .lookAround(let angle):
                stopFollowingBeforeOtherMotion(decision, missionID: missionID)
                await motion.rotate(by: angle)
                if await recoverOrStopMissionIfMotionFailed(missionID: missionID) { return }
                let poseAfter = perception.pose?.position
                let stayedPut = distance(poseBefore, poseAfter) < 0.05
                outcome = "lookAround(\(fmt(angle))) → " + (stayedPut
                    ? (newObjects > 0 ? "pose unchanged, saw \(newObjects) new thing(s)" : "pose unchanged, nothing new seen")
                    : "pose changed")

            case .ask(let question):
                setPhase(.waitingForAnswer, missionID: missionID)
                if let reply = await voice.ask(question, timeout: askTimeout) {
                    lastAnswerWasInconclusive = false
                    if let pose = perception.pose { memory.record(utterance: reply, at: pose) }
                    nextUtterance = reply
                    outcome = "ask(\"\(question)\") → replied: \"\(reply)\""
                } else {
                    lastAnswerWasInconclusive = true
                    outcome = "ask(\"\(question)\") → no reply"
                }

            case .follow(let target):
                guard let followOutcome = await startOrContinueFollowing(target,
                                                                         missionID: missionID) else {
                    setPhase(.idle, missionID: missionID)
                    RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                    return
                }
                outcome = followOutcome

            case .say(let text):
                voice.speak(text)
                outcome = "say(\"\(text)\") → (no motion)"

            case .claimRoom(let roomId):
                teamRadio?.broadcastClaim(roomId)
                outcome = "claimRoom(\(roomId)) → broadcast"

            case .stop:
                do {
                    try await cancelCurrentMissionAndWait()
                } catch {
                    publishMissionTerminal({ .failed(id: $0, command: $1,
                        message: "Rover stop could not be confirmed.") }, missionID: missionID)
                }
                return

            case .done:
                setPhase(.idle, missionID: missionID)
                RuntimeFileLog.append("mission_done", fields: ["mission": "\(missionID)"])
                return
            }

            if recordTick(decision: decision, poseBefore: poseBefore, newObjects: newObjects, outcome: outcome) {
                setPhase(.idle, missionID: missionID)
                return
            }
        }

        if !brainFailed {
            voice.speak("I've used up the time I had for this and want to check in rather than keep going. "
                + wrapUpFindings())
        }
        setPhase(.idle, missionID: missionID)
        RuntimeFileLog.append("mission_finished", fields: ["mission": "\(missionID)"])
    }

    private func attemptOfflineObjectFallback(
        utterance: String,
        missionID: Int,
        brainFailureReason: String,
        failureMessage: String
    ) async {
        guard isCurrentMission(missionID), !Task.isCancelled else {
            setPhase(.idle, missionID: missionID)
            RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                "mission": "\(missionID)",
                "reason": "mission_cancelled_before_start"
            ])
            return
        }
        guard let intent = OfflineObjectMissionIntentParser.parse(utterance) else {
            missionTelemetry("mission_offline_fallback_rejected", [
                "mission": "\(missionID)",
                "reason": "unsupported_command",
                "brain_failure": brainFailureReason
            ])
            failOfflineObjectMission(
                missionID: missionID,
                message: failureMessage,
                reason: "unsupported_command"
            )
            return
        }

        missionTelemetry("mission_brain_selected", [
            "mission": "\(missionID)",
            "brain": "offline_object_fallback",
            "reason": brainFailureReason,
        ])
        missionTelemetry("mission_offline_fallback_started", [
            "mission": "\(missionID)",
            "target": intent.objectQuery,
            "target_label": intent.targetLabel,
            "requested_colors": Self.requestedColorsDescription(intent.requestedColors),
            "return_requested": intent.shouldReturn ? "true" : "false",
            "brain_failure": brainFailureReason,
            "reason": brainFailureReason,
        ])
        await perception.refreshDetections()
        guard !missionCancellationDetected(missionID) else { return }
        await runOfflineObjectMission(
            intent,
            missionID: missionID,
            failureMessage: failureMessage
        )
    }

    private func runOfflineObjectMission(
        _ intent: OfflineObjectMissionIntent,
        missionID: Int,
        failureMessage: String
    ) async {
        let target = NavigationTarget.visualQuery(intent.objectQuery)
        let topology = roomTopology
        let sessionGeneration = topology?.snapshot.sessionGeneration
        let startRoomID = topology?.snapshot.currentRoomID
        let maximumRoomSearches = 3
        let maximumDoorwayCrossings = 2
        var searchedCandidateIDs: Set<DoorwayCandidateID> = []
        var crossedDoorwaySteps: [DoorwayRouteStep] = []
        var roomsSearched = 0
        var requiresFreshScanTurn = false
        var resolvedGoal: Vec2?

        if intent.searchOtherRooms {
            missionTelemetry("mission_room_search_started", [
                "mission": "\(missionID)",
                "reason": "other_room_requested",
                "target": intent.objectQuery,
                "room_id": startRoomID?.rawValue ?? "unavailable",
                "session_generation": sessionGeneration.map(String.init) ?? "unavailable",
            ])
        }

        while resolvedGoal == nil {
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_while_scanning")
                RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                    "mission": "\(missionID)",
                    "reason": "cancelled_while_scanning"
                ])
                return
            }
            if sessionGenerationChanged(from: sessionGeneration) {
                failOfflineObjectMission(
                    missionID: missionID,
                    message: failureMessage,
                    reason: "session_generation_changed"
                )
                return
            }

            var scanSteps = 0
            if !requiresFreshScanTurn,
               let visibleGoal = resolve(target, missionID: missionID) {
                resolvedGoal = visibleGoal
            } else {
                switch await scanForUnresolvedVisualTarget(
                    target,
                    missionID: missionID,
                    scanSteps: &scanSteps,
                    requireNewFrameAfterInitialTurn: requiresFreshScanTurn,
                    expectedSessionGeneration: sessionGeneration,
                    requireTrackedFrame: requiresFreshScanTurn
                ) {
                case .found(let scannedGoal):
                    resolvedGoal = scannedGoal
                case .notFound:
                    break
                case .failed(let reason):
                    failOfflineObjectMission(
                        missionID: missionID,
                        message: reason,
                        reason: "scan_rotation_failed"
                    )
                    return
                case .cancelled:
                    cancelActiveMotion(missionID: missionID, reason: "cancelled_while_scanning")
                    RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                        "mission": "\(missionID)",
                        "reason": "cancelled_while_scanning"
                    ])
                    return
                case .sessionGenerationChanged:
                    failOfflineObjectMission(
                        missionID: missionID,
                        message: failureMessage,
                        reason: "session_generation_changed"
                    )
                    return
                }
            }

            roomsSearched += 1
            RuntimeFileLog.append("mission_offline_room_searched", fields: [
                "mission": "\(missionID)",
                "room_id": topology?.snapshot.currentRoomID?.rawValue ?? "unavailable",
                "room_number": String(roomsSearched),
                "scan_steps": String(scanSteps),
                "target": intent.objectQuery,
                "target_found": resolvedGoal == nil ? "false" : "true",
            ])
            if intent.searchOtherRooms {
                missionTelemetry("mission_room_searched", [
                    "mission": "\(missionID)",
                    "reason": resolvedGoal == nil ? "target_not_found" : "target_found",
                    "room_id": topology?.snapshot.currentRoomID?.rawValue ?? "unavailable",
                    "room_number": String(roomsSearched),
                    "scan_steps": String(scanSteps),
                    "target": intent.objectQuery,
                ])
            }
            guard resolvedGoal == nil else { break }
            guard intent.searchOtherRooms else {
                failOfflineObjectMission(
                    missionID: missionID,
                    message: failureMessage,
                    reason: "target_not_found"
                )
                return
            }
            guard let topology,
                  sessionGeneration != nil,
                  startRoomID != nil,
                  roomsSearched < maximumRoomSearches,
                  crossedDoorwaySteps.count < maximumDoorwayCrossings else {
                failOfflineObjectMission(
                    missionID: missionID,
                    message: failureMessage,
                    reason: "search_exhausted"
                )
                return
            }

            switch await traverseNextDoorway(
                excluding: searchedCandidateIDs,
                missionID: missionID
            ) {
            case .crossed(let step):
                guard topology.snapshot.sessionGeneration == sessionGeneration else {
                    failOfflineObjectMission(
                        missionID: missionID,
                        message: failureMessage,
                        reason: "session_generation_changed"
                    )
                    return
                }
                crossedDoorwaySteps.append(step)
                let candidateID = topology.snapshot.doorways.first {
                    $0.id == step.doorwayID
                }?.candidateID
                if let candidateID {
                    searchedCandidateIDs.insert(candidateID)
                    RuntimeFileLog.append("mission_offline_doorway_marked_searched", fields: [
                        "mission": "\(missionID)",
                        "candidate_id": candidateID.rawValue,
                        "doorway_id": step.doorwayID.rawValue,
                    ])
                }
                RuntimeFileLog.append("mission_offline_doorway_crossed", fields: [
                    "mission": "\(missionID)",
                    "doorway_id": step.doorwayID.rawValue,
                    "from_room_id": step.fromRoomID.rawValue,
                    "to_room_id": step.toRoomID.rawValue,
                ])
                missionTelemetry("mission_doorway_crossed", [
                    "mission": "\(missionID)",
                    "reason": "searching_next_room",
                    "doorway_id": step.doorwayID.rawValue,
                    "from_room_id": step.fromRoomID.rawValue,
                    "to_room_id": step.toRoomID.rawValue,
                ])
                publishRoomTransitionState(.completed(
                    doorwayID: step.doorwayID,
                    roomID: step.toRoomID
                ))
                requiresFreshScanTurn = true
            case .exhausted(let reason):
                failOfflineObjectMission(
                    missionID: missionID,
                    message: failureMessage,
                    reason: reason == "session_generation_changed"
                        ? reason
                        : "search_exhausted"
                )
                return
            case .cancelled:
                cancelActiveMotion(missionID: missionID, reason: "cancelled_during_room_traversal")
                RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                    "mission": "\(missionID)",
                    "reason": "cancelled_during_room_traversal"
                ])
                return
            }
        }

        guard let goal = resolvedGoal else {
            failOfflineObjectMission(
                missionID: missionID,
                message: failureMessage,
                reason: "search_exhausted"
            )
            return
        }

        guard !missionCancellationDetected(missionID) else {
            cancelActiveMotion(missionID: missionID, reason: "cancelled_before_navigation")
            RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                "mission": "\(missionID)",
                "reason": "cancelled_before_navigation"
            ])
            return
        }

        setPhase(.acting, missionID: missionID)
        switch await approachVisualTarget(
            target,
            initialGoal: goal,
            missionID: missionID,
            expectedSessionGeneration: sessionGeneration
        ) {
        case .arrived:
            break
        case .failed(let reason):
            failOfflineObjectMission(
                missionID: missionID,
                message: reason,
                reason: "target_navigation_failed"
            )
            return
        case .cancelled:
            RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                "mission": "\(missionID)",
                "reason": "cancelled_during_navigation"
            ])
            return
        case .sessionGenerationChanged:
            failOfflineObjectMission(
                missionID: missionID,
                message: failureMessage,
                reason: "session_generation_changed"
            )
            return
        }

        RuntimeFileLog.append("mission_offline_fallback_target_arrived", fields: [
            "mission": "\(missionID)",
            "target": intent.objectQuery,
            "goal_x": String(format: "%.2f", goal.x),
            "goal_y": String(format: "%.2f", goal.y),
            "stop_distance": String(format: "%.2f", RoverConfig.visualTargetStopDistance)
        ])

        if sessionGenerationChanged(from: sessionGeneration) {
            failOfflineObjectMission(
                missionID: missionID,
                message: failureMessage,
                reason: "session_generation_changed"
            )
            return
        }

        if intent.shouldReturn {
            guard let start = memory.missionStartPose else {
                failOfflineObjectMission(
                    missionID: missionID,
                    message: failureMessage,
                    reason: "missing_start_pose"
                )
                return
            }
            if !crossedDoorwaySteps.isEmpty {
                guard let topology,
                      let startRoomID,
                      topology.snapshot.sessionGeneration == sessionGeneration,
                      let currentRoomID = topology.snapshot.currentRoomID,
                      let returnRoute = topology.shortestDoorwayPath(
                          from: currentRoomID,
                          to: startRoomID
                      ) else {
                    failOfflineObjectMission(
                        missionID: missionID,
                        message: failureMessage,
                        reason: "return_route_unavailable"
                    )
                    return
                }
                RuntimeFileLog.append("mission_offline_return_route_started", fields: [
                    "mission": "\(missionID)",
                    "from_room_id": currentRoomID.rawValue,
                    "to_room_id": startRoomID.rawValue,
                    "step_count": String(returnRoute.count),
                    "outbound_step_count": String(crossedDoorwaySteps.count),
                ])
                missionTelemetry("mission_return_route_started", [
                    "mission": "\(missionID)",
                    "reason": "explicit_return_requested",
                    "from_room_id": currentRoomID.rawValue,
                    "to_room_id": startRoomID.rawValue,
                    "step_count": String(returnRoute.count),
                ])
                for (index, expectedStep) in returnRoute.enumerated() {
                    guard topology.snapshot.sessionGeneration == sessionGeneration else {
                        failOfflineObjectMission(
                            missionID: missionID,
                            message: failureMessage,
                            reason: "session_generation_changed"
                        )
                        return
                    }
                    switch await traverseNextDoorway(
                        preferredDoorwayID: expectedStep.doorwayID,
                        missionID: missionID
                    ) {
                    case .crossed(let actualStep):
                        guard actualStep == expectedStep else {
                            failOfflineObjectMission(
                                missionID: missionID,
                                message: failureMessage,
                                reason: "return_route_changed"
                            )
                            return
                        }
                        RuntimeFileLog.append("mission_offline_return_route_step", fields: [
                            "mission": "\(missionID)",
                            "step": String(index + 1),
                            "doorway_id": actualStep.doorwayID.rawValue,
                            "from_room_id": actualStep.fromRoomID.rawValue,
                            "to_room_id": actualStep.toRoomID.rawValue,
                        ])
                        missionTelemetry("mission_return_route_step", [
                            "mission": "\(missionID)",
                            "reason": "returning_to_start_room",
                            "step": String(index + 1),
                            "doorway_id": actualStep.doorwayID.rawValue,
                            "from_room_id": actualStep.fromRoomID.rawValue,
                            "to_room_id": actualStep.toRoomID.rawValue,
                        ])
                        missionTelemetry("mission_doorway_crossed", [
                            "mission": "\(missionID)",
                            "reason": "returning_to_start_room",
                            "doorway_id": actualStep.doorwayID.rawValue,
                            "from_room_id": actualStep.fromRoomID.rawValue,
                            "to_room_id": actualStep.toRoomID.rawValue,
                        ])
                        publishRoomTransitionState(.completed(
                            doorwayID: actualStep.doorwayID,
                            roomID: actualStep.toRoomID
                        ))
                    case .exhausted(let reason):
                        failOfflineObjectMission(
                            missionID: missionID,
                            message: failureMessage,
                            reason: reason == "session_generation_changed"
                                ? reason
                                : "return_route_failed"
                        )
                        return
                    case .cancelled:
                        cancelActiveMotion(
                            missionID: missionID,
                            reason: "cancelled_during_return_route"
                        )
                        RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                            "mission": "\(missionID)",
                            "reason": "cancelled_during_return_route"
                        ])
                        return
                    }
                }
                RuntimeFileLog.append("mission_offline_return_route_completed", fields: [
                    "mission": "\(missionID)",
                    "room_id": startRoomID.rawValue,
                    "step_count": String(returnRoute.count),
                ])
                missionTelemetry("mission_return_route_completed", [
                    "mission": "\(missionID)",
                    "reason": "start_room_reached",
                    "room_id": startRoomID.rawValue,
                    "step_count": String(returnRoute.count),
                ])
            }
            RuntimeFileLog.append("mission_offline_fallback_return_started", fields: [
                "mission": "\(missionID)",
                "goal_x": String(format: "%.2f", start.position.x),
                "goal_y": String(format: "%.2f", start.position.y)
            ])
            switch await alignHeadingForReturn(
                to: start.position,
                missionID: missionID,
                expectedSessionGeneration: sessionGeneration
            ) {
            case .cancelled:
                RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                    "mission": "\(missionID)",
                    "reason": "cancelled_during_return_alignment"
                ])
                return
            case .failed:
                publishMissionTerminal(
                    { .failed(id: $0, command: $1, message: failureMessage) },
                    missionID: missionID
                )
                RuntimeFileLog.append("mission_offline_fallback_failed", fields: [
                    "mission": "\(missionID)",
                    "reason": "return_failed"
                ])
                return
            case .sessionGenerationChanged:
                failOfflineObjectMission(
                    missionID: missionID,
                    message: failureMessage,
                    reason: "session_generation_changed"
                )
                return
            case .arrived:
                break
            }
            switch await navigateReturn(
                to: start.position,
                missionID: missionID,
                expectedSessionGeneration: sessionGeneration
            ) {
            case .cancelled:
                RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                    "mission": "\(missionID)",
                    "reason": "cancelled_during_return_navigation"
                ])
                return
            case .failed:
                publishMissionTerminal(
                    { .failed(id: $0, command: $1, message: failureMessage) },
                    missionID: missionID
                )
                RuntimeFileLog.append("mission_offline_fallback_failed", fields: [
                    "mission": "\(missionID)",
                    "reason": "return_failed"
                ])
                return
            case .sessionGenerationChanged:
                failOfflineObjectMission(
                    missionID: missionID,
                    message: failureMessage,
                    reason: "session_generation_changed"
                )
                return
            case .arrived:
                break
            }
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_before_return_completion")
                RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                    "mission": "\(missionID)",
                    "reason": "cancelled_before_return_completion"
                ])
                return
            }
            RuntimeFileLog.append("mission_offline_fallback_return_completed", fields: [
                "mission": "\(missionID)"
            ])
        }

        setPhase(.idle, missionID: missionID)
        guard !missionCancellationDetected(missionID) else {
            cancelActiveMotion(missionID: missionID, reason: "cancelled_before_success")
            RuntimeFileLog.append("mission_offline_fallback_cancelled", fields: [
                "mission": "\(missionID)",
                "reason": "cancelled_before_success"
            ])
            return
        }
        publishMissionTerminal(
            { .succeeded(id: $0, command: $1, message: "Command completed.") },
            missionID: missionID
        )
        RuntimeFileLog.append("mission_offline_fallback_completed", fields: [
            "mission": "\(missionID)",
            "target": intent.objectQuery,
            "returned": intent.shouldReturn ? "true" : "false"
        ])
    }

    private func approachVisualTarget(
        _ target: NavigationTarget,
        initialGoal: Vec2,
        missionID: Int,
        expectedSessionGeneration: UInt64?
    ) async -> VisualTargetApproachOutcome {
        let maximumMapOpeningAttempts = 3
        var targetGoal = initialGoal
        var usedCandidateIDs = Set<String>()
        var stagingAttempts = 0
        var mapOpeningAttempts = 0
        var incrementalVisualRayAttempts = 0
        var stagingReason = "target_goal_unreachable"
        var shouldAttemptTargetGoal = true
        var hasRecoveredPlanningContext = false
        var hasPreparedInitialVisualPose = false

        while true {
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_before_target_approach")
                return .cancelled
            }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                await motion.stopAndWait()
                return .sessionGenerationChanged
            }

            if shouldAttemptTargetGoal {
                stagingReason = "target_goal_unreachable"
            }
            var startedTargetNavigation = false
            if shouldAttemptTargetGoal {
                switch reachableVisualNavigationGoal(for: targetGoal) {
                case .reachable(let navigationGoal):
                    motion.navigate(
                        to: navigationGoal,
                        stoppingAtForwardClearance: RoverConfig.visualTargetStopDistance
                    )
                    startedTargetNavigation = true
                case .rejected(.trackingUnstable) where !hasPreparedInitialVisualPose:
                    hasPreparedInitialVisualPose = true
                    switch await motion.preparePlanningContext(requiring: .stablePose) {
                    case .ready:
                        continue
                    case .trackingTimeout:
                        return .failed("AR tracking did not recover in time.")
                    case .meshTimeout:
                        preconditionFailure("stablePose cannot produce meshTimeout")
                    case .cancelled:
                        return .cancelled
                    case .sessionGenerationChanged:
                        return .sessionGenerationChanged
                    }
                case .rejected:
                    break
                }
            }
            if startedTargetNavigation {
                guard await waitForMotionToSettle(
                    missionID: missionID,
                    expectedSessionGeneration: expectedSessionGeneration
                ) else {
                    return sessionGenerationChanged(from: expectedSessionGeneration)
                        ? .sessionGenerationChanged
                        : .cancelled
                }
                if case .failed(let reason) = motion.state {
                    guard Self.isRecoverableTargetApproachFailure(reason) else {
                        return .failed(reason)
                    }
                    stagingReason = reason
                    RuntimeFileLog.append("mission_target_navigation_recovery", fields: [
                        "mission": "\(missionID)",
                        "reason": reason,
                        "recovery": "target_staging"
                    ])
                } else {
                    guard case .arrived = motion.state else {
                        return .failed("Target navigation did not complete.")
                    }
                    return .arrived
                }
            }
            shouldAttemptTargetGoal = true

            guard mapOpeningAttempts < maximumMapOpeningAttempts
                    || incrementalVisualRayAttempts < RoverConfig.visualTargetMaximumIncrementalSteps else {
                RuntimeFileLog.append("mission_target_staging_exhausted", fields: [
                    "mission": "\(missionID)",
                    "attempts": "\(stagingAttempts)",
                    "reason": "attempt_limit"
                ])
                return .failed("No safe route toward target.")
            }

            RuntimeFileLog.append("mission_target_staging_started", fields: [
                "mission": "\(missionID)",
                "attempt": "\(stagingAttempts + 1)",
                "target_goal_x": String(format: "%.2f", targetGoal.x),
                "target_goal_y": String(format: "%.2f", targetGoal.y),
                "reason": stagingReason
            ])
            var rejectionCounts: [NavigationGoalRejection: Int] = [:]
            guard let selected = rankedVisualTargetStagingCandidates(
                toward: targetGoal,
                excluding: usedCandidateIDs,
                allowMapOpenings: mapOpeningAttempts < maximumMapOpeningAttempts,
                allowIncrementalVisualRay: incrementalVisualRayAttempts
                    < RoverConfig.visualTargetMaximumIncrementalSteps,
                missionID: missionID,
                rejectionCounts: &rejectionCounts
            ).first else {
                let dominantRejection = Self.dominantRejection(in: rejectionCounts)
                if !hasRecoveredPlanningContext, dominantRejection != nil {
                    hasRecoveredPlanningContext = true
                    guard !missionCancellationDetected(missionID) else {
                        cancelActiveMotion(missionID: missionID, reason: "cancelled_before_target_reacquisition")
                        return .cancelled
                    }
                    guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                        await motion.stopAndWait()
                        return .sessionGenerationChanged
                    }
                    await perception.refreshDetections()
                    guard !missionCancellationDetected(missionID) else { return .cancelled }
                    guard let refreshedGoal = resolve(target, missionID: missionID) else {
                        return .failed("I couldn't reacquire the target before refreshing the map.")
                    }
                    guard !missionCancellationDetected(missionID) else {
                        cancelActiveMotion(missionID: missionID, reason: "cancelled_after_target_reacquisition")
                        return .cancelled
                    }
                    guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                        await motion.stopAndWait()
                        return .sessionGenerationChanged
                    }
                    targetGoal = refreshedGoal
                    switch await motion.preparePlanningContext(requiring: .refreshedTrustedMesh) {
                    case .ready:
                        guard !missionCancellationDetected(missionID) else {
                            cancelActiveMotion(missionID: missionID, reason: "cancelled_after_planning_readiness")
                            return .cancelled
                        }
                        guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                            await motion.stopAndWait()
                            return .sessionGenerationChanged
                        }
                        usedCandidateIDs.removeAll()
                        RuntimeFileLog.append("mission_target_planning_retry", fields: [
                            "mission": "\(missionID)",
                            "target": targetDescription(target),
                            "original_rejection_reason": dominantRejection?.rawValue ?? "unknown",
                            "inference_completed_before_readiness": "true"
                        ])
                        continue
                    case .trackingTimeout:
                        return .failed("AR tracking did not recover in time.")
                    case .meshTimeout:
                        return .failed("The navigation map did not refresh in time.")
                    case .cancelled:
                        return .cancelled
                    case .sessionGenerationChanged:
                        return .sessionGenerationChanged
                    }
                }
                RuntimeFileLog.append("mission_target_staging_exhausted", fields: [
                    "mission": "\(missionID)",
                    "attempts": "\(stagingAttempts)",
                    "reason": "no_progress_candidate",
                    "rejection_reason": dominantRejection?.rawValue ?? "none"
                ])
                return .failed("No safe route toward target.")
            }

            stagingAttempts += 1
            switch selected.kind {
            case .mapOpening:
                mapOpeningAttempts += 1
            case .incrementalVisualRay:
                incrementalVisualRayAttempts += 1
            }
            usedCandidateIDs.insert(selected.candidate.id)
            RuntimeFileLog.append("mission_target_staging_selected", fields: [
                "mission": "\(missionID)",
                "attempt": "\(stagingAttempts)",
                "candidate": selected.candidate.id,
                "source": selected.kind.rawValue,
                "goal_x": String(format: "%.2f", selected.candidate.worldPoint.x),
                "goal_y": String(format: "%.2f", selected.candidate.worldPoint.y),
                "target_progress": String(format: "%.2f", selected.targetProgress),
                "path_distance": String(format: "%.2f", selected.assessment.pathDistance)
            ])

            setPhase(.acting, missionID: missionID)
            let frameBeforeStaging = perception.frameSequence
            motion.navigate(to: selected.candidate.worldPoint)
            guard await waitForMotionToSettle(
                missionID: missionID,
                expectedSessionGeneration: expectedSessionGeneration
            ) else {
                return sessionGenerationChanged(from: expectedSessionGeneration)
                    ? .sessionGenerationChanged
                    : .cancelled
            }
            if case .failed(let reason) = motion.state {
                if Self.isRecoverableTargetApproachFailure(reason) {
                    stagingReason = reason
                    shouldAttemptTargetGoal = false
                    RuntimeFileLog.append("mission_target_staging_rejected", fields: [
                        "mission": "\(missionID)",
                        "attempt": "\(stagingAttempts)",
                        "candidate": selected.candidate.id,
                        "reason": reason,
                        "recovery": "next_candidate"
                    ])
                    continue
                }
                return .failed(reason)
            }
            guard case .arrived = motion.state else {
                return .failed("Target staging navigation did not complete.")
            }

            switch await waitForFreshTrackedPerceptionFrame(
                newerThan: frameBeforeStaging,
                expectedSessionGeneration: expectedSessionGeneration,
                missionID: missionID
            ) {
            case .ready:
                break
            case .timedOut:
                return .failed("The camera view did not refresh after moving.")
            case .cancelled:
                return .cancelled
            case .sessionGenerationChanged:
                return .sessionGenerationChanged
            }

            await perception.refreshDetections()
            guard !missionCancellationDetected(missionID) else { return .cancelled }
            var reacquiredGoal = resolve(target, missionID: missionID)
            if reacquiredGoal == nil {
                var scanSteps = 0
                switch await scanForUnresolvedVisualTarget(
                    target,
                    missionID: missionID,
                    scanSteps: &scanSteps,
                    expectedSessionGeneration: expectedSessionGeneration,
                    requireTrackedFrame: expectedSessionGeneration != nil
                ) {
                case .found(let goal):
                    reacquiredGoal = goal
                case .notFound:
                    break
                case .failed(let reason):
                    return .failed(reason)
                case .cancelled:
                    return .cancelled
                case .sessionGenerationChanged:
                    return .sessionGenerationChanged
                }
            }
            if let reacquiredGoal {
                targetGoal = reacquiredGoal
            }
            RuntimeFileLog.append("mission_target_staging_completed", fields: [
                "mission": "\(missionID)",
                "attempt": "\(stagingAttempts)",
                "candidate": selected.candidate.id,
                "target_reacquired": reacquiredGoal == nil ? "false" : "true",
                "target_goal_x": String(format: "%.2f", targetGoal.x),
                "target_goal_y": String(format: "%.2f", targetGoal.y)
            ])
        }
    }

    private func failOfflineObjectMission(
        missionID: Int,
        message: String,
        reason: String
    ) {
        guard isCurrentMission(missionID) else { return }
        motion.cancel()
        voice.speak(message.replacingOccurrences(of: "I’m", with: "I'm"))
        setPhase(.idle, missionID: missionID)
        publishMissionTerminal(
            { .failed(id: $0, command: $1, message: message) },
            missionID: missionID
        )
        RuntimeFileLog.append("mission_offline_fallback_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason
        ])
    }

    private func nextBrainAction(_ brain: RoverBrain, context: MissionContext) async throws -> BrainOutput {
        let race = BrainDecisionRace()
        let output = try await withTaskCancellationHandler {
            try await withCheckedThrowingContinuation { continuation in
                race.start(
                    brain: brain,
                    context: context,
                    timeout: brainDecisionTimeout,
                    continuation: continuation
                )
            }
        } onCancel: {
            Task { @MainActor in race.cancel() }
        }
        try Task.checkCancellation()
        return output
    }

    private func isCurrentMission(_ missionID: Int) -> Bool {
        missionGeneration == missionID
    }

    private func missionCancellationDetected(_ missionID: Int) -> Bool {
        Task.isCancelled || !isCurrentMission(missionID)
    }

    private func cancelActiveMotion(missionID: Int, reason: String) {
        guard isCurrentMission(missionID) else { return }
        motion.cancel()
        setPhase(.idle, missionID: missionID)
        RuntimeFileLog.append("mission_cancelled", fields: [
            "mission": "\(missionID)",
            "reason": reason
        ])
    }

    private func isEmergencyStopUtterance(_ utterance: String) -> Bool {
        let tokens = Set(utterance
            .lowercased()
            .components(separatedBy: CharacterSet.alphanumerics.inverted)
            .filter { !$0.isEmpty })
        return tokens.contains("stop")
            || tokens.contains("halt")
            || tokens.contains("cancel")
    }

    /// Updates action-history/no-op bookkeeping for one executed tick. Returns `true` if
    /// the bounded fallback fired and the mission loop should end now.
    @discardableResult
    private func recordTick(decision: RoverDecision, poseBefore: Vec2?, newObjects: Int, outcome: String) -> Bool {
        let poseAfter = perception.pose?.position
        let isNoOp = decision == lastDecision && distance(poseBefore, poseAfter) < 0.05 && newObjects == 0
        consecutiveNoOpTicks = isNoOp ? consecutiveNoOpTicks + 1 : 0
        lastDecision = decision
        appendRecentAction(outcome)
        if consecutiveNoOpTicks >= 2 {
            appendRecentAction(noOpWarningLine(consecutiveNoOpTicks))
        }
        if consecutiveNoOpTicks >= 5 {
            fireHeuristicFallback()
            return true
        }
        return false
    }

    private func appendRecentAction(_ line: String) {
        recentActionLines.append(line)
        if recentActionLines.count > recentActionsLimit {
            recentActionLines.removeFirst(recentActionLines.count - recentActionsLimit)
        }
    }

    private func noOpWarningLine(_ count: Int) -> String {
        "WARNING: your last \(count) actions were identical and produced no new information — " +
        "repeating again will not help. Pick a different action: explore an unexplored opening, " +
        "navigate somewhere new, or report your findings and finish with done."
    }

    /// Bounded fallback for a brain that keeps looping despite the escalating warnings —
    /// this is also the real IRL battery-preservation behavior (end the search rather than
    /// drain the pack for nothing), not just a test guard. Logged loudly (a plain stdout
    /// marker, not `EventLog` — that class lives in the test target only) so takes where it
    /// fired are identifiable; prefer retakes where the model exits on its own.
    private func fireHeuristicFallback() {
        print("HEURISTIC_FALLBACK_FIRED")
        voice.speak("I'm not finding anything new — ending the search. " + wrapUpFindings())
    }

    private func wrapUpFindings() -> String {
        let labels = memory.rememberedObjects.map { $0.label }
        return labels.isEmpty ? "I didn't find anything notable." : "Found: \(labels.joined(separator: ", "))."
    }

    private func distance(_ a: Vec2?, _ b: Vec2?) -> Double {
        guard let a, let b else { return .greatestFiniteMagnitude }
        return a.distance(to: b)
    }

    private func fmt(_ v: Double) -> String { String(format: "%.2f", v) }

    private func describeMotionOutcome(label: String, poseBefore: Vec2?) -> String {
        let poseAfter = perception.pose?.position
        if case .failed(let reason) = motion.state {
            return "\(label) → FAILED: \(reason)"
        }
        if motion.state == .arrived, let p = poseAfter {
            return "\(label) → arrived at (\(fmt(p.x)), \(fmt(p.y)))"
        }
        if distance(poseBefore, poseAfter) < 0.05 {
            return "\(label) → pose unchanged, no progress"
        }
        if let p = poseAfter {
            return "\(label) → moved to (\(fmt(p.x)), \(fmt(p.y)))"
        }
        return "\(label) → moved"
    }

    /// Once per tick, before thinking: fold what perception sees *right now* into
    /// persistent world memory, so the brain reasons over more than the current frame.
    private func updateWorldModel() {
        // Object permanence: pin every current detection to the nav plane.
        for object in perception.detectObjects() {
            if let world = perception.groundPoint(of: object) {
                memory.rememberObject(label: object.label, at: world)
            }
        }

        // Exploration candidates: match fresh frontiers to known openings by proximity so
        // ids (and visited status) stay stable across ticks; unmatched frontiers become
        // new candidates. Visited ones are kept even after their frontier disappears
        // (they're memory: "already checked, it was a hallway"); stale *unexplored* ones
        // are dropped so the brain isn't offered openings that no longer exist.
        var refreshed: [ExplorationCandidate] = []
        var matchedIds = Set<String>()
        for frontier in perception.explorationFrontiers() {
            if let existing = explorationCandidates.first(where: {
                !matchedIds.contains($0.id) &&
                $0.worldPoint.distance(to: frontier.centroid) < candidateMatchRadius
            }) {
                var updated = existing
                updated.worldPoint = frontier.centroid
                updated.widthMeters = frontier.widthMeters
                refreshed.append(updated)
                matchedIds.insert(existing.id)
            } else {
                refreshed.append(ExplorationCandidate(id: "opening_\(nextCandidateNumber)",
                                                      worldPoint: frontier.centroid,
                                                      widthMeters: frontier.widthMeters))
                nextCandidateNumber += 1
            }
        }
        let rememberedVisited = explorationCandidates.filter {
            $0.status == .visited && !matchedIds.contains($0.id)
        }
        explorationCandidates = refreshed + rememberedVisited

        // Being at (or driving right up to) an opening counts as having checked it.
        if let here = perception.pose?.position {
            for i in explorationCandidates.indices
            where explorationCandidates[i].worldPoint.distance(to: here) < visitedRadius {
                explorationCandidates[i].status = .visited
            }
        }
    }

    private func resolve(_ target: NavigationTarget, missionID: Int) -> Vec2? {
        switch target {
        case .worldPoint(let p): return p
        case .imagePoint(let p): return perception.unproject(normalizedPoint: p)
        case .visualQuery(let q):
            guard let object = lockedVisualTarget(query: q, missionID: missionID) else { return nil }
            return perception.unproject(object: object)
        }
    }

    private func effectiveNavigationTarget(_ target: NavigationTarget,
                                           lockedVisualQuery: inout String?,
                                           missionID: Int) -> NavigationTarget {
        guard case .visualQuery(let query) = target else { return target }
        let normalized = Self.normalizedVisualQuery(query)
        guard !normalized.isEmpty else { return target }
        if let locked = lockedVisualQuery {
            if normalized != Self.normalizedVisualQuery(locked) {
                RuntimeFileLog.append("mission_target_lock_kept", fields: [
                    "mission": "\(missionID)",
                    "target": locked,
                    "ignored": query
                ])
            }
            return .visualQuery(locked)
        }
        lockedVisualQuery = query
        RuntimeFileLog.append("mission_target_locked", fields: [
            "mission": "\(missionID)",
            "target": query,
            "threshold": String(format: "%.2f", visualTargetConfidenceThreshold)
        ])
        return target
    }

    private func lockedVisualTarget(query: String, missionID: Int) -> PerceivedObject? {
        let objects = perception.detectObjects()
        return lockedVisualTarget(query: query, objects: objects, missionID: missionID)
    }

    private func lockedVisualTarget(query: String,
                                    objects: [PerceivedObject],
                                    missionID: Int) -> PerceivedObject? {
        guard let match = Self.bestVisualTargetMatch(
            query: query,
            objects: objects,
            minimumConfidence: visualTargetConfidenceThreshold,
            missionID: missionID,
            telemetry: missionTelemetry
        ) else {
            RuntimeFileLog.append("mission_target_not_locked", fields: [
                "mission": "\(missionID)",
                "target": query,
                "visible": visibleObjectsLogSummary(objects),
                "threshold": String(format: "%.2f", visualTargetConfidenceThreshold)
            ])
            return nil
        }
        RuntimeFileLog.append("mission_target_match", fields: [
            "mission": "\(missionID)",
            "target": query,
            "label": match.label,
            "confidence": String(format: "%.2f", match.confidence),
            "direction": Self.visualTargetDirection(for: match.normalizedPoint),
            "x": String(format: "%.2f", match.normalizedPoint.x)
        ])
        return match
    }

    static func bestVisualTargetMatch(query: String,
                                      objects: [PerceivedObject],
                                      minimumConfidence: Float,
                                      missionID: Int? = nil,
                                      telemetry: @escaping MissionTelemetrySink = { event, fields in
                                          RuntimeFileLog.append(event, fields: fields)
                                      }) -> PerceivedObject? {
        guard !normalizedVisualQueryTokens(query).isEmpty else { return nil }
        let intent = visualTargetIntent(for: query)
        if let intent,
           let match = bestVisualTargetMatch(intent: intent,
                                             objects: objects,
                                             minimumConfidence: minimumConfidence,
                                             missionID: missionID,
                                             telemetry: telemetry) {
            return match
        }
        // The intent's exact-category check can't express compound detector labels
        // ("red_toolbox", "person_hat") or everyday words for a person ("the guy"), so fall
        // back to label specificity: the labels the query describes most specifically,
        // then the most confident of those. Only whole-token label matches count here (a
        // "black chair" must not match a "wheelchair"), and any colour the operator named
        // must be carried by the label itself or backed by colour evidence.
        let requestedColors = intent?.requestedColors ?? []
        let queryTokens = Set(normalizedVisualQueryTokens(query))
        let candidates = objects.filter { object in
            guard object.confidence >= minimumConfidence else { return false }
            let labelTokens = Set(normalizedVisualQueryTokens(object.label))
            guard queryTokens.contains(normalizedVisualQuery(object.label))
                    || !labelTokens.isDisjoint(with: queryTokens) else { return false }
            return requestedColors.allSatisfy { color in
                labelTokens.contains(color.rawValue)
                    || object.colorEvidence.contains { $0.color == color && $0.confidence >= 0.70 }
            }
        }
        let match = mostSpecificMatches(candidates, query: query, label: \.label)
            .max { $0.confidence < $1.confidence }
        if let match {
            telemetry("mission_attribute_match", [
                "mission": missionID.map(String.init) ?? "none",
                "target_label": intent.map { canonicalVisualLabel($0.targetLabel) } ?? normalizedVisualQuery(query),
                "requested_colors": requestedColorsDescription(requestedColors),
                "label": match.label,
                "object_confidence": String(format: "%.2f", match.confidence),
                "object_threshold": String(format: "%.2f", minimumConfidence),
                "reason": "label_specificity_match",
            ])
        }
        return match
    }

    static func bestVisualTargetMatch(
        intent: OfflineObjectMissionIntent,
        objects: [PerceivedObject],
        minimumConfidence: Float = 0.90,
        minimumColorConfidence: Float = 0.70,
        missionID: Int? = nil,
        telemetry: @escaping MissionTelemetrySink = { event, fields in
            RuntimeFileLog.append(event, fields: fields)
        }
    ) -> PerceivedObject? {
        let requestedColors = requestedColorsDescription(intent.requestedColors)
        let targetLabel = canonicalVisualLabel(intent.targetLabel)
        var matches: [PerceivedObject] = []

        for object in objects {
            let labelMatches = canonicalVisualLabel(object.label) == targetLabel
            let objectConfidenceMatches = object.confidence >= minimumConfidence
            let requestedEvidence = intent.requestedColors.map { requestedColor in
                object.colorEvidence
                    .filter { $0.color == requestedColor }
                    .map(\.confidence)
                    .max() ?? 0
            }
            let colorConfidence = requestedEvidence.min()
            let colorMatches = requestedEvidence.allSatisfy { $0 >= minimumColorConfidence }
            let fields = [
                "mission": missionID.map(String.init) ?? "none",
                "target_label": targetLabel,
                "requested_colors": requestedColors,
                "label": object.label,
                "object_confidence": String(format: "%.2f", object.confidence),
                "color_confidence": colorConfidence.map { String(format: "%.2f", $0) }
                    ?? "not_required",
                "object_threshold": String(format: "%.2f", minimumConfidence),
                "color_threshold": String(format: "%.2f", minimumColorConfidence),
                "reason": labelMatches && objectConfidenceMatches && colorMatches
                    ? "matched_required_attributes"
                    : "attributes_below_threshold",
            ]

            guard labelMatches, objectConfidenceMatches, colorMatches else {
                telemetry("mission_attribute_rejected", fields)
                continue
            }
            telemetry("mission_attribute_match", fields)
            matches.append(object)
        }

        return matches.max { $0.confidence < $1.confidence }
    }

    private static func visualTargetIntent(for query: String) -> OfflineObjectMissionIntent? {
        OfflineObjectMissionIntentParser.parse(query)
            ?? OfflineObjectMissionIntentParser.parse("go to \(query)")
    }

    private static func requestedColorsDescription(_ colors: Set<LocalObjectColor>) -> String {
        colors.map(\.rawValue).sorted().joined(separator: ",")
    }

    private static func canonicalVisualLabel(_ value: String) -> String {
        let tokens = normalizedVisualQueryTokens(value)
        guard !tokens.isEmpty else { return "" }
        let normalized = tokens.joined(separator: " ")
        switch normalized {
        case "monitor", "computer monitor", "display", "computer display",
             "screen", "computer screen", "tv", "laptop":
            return "screen"
        default:
            return normalized
        }
    }

    /// How much of a label the query accounts for: the number of the label's tokens the
    /// query names, 0 when `visualQueryMatches` rejects it. "the red toolbox" scores
    /// `red_toolbox` 2 and `blue_toolbox` 1, "the guy with the hat" scores `person_hat` 2
    /// and `person` 1.
    static func visualQueryMatchScore(query: String, label: String) -> Int {
        guard visualQueryMatches(query: query, label: label) else { return 0 }
        let queryTokens = Set(normalizedVisualQueryTokens(query))
        return max(1, normalizedVisualQueryTokens(label).filter(queryTokens.contains).count)
    }

    /// The matching items whose labels the query describes most specifically. Without
    /// this, any label sharing the category word matches — the plain `person` satisfies
    /// "the person with the hat" — and the higher-confidence one wins regardless of the
    /// attribute the operator actually named. Ties keep every tied item, so a query that
    /// names only the category ("the toolbox") still matches every instance.
    static func mostSpecificMatches<T>(_ items: [T], query: String, label: (T) -> String) -> [T] {
        let scored = items
            .map { ($0, visualQueryMatchScore(query: query, label: label($0))) }
            .filter { $0.1 > 0 }
        guard let best = scored.map(\.1).max() else { return [] }
        return scored.filter { $0.1 == best }.map(\.0)
    }

    /// Whether a detector label satisfies a free-text visual query, using the same
    /// tokenization and canonicalization `bestVisualTargetMatch` grounds navigation
    /// targets with. Exposed as a predicate (rather than kept inline) so `TargetTracker`
    /// selects follow candidates by exactly the same rule — a query that steers a
    /// `navigate` must steer a `follow` identically.
    static func visualQueryMatches(query: String, label: String) -> Bool {
        let queryTokens = normalizedVisualQueryTokens(query)
        guard !queryTokens.isEmpty else { return false }
        let normalizedLabel = normalizedVisualQuery(label)
        let labelTokens = normalizedVisualQueryTokens(label)
        return queryTokens.contains(normalizedLabel)
            || labelTokens.contains(where: { queryTokens.contains($0) })
            || queryTokens.contains(where: { normalizedLabel.contains($0) })
    }

    private static func normalizedVisualQuery(_ value: String) -> String {
        normalizedVisualQueryTokens(value).joined(separator: " ")
    }

    private static func normalizedVisualQueryTokens(_ value: String) -> [String] {
        let stopWords: Set<String> = [
            "a", "an", "and", "at", "find", "for", "go", "in", "look", "of", "on",
            "please", "see", "the", "to", "toward", "towards"
        ]
        return value
            .lowercased()
            .components(separatedBy: CharacterSet.alphanumerics.inverted)
            .filter { !$0.isEmpty && !stopWords.contains($0) }
            .map(canonicalVisualToken)
    }

    private static func canonicalVisualToken(_ token: String) -> String {
        switch token {
        case "fridge", "fridges":
            return "refrigerator"
        case "refrigerators":
            return "refrigerator"
        case "couch", "couches":
            return "sofa"
        case "sofas":
            return "sofa"
        case "tvs", "television", "televisions":
            return "tv"
        // The detector's only word for a human is COCO's "person"; operators say anything but.
        case "people", "persons", "guy", "guys", "man", "men", "woman", "women", "lady",
             "ladies", "boy", "boys", "girl", "girls", "kid", "kids", "child", "children",
             "someone", "somebody", "human", "humans":
            return "person"
        default:
            return token
        }
    }

    static func visualTargetDirection(for normalizedPoint: CGPoint) -> String {
        switch normalizedPoint.x {
        case ..<0.42:
            return "left"
        case 0.58...:
            return "right"
        default:
            return "ahead"
        }
    }

    private func scanForUnresolvedVisualTarget(_ target: NavigationTarget,
                                               missionID: Int,
                                               scanSteps: inout Int,
                                               requireNewFrameAfterInitialTurn: Bool = false,
                                               expectedSessionGeneration: UInt64? = nil,
                                               requireTrackedFrame: Bool = false) async
        -> VisualTargetScanResult {
        guard case .visualQuery(let query) = target else { return .notFound }
        var shouldEvaluateBeforeTurn = !requireNewFrameAfterInitialTurn
        var scanDirection = 1.0
        while scanSteps < maxVisualTargetScanSteps {
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                return .sessionGenerationChanged
            }
            if shouldEvaluateBeforeTurn {
                switch await waitForVisualTarget(
                    query: query,
                    missionID: missionID,
                    expectedSessionGeneration: expectedSessionGeneration,
                    requireTrackedFrame: requireTrackedFrame
                ) {
                case .found(let goal):
                    return .found(goal)
                case .noVisibleObjects:
                    break
                case .cancelled:
                    return .cancelled
                case .sessionGenerationChanged:
                    return .sessionGenerationChanged
                case .timedOut:
                    break
                }
            }
            shouldEvaluateBeforeTurn = true
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_before_scan_turn")
                return .cancelled
            }

            let attemptedStep = scanSteps + 1
            let angle = visualTargetScanAngle(forScanStep: attemptedStep) * scanDirection
            RuntimeFileLog.append("mission_target_scan_step", fields: [
                "mission": "\(missionID)",
                "target": query,
                "step": "\(attemptedStep)",
                "max": "\(maxVisualTargetScanSteps)",
                "angle": String(format: "%.0fdeg", angle * 180 / .pi)
            ])
            let frameBeforeTurn = perception.frameSequence
            let firstRotation = await rotateForScanRespectingCancellation(
                missionID: missionID, by: angle,
                expectedSessionGeneration: expectedSessionGeneration
            )
            switch firstRotation {
            case .completed:
                scanSteps = attemptedStep
            case .failed(let firstReason):
                let retryAngle = -angle
                RuntimeFileLog.append("mission_target_scan_rotation_retry", fields: [
                    "mission": "\(missionID)",
                    "target": query,
                    "step": "\(attemptedStep)",
                    "failed_angle": String(format: "%.0fdeg", angle * 180 / .pi),
                    "retry_angle": String(format: "%.0fdeg", retryAngle * 180 / .pi),
                    "reason": firstReason,
                ])
                switch await rotateForScanRespectingCancellation(
                    missionID: missionID, by: retryAngle,
                    expectedSessionGeneration: expectedSessionGeneration
                ) {
                case .completed:
                    scanDirection *= -1
                    scanSteps = attemptedStep
                case .failed(let retryReason):
                    RuntimeFileLog.append("mission_target_scan_rotation_failed", fields: [
                        "mission": "\(missionID)",
                        "target": query,
                        "step": "\(attemptedStep)",
                        "first_reason": firstReason,
                        "retry_reason": retryReason,
                    ])
                    return .failed(retryReason)
                case .cancelled:
                    return .cancelled
                case .sessionGenerationChanged:
                    return .sessionGenerationChanged
                }
            case .cancelled:
                return .cancelled
            case .sessionGenerationChanged:
                return .sessionGenerationChanged
            }
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_during_scan_turn")
                return .cancelled
            }

            switch await waitForVisualTarget(
                query: query,
                missionID: missionID,
                newerThanFrame: frameBeforeTurn,
                expectedSessionGeneration: expectedSessionGeneration,
                requireTrackedFrame: requireTrackedFrame
            ) {
            case .found(let goal):
                return .found(goal)
            case .cancelled:
                return .cancelled
            case .sessionGenerationChanged:
                return .sessionGenerationChanged
            case .noVisibleObjects, .timedOut:
                break
            }
        }

        RuntimeFileLog.append("mission_target_scan_exhausted", fields: [
            "mission": "\(missionID)",
            "target": query,
            "steps": "\(scanSteps)"
        ])
        return .notFound
    }

    private enum VisualTargetWaitResult {
        case found(Vec2)
        case timedOut
        case noVisibleObjects
        case cancelled
        case sessionGenerationChanged
    }

    private enum FreshTrackedFrameWaitOutcome {
        case ready
        case timedOut
        case cancelled
        case sessionGenerationChanged
    }

    private func waitForFreshTrackedPerceptionFrame(
        newerThan baseline: UInt64?,
        expectedSessionGeneration: UInt64?,
        missionID: Int
    ) async -> FreshTrackedFrameWaitOutcome {
        guard let expectedSessionGeneration else { return .ready }
        guard !missionCancellationDetected(missionID) else { return .cancelled }
        guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
            await motion.stopAndWait()
            return .sessionGenerationChanged
        }
        guard let baseline else {
            RuntimeFileLog.append("mission_fresh_frame_unavailable", fields: [
                "mission": "\(missionID)",
                "reason": "missing_frame_sequence"
            ])
            return .timedOut
        }
        let deadline = Date().addingTimeInterval(RoverConfig.scanFrameFreshnessTimeout)

        while Date() < deadline {
            guard !missionCancellationDetected(missionID) else { return .cancelled }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                await motion.stopAndWait()
                return .sessionGenerationChanged
            }
            if let observation = perception.latestObservation,
               observation.frameSequence > baseline,
               observation.trackingQuality == .normal,
               observation.sessionGeneration == expectedSessionGeneration {
                return .ready
            }
            if roomTransitionPollInterval > 0 {
                do {
                    try await Task.sleep(for: .seconds(roomTransitionPollInterval))
                } catch {
                    return .cancelled
                }
            } else {
                await Task.yield()
            }
        }

        return .timedOut
    }

    private func waitForVisualTarget(query: String,
                                     missionID: Int,
                                     newerThanFrame frameBaseline: UInt64? = nil,
                                     expectedSessionGeneration: UInt64? = nil,
                                     requireTrackedFrame: Bool = false) async -> VisualTargetWaitResult {
        if visualTargetScanDelay <= 0 {
            guard !missionCancellationDetected(missionID) else { return .cancelled }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                return .sessionGenerationChanged
            }
            await perception.refreshDetections()
            guard !missionCancellationDetected(missionID) else { return .cancelled }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else { return .sessionGenerationChanged }
            guard hasUsablePerceptionFrame(
                newerThan: frameBaseline,
                expectedSessionGeneration: expectedSessionGeneration,
                requireTrackedFrame: requireTrackedFrame
            ) else { return .timedOut }
            let objects = perception.detectObjects()
            guard !objects.isEmpty else { return .timedOut }
            if let object = lockedVisualTarget(query: query, objects: objects, missionID: missionID),
               let goal = perception.unproject(object: object) {
                return .found(goal)
            }
            return .timedOut
        }

        let deadline = Date().addingTimeInterval(visualTargetScanDelay)
        var sawVisibleObjects = false
        var didLogFreshFrameWait = false
        while Date() < deadline {
            guard !missionCancellationDetected(missionID) else { return .cancelled }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                return .sessionGenerationChanged
            }
            await perception.refreshDetections()
            guard !missionCancellationDetected(missionID) else { return .cancelled }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else { return .sessionGenerationChanged }
            guard hasUsablePerceptionFrame(
                newerThan: frameBaseline,
                expectedSessionGeneration: expectedSessionGeneration,
                requireTrackedFrame: requireTrackedFrame
            ) else {
                if !didLogFreshFrameWait {
                    RuntimeFileLog.append("mission_target_scan_wait_fresh_frame", fields: [
                        "mission": "\(missionID)",
                        "target": query,
                        "baseline": frameBaseline.map(String.init) ?? "none",
                        "current": perception.frameSequence.map(String.init) ?? "unavailable"
                    ])
                    didLogFreshFrameWait = true
                }
                do {
                    try await Task.sleep(for: .seconds(visualTargetPollInterval()))
                } catch is CancellationError {
                    return .cancelled
                } catch {
                    return .cancelled
                }
                continue
            }
            let objects = perception.detectObjects()
            guard !objects.isEmpty else {
                do {
                    try await Task.sleep(for: .seconds(visualTargetPollInterval()))
                } catch is CancellationError {
                    return .cancelled
                } catch {
                    return .cancelled
                }
                continue
            }
            sawVisibleObjects = true
            if let object = lockedVisualTarget(query: query, objects: objects, missionID: missionID),
               let goal = perception.unproject(object: object) {
                RuntimeFileLog.append("mission_target_scan_wait_match", fields: [
                    "mission": "\(missionID)",
                    "target": query
                ])
                return .found(goal)
            }
            do {
                try await Task.sleep(for: .seconds(visualTargetPollInterval()))
            } catch is CancellationError {
                return .cancelled
            } catch {
                return .cancelled
            }
        }

        if !sawVisibleObjects {
            RuntimeFileLog.append("mission_target_scan_wait_no_visible", fields: [
                "mission": "\(missionID)",
                "target": query,
                "seconds": String(format: "%.2f", visualTargetScanDelay)
            ])
            return .noVisibleObjects
        }
        RuntimeFileLog.append("mission_target_scan_wait_timeout", fields: [
            "mission": "\(missionID)",
            "target": query,
            "seconds": String(format: "%.2f", visualTargetScanDelay)
        ])
        return .timedOut
    }

    private func hasUsablePerceptionFrame(
        newerThan baseline: UInt64?,
        expectedSessionGeneration: UInt64?,
        requireTrackedFrame: Bool
    ) -> Bool {
        guard requireTrackedFrame else {
            return hasFreshPerceptionFrame(newerThan: baseline)
        }
        guard let observation = perception.latestObservation,
              observation.trackingQuality == .normal,
              observation.sessionGeneration == expectedSessionGeneration else {
            return false
        }
        guard let baseline else { return true }
        return observation.frameSequence > baseline
    }

    private func hasFreshPerceptionFrame(newerThan baseline: UInt64?) -> Bool {
        guard let baseline, let current = perception.frameSequence else { return true }
        return current > baseline
    }

    private func visualTargetPollInterval() -> TimeInterval {
        min(0.2, max(0.01, visualTargetScanDelay / 5))
    }

    private func visualTargetScanAngle(forScanStep step: Int) -> Double {
        guard step > 0 else { return 0 }
        return visualTargetScanAngle
    }

    private func nextUnexploredCandidate() -> ExplorationCandidate? {
        explorationCandidates
            .filter { $0.status == .unexplored }
            .max { $0.widthMeters < $1.widthMeters }
    }

    private func rankedVisualTargetStagingCandidates(
        toward targetGoal: Vec2,
        excluding usedCandidateIDs: Set<String>,
        allowMapOpenings: Bool,
        allowIncrementalVisualRay: Bool,
        missionID: Int,
        rejectionCounts: inout [NavigationGoalRejection: Int]
    ) -> [VisualTargetStagingCandidate] {
        updateWorldModel()
        guard let currentPosition = perception.pose?.position else { return [] }
        let targetDistanceBefore = currentPosition.distance(to: targetGoal)

        let mapOpeningCandidates = explorationCandidates
            .filter {
                allowMapOpenings
                    && $0.status == .unexplored
                    && !usedCandidateIDs.contains($0.id)
            }
            .compactMap { candidate in
                let assessment = motion.assessGoal(candidate.worldPoint)
                let targetDistanceAfter = candidate.worldPoint.distance(to: targetGoal)
                let ranked = VisualTargetStagingCandidate(
                    candidate: candidate,
                    assessment: assessment,
                    targetDistanceBefore: targetDistanceBefore,
                    targetDistanceAfter: targetDistanceAfter,
                    kind: .mapOpening
                )
                let accepted = assessment.isReachable && ranked.targetProgress >= 0.10
                if !assessment.isReachable {
                    rejectionCounts[assessment.rejectionReason ?? .noConnectedPath, default: 0] += 1
                }
                RuntimeFileLog.append("mission_target_staging_candidate", fields: [
                    "mission": "\(missionID)",
                    "candidate": candidate.id,
                    "source": VisualTargetStagingCandidate.Kind.mapOpening.rawValue,
                    "goal_x": String(format: "%.2f", candidate.worldPoint.x),
                    "goal_y": String(format: "%.2f", candidate.worldPoint.y),
                    "reachable": assessment.isReachable ? "true" : "false",
                    "target_progress": String(format: "%.2f", ranked.targetProgress),
                    "path_distance": String(format: "%.2f", assessment.pathDistance),
                    "accepted": accepted ? "true" : "false",
                    "rejection_reason": assessment.rejectionReason?.rawValue ?? "none"
                ])
                return accepted ? ranked : nil
            }
            .sorted { lhs, rhs in
                if abs(lhs.targetProgress - rhs.targetProgress) > 0.001 {
                    return lhs.targetProgress > rhs.targetProgress
                }
                return lhs.assessment.pathDistance < rhs.assessment.pathDistance
            }

        guard mapOpeningCandidates.isEmpty, allowIncrementalVisualRay else {
            return mapOpeningCandidates
        }
        return incrementalVisualTargetStagingCandidates(
            from: currentPosition,
            toward: targetGoal,
            excluding: usedCandidateIDs,
            missionID: missionID,
            rejectionCounts: &rejectionCounts
        )
    }

    private func incrementalVisualTargetStagingCandidates(
        from currentPosition: Vec2,
        toward targetGoal: Vec2,
        excluding usedCandidateIDs: Set<String>,
        missionID: Int,
        rejectionCounts: inout [NavigationGoalRejection: Int]
    ) -> [VisualTargetStagingCandidate] {
        let offset = targetGoal - currentPosition
        let targetDistanceBefore = offset.length
        let maximumStep = min(
            RoverConfig.visualTargetIncrementalStepDistance,
            targetDistanceBefore - RoverConfig.visualTargetStopDistance
        )
        guard maximumStep >= RoverConfig.visualTargetMinimumIncrementalStepDistance,
              targetDistanceBefore > 0 else {
            return []
        }

        let direction = offset * (1 / targetDistanceBefore)
        var candidates: [VisualTargetStagingCandidate] = []
        var stepDistance = maximumStep
        while stepDistance >= RoverConfig.visualTargetMinimumIncrementalStepDistance - 0.001 {
            let worldPoint = currentPosition + direction * stepDistance
            let candidateID = String(
                format: "visual_step_%.2f_%.2f",
                worldPoint.x,
                worldPoint.y
            )
            guard !usedCandidateIDs.contains(candidateID) else {
                stepDistance -= RoverConfig.visualTargetIncrementalStepDecrement
                continue
            }

            let candidate = ExplorationCandidate(
                id: candidateID,
                worldPoint: worldPoint,
                widthMeters: 0
            )
            let assessment = motion.assessGoal(worldPoint)
            let ranked = VisualTargetStagingCandidate(
                candidate: candidate,
                assessment: assessment,
                targetDistanceBefore: targetDistanceBefore,
                targetDistanceAfter: worldPoint.distance(to: targetGoal),
                kind: .incrementalVisualRay
            )
            let accepted = assessment.isReachable && ranked.targetProgress >= 0.10
            if !assessment.isReachable {
                rejectionCounts[assessment.rejectionReason ?? .noConnectedPath, default: 0] += 1
            }
            RuntimeFileLog.append("mission_target_staging_candidate", fields: [
                "mission": "\(missionID)",
                "candidate": candidateID,
                "source": VisualTargetStagingCandidate.Kind.incrementalVisualRay.rawValue,
                "goal_x": String(format: "%.2f", worldPoint.x),
                "goal_y": String(format: "%.2f", worldPoint.y),
                "reachable": assessment.isReachable ? "true" : "false",
                "target_progress": String(format: "%.2f", ranked.targetProgress),
                "path_distance": String(format: "%.2f", assessment.pathDistance),
                "accepted": accepted ? "true" : "false",
                "rejection_reason": assessment.rejectionReason?.rawValue ?? "none"
            ])
            if accepted {
                candidates.append(ranked)
            }
            stepDistance -= RoverConfig.visualTargetIncrementalStepDecrement
        }
        return candidates.sorted {
            if abs($0.targetProgress - $1.targetProgress) > 0.001 {
                return $0.targetProgress > $1.targetProgress
            }
            return $0.assessment.pathDistance < $1.assessment.pathDistance
        }
    }

    @discardableResult
    private func navigate(to goal: Vec2, for target: NavigationTarget) -> Bool {
        if case .visualQuery = target {
            guard case .reachable(let navigationGoal) = reachableVisualNavigationGoal(for: goal) else {
                return false
            }
            motion.navigate(to: navigationGoal,
                            stoppingAtForwardClearance: RoverConfig.visualTargetStopDistance)
            return true
        } else {
            motion.navigate(to: goal)
            return true
        }
    }

    private func reachableVisualNavigationGoal(for objectGoal: Vec2) -> VisualNavigationGoalResult {
        let objectAssessment = motion.assessGoal(objectGoal)
        guard !objectAssessment.isReachable else { return .reachable(objectGoal) }
        var rejectionCounts: [NavigationGoalRejection: Int] = [
            objectAssessment.rejectionReason ?? .noConnectedPath: 1
        ]
        guard let pose = perception.pose?.position else {
            RuntimeFileLog.append("mission_target_approach_goal_unreachable", fields: [
                "object_goal_x": String(format: "%.2f", objectGoal.x),
                "object_goal_y": String(format: "%.2f", objectGoal.y),
                "reason": "pose_unavailable"
            ])
            return .rejected(.missingPose)
        }
        let offset = objectGoal - pose
        let distance = offset.length
        guard distance > RoverConfig.visualTargetStopDistance else { return .reachable(pose) }

        let maximumStandOff = min(RoverConfig.visualTargetApproachDistance, distance)
        var standOff = RoverConfig.visualTargetStopDistance
        while standOff <= maximumStandOff + 0.001 {
            let approachGoal = objectGoal - offset * (standOff / distance)
            let assessment = motion.assessGoal(approachGoal)
            if assessment.isReachable {
                RuntimeFileLog.append("mission_target_approach_goal_adjusted", fields: [
                    "object_goal_x": String(format: "%.2f", objectGoal.x),
                    "object_goal_y": String(format: "%.2f", objectGoal.y),
                    "approach_goal_x": String(format: "%.2f", approachGoal.x),
                    "approach_goal_y": String(format: "%.2f", approachGoal.y),
                    "stand_off": String(format: "%.2f", standOff),
                    "path_distance": String(format: "%.2f", assessment.pathDistance)
                ])
                return .reachable(approachGoal)
            }
            rejectionCounts[assessment.rejectionReason ?? .noConnectedPath, default: 0] += 1
            RuntimeFileLog.append("mission_target_approach_goal_rejected", fields: [
                "approach_goal_x": String(format: "%.2f", approachGoal.x),
                "approach_goal_y": String(format: "%.2f", approachGoal.y),
                "stand_off": String(format: "%.2f", standOff),
                "rejection_reason": assessment.rejectionReason?.rawValue ?? "unknown"
            ])
            standOff += RoverConfig.visualTargetStandOffIncrement
        }

        RuntimeFileLog.append("mission_target_approach_goal_unreachable", fields: [
            "object_goal_x": String(format: "%.2f", objectGoal.x),
            "object_goal_y": String(format: "%.2f", objectGoal.y),
            "maximum_stand_off": String(format: "%.2f", maximumStandOff),
            "rejection_reason": objectAssessment.rejectionReason?.rawValue ?? "unknown"
        ])
        return .rejected(Self.dominantRejection(in: rejectionCounts) ?? .noConnectedPath)
    }

    private static func dominantRejection(
        in counts: [NavigationGoalRejection: Int]
    ) -> NavigationGoalRejection? {
        NavigationGoalRejection.allCases.max { lhs, rhs in
            let left = counts[lhs, default: 0]
            let right = counts[rhs, default: 0]
            if left == right {
                return NavigationGoalRejection.allCases.firstIndex(of: lhs)!
                    > NavigationGoalRejection.allCases.firstIndex(of: rhs)!
            }
            return left < right
        }.flatMap { counts[$0, default: 0] > 0 ? $0 : nil }
    }

    private func finishMissionIfVisualTargetArrived(_ target: NavigationTarget,
                                                    missionID: Int,
                                                    missionUtterance: String) -> Bool {
        guard case .arrived = motion.state, case .visualQuery = target else { return false }
        guard !Self.hasFollowUpIntent(missionUtterance), !Self.hasFollowUpIntent(plan ?? "") else {
            return false
        }
        setPhase(.idle, missionID: missionID)
        RuntimeFileLog.append("mission_target_navigation_done", fields: [
            "mission": "\(missionID)",
            "target": targetDescription(target)
        ])
        return true
    }

    private func completeReturnLegIfNeeded(after target: NavigationTarget,
                                           missionID: Int,
                                           missionUtterance: String,
                                           lockedVisualQuery: inout String?) async -> Bool {
        guard case .arrived = motion.state,
              case .visualQuery = target,
              Self.hasReturnIntent(missionUtterance),
              let start = memory.missionStartPose else {
            return false
        }

        lockedVisualQuery = nil
        RuntimeFileLog.append("mission_leg_completed", fields: [
            "mission": "\(missionID)",
            "leg": "primary_target",
            "target": targetDescription(target)
        ])
        RuntimeFileLog.append("mission_return_started", fields: [
            "mission": "\(missionID)",
            "goal_x": String(format: "%.2f", start.position.x),
            "goal_y": String(format: "%.2f", start.position.y)
        ])

        guard case .arrived = await alignHeadingForReturn(to: start.position, missionID: missionID) else {
            return true
        }
        guard case .arrived = await navigateReturn(to: start.position, missionID: missionID) else {
            return true
        }

        setPhase(.idle, missionID: missionID)
        plan = "Primary target reached; returned to mission start."
        RuntimeFileLog.append("mission_return_completed", fields: [
            "mission": "\(missionID)",
            "goal_x": String(format: "%.2f", start.position.x),
            "goal_y": String(format: "%.2f", start.position.y)
        ])
        return true
    }

    private func alignHeadingForReturn(
        to goal: Vec2,
        missionID: Int,
        expectedSessionGeneration: UInt64? = nil
    ) async -> ReturnLegOutcome {
        guard !missionCancellationDetected(missionID) else {
            cancelActiveMotion(missionID: missionID, reason: "cancelled_before_return_alignment")
            return .cancelled
        }
        guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
            await motion.stopAndWait()
            return .sessionGenerationChanged
        }
        guard let pose = perception.pose else {
            failReturnMission(missionID: missionID,
                              reason: "I lost my position before I could return.")
            return .failed
        }

        let offset = goal - pose.position
        guard offset.length > 0.05 else { return .arrived }

        let targetYaw = atan2(offset.y, offset.x)
        var remaining = normalizeAngle(targetYaw - pose.yaw)
        let maximumStep = max(abs(blockedHeadingRecoveryAngle), .pi / 180)
        var step = 0

        RuntimeFileLog.append("mission_return_alignment_started", fields: [
            "mission": "\(missionID)",
            "current_yaw_deg": String(format: "%.0f", pose.yaw * 180 / .pi),
            "target_yaw_deg": String(format: "%.0f", targetYaw * 180 / .pi),
            "turn_deg": String(format: "%.0f", remaining * 180 / .pi)
        ])

        while abs(remaining) > .pi / 180 {
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_during_return_alignment")
                return .cancelled
            }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                await motion.stopAndWait()
                return .sessionGenerationChanged
            }

            let angle = min(abs(remaining), maximumStep) * (remaining < 0 ? -1 : 1)
            step += 1
            RuntimeFileLog.append("mission_return_alignment_step", fields: [
                "mission": "\(missionID)",
                "step": "\(step)",
                "angle_deg": String(format: "%.0f", angle * 180 / .pi)
            ])
            switch await rotateForScanRespectingCancellation(
                missionID: missionID, by: angle,
                expectedSessionGeneration: expectedSessionGeneration
            ) {
            case .completed:
                break
            case .failed(let reason):
                failReturnMission(missionID: missionID, reason: reason)
                return .failed
            case .cancelled:
                return .cancelled
            case .sessionGenerationChanged:
                return .sessionGenerationChanged
            }

            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_during_return_alignment")
                return .cancelled
            }

            if case .failed(let reason) = motion.state {
                failReturnMission(missionID: missionID, reason: reason)
                return .failed
            }
            remaining -= angle
        }

        RuntimeFileLog.append("mission_return_alignment_completed", fields: [
            "mission": "\(missionID)",
            "steps": "\(step)"
        ])
        return .arrived
    }

    private func rotateForScanRespectingCancellation(
        missionID: Int,
        by angle: Double,
        expectedSessionGeneration: UInt64? = nil
    ) async -> ScanRotationOutcome {
        guard !missionCancellationDetected(missionID) else {
            cancelActiveMotion(missionID: missionID, reason: "cancelled_before_scan_rotation")
            return .cancelled
        }
        guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
            motion.cancel()
            return .sessionGenerationChanged
        }
        let race = ScanRotationRace()
        return await withTaskCancellationHandler {
            await withCheckedContinuation { continuation in
                race.start(
                    motion: motion,
                    topology: roomTopology,
                    expectedSessionGeneration: expectedSessionGeneration,
                    ownsMotion: { [weak self] in self?.isCurrentMission(missionID) ?? false },
                    angle: angle,
                    continuation: continuation
                )
            }
        } onCancel: {
            Task { @MainActor in
                race.cancel()
            }
        }
    }

    private func navigateReturn(
        to goal: Vec2,
        missionID: Int,
        expectedSessionGeneration: UInt64? = nil
    ) async -> ReturnLegOutcome {
        let maximumAttempts = 2

        for attempt in 1...maximumAttempts {
            guard !missionCancellationDetected(missionID) else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_before_return_navigation")
                return .cancelled
            }
            guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
                await motion.stopAndWait()
                return .sessionGenerationChanged
            }
            RuntimeFileLog.append("mission_return_navigation_attempt", fields: [
                "mission": "\(missionID)",
                "attempt": "\(attempt)",
                "max": "\(maximumAttempts)"
            ])
            motion.navigate(to: goal)
            guard await waitForMotionToSettle(
                missionID: missionID,
                expectedSessionGeneration: expectedSessionGeneration
            ) else {
                if sessionGenerationChanged(from: expectedSessionGeneration) {
                    return .sessionGenerationChanged
                }
                return .cancelled
            }

            if case .arrived = motion.state { return .arrived }

            if case .failed(let reason) = motion.state,
               Self.isBlockedHeading(reason),
               attempt < maximumAttempts {
                RuntimeFileLog.append("mission_return_recovery", fields: [
                    "mission": "\(missionID)",
                    "attempt": "\(attempt)",
                    "reason": reason,
                    "recovery": recoveryDescription
                ])
                switch await rotateForBlockedHeadingRecovery(
                    missionID: missionID,
                    expectedSessionGeneration: expectedSessionGeneration
                ) {
                case .completed:
                    break
                case .cancelled:
                    cancelActiveMotion(missionID: missionID, reason: "cancelled_during_return_recovery")
                    return .cancelled
                case .sessionGenerationChanged:
                    return .sessionGenerationChanged
                }
                guard !missionCancellationDetected(missionID) else {
                    cancelActiveMotion(missionID: missionID, reason: "cancelled_during_return_recovery")
                    return .cancelled
                }
                if case .failed(let recoveryReason) = motion.state {
                    failReturnMission(missionID: missionID, reason: recoveryReason)
                    return .failed
                }
                continue
            }

            let reason: String
            if case .failed(let failureReason) = motion.state {
                reason = failureReason
            } else {
                reason = "Return navigation stopped before reaching the start."
            }
            failReturnMission(missionID: missionID, reason: reason)
            return .failed
        }

        failReturnMission(missionID: missionID,
                          reason: "I couldn't find a clear route back to the start.")
        return .failed
    }

    private func failReturnMission(missionID: Int, reason: String) {
        guard isCurrentMission(missionID) else { return }
        motion.cancel()
        voice.speak(reason)
        setPhase(.idle, missionID: missionID)
        RuntimeFileLog.append("mission_motion_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason
        ])
        RuntimeFileLog.append("mission_return_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason
        ])
    }

    private func failScanRotationMission(missionID: Int, reason: String) {
        guard isCurrentMission(missionID) else { return }
        motion.cancel()
        voice.speak(reason)
        publishMissionTerminal(
            { .failed(id: $0, command: $1, message: reason) },
            missionID: missionID
        )
        setPhase(.idle, missionID: missionID)
        RuntimeFileLog.append("mission_target_scan_rotation_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason,
        ])
    }

    private func failTargetNavigationMission(missionID: Int, reason: String) {
        guard isCurrentMission(missionID) else { return }
        motion.cancel()
        voice.speak(reason)
        publishMissionTerminal(
            { .failed(id: $0, command: $1, message: reason) },
            missionID: missionID
        )
        setPhase(.idle, missionID: missionID)
        RuntimeFileLog.append("mission_target_navigation_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason,
        ])
    }

    private func recoverVisualNavigation(_ target: NavigationTarget,
                                         missionID: Int,
                                         scanSteps: inout Int,
                                         missionUtterance: String) async -> Bool {
        guard case .visualQuery(let query) = target,
              case .failed(let reason) = motion.state,
              Self.isNavigationStalled(reason) || Self.isBlockedHeading(reason) else {
            return false
        }

        RuntimeFileLog.append("mission_visual_navigation_recovery", fields: [
            "mission": "\(missionID)",
            "target": query,
            "reason": reason,
            "recovery": "scan_30deg"
        ])

        switch await scanForUnresolvedVisualTarget(target,
                                                   missionID: missionID,
                                                   scanSteps: &scanSteps) {
        case .found(let freshGoal):
            scanSteps = 0
            RuntimeFileLog.append("mission_target_reacquired", fields: [
                "mission": "\(missionID)",
                "target": query,
                "goal_x": String(format: "%.2f", freshGoal.x),
                "goal_y": String(format: "%.2f", freshGoal.y)
            ])
            navigate(to: freshGoal, for: target)
            await waitForMotionToSettle(missionID: missionID)
            guard isCurrentMission(missionID) else {
                setPhase(.idle, missionID: missionID)
                RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                return true
            }
            if await recoverOrStopMissionIfMotionFailed(missionID: missionID,
                                                        recoverFromBlockedHeading: true) {
                return true
            }
            return finishMissionIfVisualTargetArrived(target,
                                                       missionID: missionID,
                                                       missionUtterance: missionUtterance)

        case .notFound:
            motion.cancel()
            voice.speak("I couldn't find the target after looking around.")
            setPhase(.idle, missionID: missionID)
            RuntimeFileLog.append("mission_target_recovery_exhausted", fields: [
                "mission": "\(missionID)",
                "target": query
            ])
            return true

        case .failed(let reason):
            failScanRotationMission(missionID: missionID, reason: reason)
            return true

        case .cancelled:
            setPhase(.idle, missionID: missionID)
            RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
            return true

        case .sessionGenerationChanged:
            setPhase(.idle, missionID: missionID)
            RuntimeFileLog.append("mission_cancelled", fields: [
                "mission": "\(missionID)",
                "reason": "session_generation_changed"
            ])
            return true
        }
    }

    private static func hasFollowUpIntent(_ text: String) -> Bool {
        let lowercased = text.lowercased()
        return hasReturnIntent(lowercased)
            || lowercased.contains(" then ")
            || lowercased.hasPrefix("then ")
    }

    private static func hasReturnIntent(_ text: String) -> Bool {
        let lowercased = text.lowercased()
        let trimmed = lowercased.trimmingCharacters(in: .whitespacesAndNewlines)
        return lowercased.contains("come back")
            || lowercased.contains("and back")
            || lowercased.contains("then go back")
            || lowercased.contains("and go back")
            || lowercased.contains("then return")
            || lowercased.contains("and return")
            || lowercased.contains("return to start")
            || lowercased.contains("return to the start")
            || lowercased.contains("return here")
            || lowercased.contains("return home")
            || trimmed == "go back"
            || trimmed == "return"
    }

    @discardableResult
    private func waitForMotionToSettle(
        missionID: Int? = nil,
        expectedSessionGeneration: UInt64? = nil
    ) async -> Bool {
        RuntimeFileLog.append("mission_motion_wait_started", fields: [
            "state": motion.state.description
        ])
        while motion.state.isMotionActive {
            if let missionID, !isCurrentMission(missionID) { return false }
            if sessionGenerationChanged(from: expectedSessionGeneration) {
                await motion.stopAndWait()
                return false
            }
            if let missionID, missionCancellationDetected(missionID) {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_while_motion_active")
                return false
            }
            do {
                try await Task.sleep(for: .seconds(RoverConfig.commandInterval))
            } catch is CancellationError {
                if let missionID {
                    cancelActiveMotion(missionID: missionID, reason: "cancelled_while_waiting_for_motion")
                } else {
                    motion.cancel()
                }
                return false
            } catch {
                if let missionID {
                    cancelActiveMotion(missionID: missionID, reason: "motion_wait_interrupted")
                } else {
                    motion.cancel()
                }
                return false
            }
        }
        if let missionID, !isCurrentMission(missionID) { return false }
        if sessionGenerationChanged(from: expectedSessionGeneration) {
            await motion.stopAndWait()
            return false
        }
        if let missionID, missionCancellationDetected(missionID) {
            cancelActiveMotion(missionID: missionID, reason: "cancelled_after_motion_settled")
            return false
        }
        RuntimeFileLog.append("motion_settled", fields: ["state": motion.state.description])
        return true
    }

    private func sessionGenerationChanged(from expected: UInt64?) -> Bool {
        guard let expected else { return false }
        return roomTopology?.snapshot.sessionGeneration != expected
    }

    private func recoverOrStopMissionIfMotionFailed(missionID: Int,
                                                    recoverFromBlockedHeading: Bool = false,
                                                    blockedCandidateId: String? = nil) async -> Bool {
        guard case .failed(let reason) = motion.state else { return false }
        if recoverFromBlockedHeading, Self.isBlockedHeading(reason) {
            if let blockedCandidateId {
                markExplorationCandidateVisited(blockedCandidateId, reason: reason)
            }
            RuntimeFileLog.append("mission_blocked_heading", fields: [
                "mission": "\(missionID)",
                "reason": reason,
                "recovery": recoveryDescription
            ])
            guard await rotateForBlockedHeadingRecovery(missionID: missionID) == .completed else {
                cancelActiveMotion(missionID: missionID, reason: "cancelled_during_blocked_recovery")
                return true
            }
            guard case .failed(let recoveryReason) = motion.state else { return false }
            voice.speak(recoveryReason)
            setPhase(.idle, missionID: missionID)
            RuntimeFileLog.append("mission_motion_failed", fields: [
                "mission": "\(missionID)",
                "reason": recoveryReason
            ])
            return true
        }
        voice.speak(reason)
        setPhase(.idle, missionID: missionID)
        RuntimeFileLog.append("mission_motion_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason
        ])
        return true
    }

    private func markExplorationCandidateVisited(
        _ id: String,
        reason: String,
        event: String = "mission_exploration_candidate_blocked"
    ) {
        guard let index = explorationCandidates.firstIndex(where: { $0.id == id }) else { return }
        explorationCandidates[index].status = .visited
        RuntimeFileLog.append(event, fields: [
            "candidate": id,
            "reason": reason,
            "status": ExplorationCandidate.Status.visited.rawValue
        ])
    }

    private var recoveryDescription: String {
        String(format: "rotate_%.0fdeg", blockedHeadingRecoveryAngle * 180 / .pi)
    }

    private func rotateForBlockedHeadingRecovery(
        missionID: Int,
        expectedSessionGeneration: UInt64? = nil
    ) async -> BlockedHeadingRecoveryOutcome {
        guard !missionCancellationDetected(missionID) else {
            cancelActiveMotion(missionID: missionID, reason: "cancelled_before_blocked_recovery")
            return .cancelled
        }
        guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
            await motion.stopAndWait()
            return .sessionGenerationChanged
        }
        let race = BlockedHeadingRecoveryRace()
        let outcome = await withTaskCancellationHandler {
            await withCheckedContinuation { continuation in
                race.start(
                    motion: motion,
                    topology: roomTopology,
                    expectedSessionGeneration: expectedSessionGeneration,
                    ownsMotion: { [weak self] in self?.isCurrentMission(missionID) ?? false },
                    missionIsCancelled: { [weak self] in
                        self?.missionCancellationDetected(missionID) ?? true
                    },
                    angle: blockedHeadingRecoveryAngle,
                    timeout: blockedHeadingRecoveryTimeout,
                    continuation: continuation
                )
            }
        } onCancel: {
            Task { @MainActor in race.cancel() }
        }

        switch outcome {
        case .cancelled:
            cancelActiveMotion(
                missionID: missionID,
                reason: "cancelled_during_blocked_recovery"
            )
            return .cancelled
        case .sessionGenerationChanged:
            cancelActiveMotion(
                missionID: missionID,
                reason: "session_generation_changed_during_blocked_recovery"
            )
            return .sessionGenerationChanged
        case .timedOut:
            RuntimeFileLog.append("mission_blocked_heading_recovery_timeout", fields: [
                "mission": "\(missionID)",
                "recovery": recoveryDescription,
                "timeout": String(format: "%.2f", blockedHeadingRecoveryTimeout)
            ])
        case .completed:
            RuntimeFileLog.append("mission_blocked_heading_recovery_settled", fields: [
                "mission": "\(missionID)",
                "recovery": recoveryDescription
            ])
        }

        guard !missionCancellationDetected(missionID) else {
            cancelActiveMotion(missionID: missionID, reason: "cancelled_after_blocked_recovery_settled")
            return .cancelled
        }
        guard !sessionGenerationChanged(from: expectedSessionGeneration) else {
            await motion.stopAndWait()
            return .sessionGenerationChanged
        }
        return .completed
    }

    private static func isBlockedHeading(_ reason: String) -> Bool {
        reason.localizedCaseInsensitiveContains("Obstacle ahead")
            || reason.localizedCaseInsensitiveContains("Depth safety stop: stop")
    }

    private static func isRecoverableTargetApproachFailure(_ reason: String) -> Bool {
        isBlockedHeading(reason)
            || reason.localizedCaseInsensitiveContains("safely see the space needed to turn")
            || reason.localizedCaseInsensitiveContains("Depth safety stop while rotating")
    }

    private static func isNavigationStalled(_ reason: String) -> Bool {
        reason.localizedCaseInsensitiveContains("Navigation stalled")
    }

    private func makeContext(utterance: String?, missionID: Int) -> MissionContext {
        MissionContext(missionID: missionID,
                       utterance: utterance,
                       frameJPEG: perception.capturedFrameJPEG(),
                       visibleObjects: perception.detectObjects(),
                       pose: perception.pose,
                       navState: motion.state,
                       memory: memory,
                       explorationCandidates: explorationCandidates,
                       plan: plan,
                       lastAnswerWasInconclusive: lastAnswerWasInconclusive,
                       batteryPercent: battery?.percent,
                       teamContext: teamRadio?.currentTeamContext(),
                       followState: followStateDescription(),
                       recentActions: recentActionLines)
    }

    private func visibleObjectsLogSummary(_ objects: [PerceivedObject]) -> String {
        guard !objects.isEmpty else { return "none" }
        return objects
            .prefix(8)
            .map { object in
                "\(object.label.replacingOccurrences(of: " ", with: "_")):\(String(format: "%.2f", object.confidence))"
            }
            .joined(separator: ",")
    }

    /// The follow this mission was for has ended (and said why). With no brain to ask
    /// what next, that is the end of the mission — not an apology for not thinking.
    private var followEndedWithNothingElseToDo: Bool {
        guard case .follow = lastDecision, case .ended = follower?.followState else { return false }
        return true
    }

    private var isFollowing: Bool {
        guard let follower, follower.followingSpec != nil else { return false }
        if case .ended = follower.followState { return false }
        return true
    }

    /// While a follow is running, a brain that is missing, unavailable, errors or times
    /// out means "carry on", not "give up": the follow needs no reasoning to continue, and
    /// the operator can still say stop. `nil` when nothing is being followed.
    private func keepFollowingOutput(missionID: Int, error: Error? = nil,
                                     context: MissionContext? = nil) -> BrainOutput? {
        guard isFollowing, let spec = follower?.followingSpec else { return nil }
        if let error, let context { brainErrorLogger(error, context) }
        RuntimeFileLog.append("mission_follow_without_brain", fields: [
            "mission": "\(missionID)",
            "query": spec.query,
            "error": error?.localizedDescription ?? "no brain"
        ])
        return BrainOutput(decision: .follow(.visualQuery(spec.query)))
    }

    /// The target of an utterance that is nothing but a request to follow a person —
    /// "follow me", "come with me", "follow the guy with the hat" — or `nil` for anything
    /// else, which goes to the brain. Deliberately narrow: compound requests ("follow me
    /// and then come back"), and follows of things that aren't people ("follow the
    /// hallway"), are the brain's to interpret.
    static func directFollowQuery(_ utterance: String) -> String? {
        var text = " " + utterance.lowercased()
            .components(separatedBy: CharacterSet.letters.union(.whitespaces).union(CharacterSet(charactersIn: "'")).inverted)
            .joined(separator: " ")
            .split(separator: " ").joined(separator: " ") + " "
        for filler in [" hey ", " ok ", " okay ", " rover ", " phrover ", " please ", " now ",
                       " can you ", " could you ", " would you ", " will you ", " just "] {
            text = text.replacingOccurrences(of: filler, with: " ")
        }
        text = text.trimmingCharacters(in: .whitespaces)
        if ["and", "then", "until", "after", "unless"].contains(where: {
            text.split(separator: " ").map(String.init).contains($0)
        }) { return nil }
        if ["tag along", "come along", "follow"].contains(text) { return "person" }

        let verbs = ["follow", "come with", "walk with", "stay with", "stick with", "go with",
                     "keep up with", "tag along with", "come along with", "stay behind", "walk behind"]
        guard let verb = verbs.first(where: { text.hasPrefix($0 + " ") }) else { return nil }
        var target = String(text.dropFirst(verb.count + 1))
        // "follow me to the gate": where they are going doesn't change who to follow.
        for cut in [" to ", " into ", " toward ", " towards ", " over to ", " around "] {
            if let range = target.range(of: cut) { target = String(target[..<range.lowerBound]) }
        }
        target = target.trimmingCharacters(in: .whitespaces)
        if ["me", "us", "him", "her", "them", "this guy", "that guy"].contains(target) { return "person" }
        guard visualQueryMatches(query: target, label: "person") else { return nil }
        return target
    }

    /// "the person with the hat" as it would be said aloud: the brain's description,
    /// with "the" added only if it doesn't already open with a determiner.
    static func spokenTarget(_ query: String) -> String {
        let trimmed = query.trimmingCharacters(in: .whitespacesAndNewlines)
        let first = trimmed.split(separator: " ").first.map { $0.lowercased() } ?? ""
        return ["the", "a", "an", "that", "this", "my", "your", "him", "her", "them", "me"].contains(first)
            ? trimmed : "the \(trimmed)"
    }

    /// Any decision that drives the base itself ends a running follow first: two
    /// controllers writing wheel commands is the one thing neither can recover from. The
    /// mirror of `startOrContinueFollowing` cancelling `motion` before it starts a follow.
    private func stopFollowingBeforeOtherMotion(_ decision: RoverDecision, missionID: Int) {
        guard let follower, follower.followingSpec != nil else { return }
        follower.stopFollowing()
        RuntimeFileLog.append("mission_follow_stopped", fields: [
            "mission": "\(missionID)",
            "reason": decisionDescription(decision)
        ])
    }

    /// "following the guy with the hat" — the target, not just the state. A brain told
    /// only "following" re-states the follow in its own words on its next look-in, and a
    /// re-statement that drops the attribute ("the person") grounds on every person in view.
    private func followStateDescription() -> String? {
        guard let follower, let spec = follower.followingSpec else { return nil }
        return "\(follower.followState.description) \(spec.query)"
    }

    /// Start (or keep) following whatever `target` names. Returns the tick outcome line, or
    /// `nil` if the mission was cancelled while waiting.
    ///
    /// Following runs *between* ticks: this starts it and then blocks only long enough to
    /// give the brain a periodic look-in, so the brain can narrate, ask, or break off
    /// without the mission loop burning its whole tick budget spinning. Repeating `.follow`
    /// for the same target deliberately does nothing — a brain re-stating its intent must
    /// not restart the lock and lose the track it already has.
    private func startOrContinueFollowing(_ target: NavigationTarget, missionID: Int) async -> String? {
        guard let follower else {
            voice.speak("I can't follow anything right now.")
            return "follow → unavailable"
        }
        guard let resolved = followSpec(for: target) else {
            voice.speak("I couldn't tell who to follow.")
            return "follow → couldn't resolve target"
        }

        let alreadyFollowing = follower.followingSpec.map {
            Self.visualQueryMatches(query: $0.query, label: resolved.spec.query)
        } ?? false
        if !alreadyFollowing {
            // Never drive a goal and hold a stand-off at the same time — two controllers
            // writing wheel commands is the one thing neither of them can recover from.
            motion.cancel()
            follower.follow(resolved.spec, seed: resolved.seed)
            voice.speak("Following \(Self.spokenTarget(resolved.spec.query)). Say stop when you want me to hold.")
            RuntimeFileLog.append("mission_follow_started", fields: [
                "mission": "\(missionID)",
                "query": resolved.spec.query,
                "seed": resolved.seed.map { String(format: "%.2f,%.2f", $0.x, $0.y) } ?? "none"
            ])
        }

        await waitWhileFollowing(missionID: missionID)
        guard isCurrentMission(missionID) else { return nil }

        if case .ended(let message) = follower.followState {
            voice.speak(message)
            return "follow(\(resolved.spec.query)) → ended: \(message)"
        }
        return "follow(\(resolved.spec.query)) → \(follower.followState.description)"
    }

    /// Block while a follow runs, up to one brain review interval, returning early the
    /// moment it ends. Without this the mission loop would re-ask the brain at full speed
    /// and exhaust its tick budget within seconds of starting a mode that is meant to run
    /// for minutes.
    private func waitWhileFollowing(missionID: Int) async {
        guard let follower else { return }
        let deadline = Date().addingTimeInterval(followReviewInterval)
        while Date() < deadline {
            if Task.isCancelled || !isCurrentMission(missionID) { return }
            if follower.followingSpec == nil { return }
            if case .ended = follower.followState { return }
            try? await Task.sleep(for: .seconds(0.25))
        }
    }

    /// Resolve a navigation target into something followable.
    ///
    /// An `.imagePoint` from a cloud brain is treated as a *selector*, not a coordinate:
    /// the point was chosen against a frame that is a round-trip old, so by the time it
    /// arrives the pixel may be the wall a walking person was standing in front of. What
    /// survives the delay is which object was meant, so the point picks the label and seeds
    /// the tracker, and association in the current frame does the rest.
    private func followSpec(for target: NavigationTarget) -> (spec: TargetSpec, seed: Vec2?)? {
        switch target {
        case .visualQuery(let query):
            let trimmed = query.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !trimmed.isEmpty else { return nil }
            return (TargetSpec(query: trimmed), perception.groundObject(query: trimmed)
                        .flatMap { perception.unproject(normalizedPoint: $0) })
        case .imagePoint(let point):
            guard let seed = perception.unproject(normalizedPoint: point) else { return nil }
            guard let label = nearestDetectedLabel(toWorldPoint: seed) else { return nil }
            return (TargetSpec(query: label), seed)
        case .worldPoint(let point):
            guard let label = nearestDetectedLabel(toWorldPoint: point) else { return nil }
            return (TargetSpec(query: label), point)
        }
    }

    private func nearestDetectedLabel(toWorldPoint point: Vec2) -> String? {
        perception.detectObjects()
            .compactMap { object -> (String, Double)? in
                guard let world = perception.groundPoint(of: object) else {
                    return nil
                }
                return (object.label, world.distance(to: point))
            }
            .min { $0.1 < $1.1 }?
            .0
    }

    private func decisionDescription(_ decision: RoverDecision) -> String {
        switch decision {
        case .navigate(let target): return "navigate(\(targetDescription(target)))"
        case .follow(let target): return "follow(\(targetDescription(target)))"
        case .explore(let candidateId): return "explore(\(candidateId))"
        case .lookAround(let angle): return String(format: "lookAround(%.2f)", angle)
        case .ask: return "ask"
        case .say: return "say"
        case .claimRoom(let roomId): return "claimRoom(\(roomId))"
        case .stop: return "stop"
        case .done: return "done"
        }
    }

    private func targetDescription(_ target: NavigationTarget) -> String {
        switch target {
        case .imagePoint(let p): return String(format: "imagePoint(%.2f,%.2f)", p.x, p.y)
        case .worldPoint(let p): return String(format: "worldPoint(%.2f,%.2f)", p.x, p.y)
        case .visualQuery(let query): return "visualQuery(\(query))"
        }
    }
}

private struct BrainDecisionTimeoutError: LocalizedError {
    let timeout: TimeInterval

    var errorDescription: String? {
        "Brain decision timed out after \(String(format: "%.2f", timeout)) seconds."
    }
}

private enum ScanRotationOutcome: Equatable {
    case completed
    case failed(String)
    case cancelled
    case sessionGenerationChanged
}

private enum BlockedHeadingRecoveryRaceOutcome {
    case completed
    case timedOut
    case cancelled
    case sessionGenerationChanged
}

@MainActor
private final class BlockedHeadingRecoveryRace {
    private var continuation: CheckedContinuation<BlockedHeadingRecoveryRaceOutcome, Never>?
    private var tasks: [Task<Void, Never>] = []
    private weak var motion: RoverMotion?
    private var ownsMotion: () -> Bool = { false }

    func start(
        motion: RoverMotion,
        topology: (any RoomTopologyManaging)?,
        expectedSessionGeneration: UInt64?,
        ownsMotion: @escaping () -> Bool,
        missionIsCancelled: @escaping @MainActor () -> Bool,
        angle: Double,
        timeout: TimeInterval,
        continuation: CheckedContinuation<BlockedHeadingRecoveryRaceOutcome, Never>
    ) {
        guard !Task.isCancelled, ownsMotion() else {
            continuation.resume(returning: .cancelled)
            return
        }
        self.motion = motion
        self.ownsMotion = ownsMotion
        self.continuation = continuation
        tasks = [
            Task { @MainActor in
                guard ownsMotion() else { self.finish(.cancelled); return }
                await motion.rotateForScan(by: angle)
                self.finish(Task.isCancelled ? .cancelled : .completed)
            },
            Task { @MainActor in
                do {
                    try await Task.sleep(for: .seconds(timeout))
                } catch {
                    return
                }
                if ownsMotion() { motion.cancel() }
                self.finish(.timedOut)
            },
            Task { @MainActor in
                while !Task.isCancelled {
                    if missionIsCancelled() {
                        if ownsMotion() { motion.cancel() }
                        self.finish(.cancelled)
                        return
                    }
                    if let expectedSessionGeneration,
                       topology?.snapshot.sessionGeneration != expectedSessionGeneration {
                        if ownsMotion() { motion.cancel() }
                        self.finish(.sessionGenerationChanged)
                        return
                    }
                    do {
                        try await Task.sleep(for: .milliseconds(10))
                    } catch {
                        return
                    }
                }
            }
        ]
    }

    func cancel() {
        guard continuation != nil else { return }
        if ownsMotion() { motion?.cancel() }
        finish(.cancelled)
    }

    private func finish(_ outcome: BlockedHeadingRecoveryRaceOutcome) {
        guard let continuation else { return }
        self.continuation = nil
        let runningTasks = tasks
        tasks.removeAll()
        runningTasks.forEach { $0.cancel() }
        continuation.resume(returning: outcome)
    }
}

@MainActor
private final class ScanRotationRace {
    private var continuation: CheckedContinuation<ScanRotationOutcome, Never>?
    private var tasks: [Task<Void, Never>] = []
    private weak var motion: RoverMotion?
    private var ownsMotion: () -> Bool = { false }

    func start(
        motion: RoverMotion,
        topology: (any RoomTopologyManaging)?,
        expectedSessionGeneration: UInt64?,
        ownsMotion: @escaping () -> Bool,
        angle: Double,
        continuation: CheckedContinuation<ScanRotationOutcome, Never>
    ) {
        guard !Task.isCancelled, ownsMotion() else {
            continuation.resume(returning: .cancelled)
            return
        }
        self.motion = motion
        self.ownsMotion = ownsMotion
        self.continuation = continuation
        tasks = [
            Task { @MainActor in
                guard ownsMotion() else { self.finish(.cancelled); return }
                await motion.rotateForScan(by: angle)
                if Task.isCancelled {
                    self.finish(.cancelled)
                } else if case .failed(let reason) = motion.state {
                    self.finish(.failed(reason))
                } else {
                    self.finish(.completed)
                }
            },
            Task { @MainActor in
                while !Task.isCancelled {
                    guard ownsMotion() else { self.finish(.cancelled); return }
                    if let expectedSessionGeneration,
                       topology?.snapshot.sessionGeneration != expectedSessionGeneration {
                        if ownsMotion() { motion.cancel() }
                        self.finish(.sessionGenerationChanged)
                        return
                    }
                    do {
                        try await Task.sleep(for: .milliseconds(10))
                    } catch {
                        return
                    }
                }
            }
        ]
    }

    func cancel() {
        guard continuation != nil else { return }
        if ownsMotion() { motion?.cancel() }
        finish(.cancelled)
    }

    private func finish(_ outcome: ScanRotationOutcome) {
        guard let continuation else { return }
        self.continuation = nil
        let runningTasks = tasks
        tasks.removeAll()
        runningTasks.forEach { $0.cancel() }
        continuation.resume(returning: outcome)
    }
}

@MainActor
private final class BrainDecisionRace {
    private var continuation: CheckedContinuation<BrainOutput, Error>?
    private var tasks: [Task<Void, Never>] = []
    private var cancellationRequested = false

    func start(
        brain: RoverBrain,
        context: MissionContext,
        timeout: TimeInterval,
        continuation: CheckedContinuation<BrainOutput, Error>
    ) {
        self.continuation = continuation
        guard !cancellationRequested, !Task.isCancelled else {
            finish(.failure(CancellationError()))
            return
        }

        tasks = [
            Task { @MainActor in
                do {
                    self.finish(.success(try await brain.nextAction(context)))
                } catch {
                    self.finish(.failure(error))
                }
            },
            Task { @MainActor in
                do {
                    try await Task.sleep(for: .seconds(timeout))
                } catch {
                    return
                }
                self.finish(.failure(BrainDecisionTimeoutError(timeout: timeout)))
            }
        ]
    }

    func cancel() {
        cancellationRequested = true
        finish(.failure(CancellationError()))
    }

    func finish(_ result: Result<BrainOutput, Error>) {
        guard let continuation else { return }
        self.continuation = nil
        let runningTasks = tasks
        tasks.removeAll()
        runningTasks.forEach { $0.cancel() }
        continuation.resume(with: result)
    }
}
