import Foundation
import CoreGraphics
import UIKit
import RoverNav

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
    func cancel()
}

extension RoverMotion {
    public func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) {
        navigate(to: goal)
    }

    public func rotateForScan(by angle: Double) async {
        await rotate(by: angle)
    }
}

extension NavigationController: RoverMotion {}

/// Perception surface `MissionAgent` reads. A separate protocol from `ARSessionManager` +
/// `Detector` so the mission loop can be tested by scripting what's "visible" without a
/// live ARKit session or the bundled CoreML model.
@MainActor
public protocol RoverPerception: AnyObject {
    var pose: Pose2D? { get }
    func detectObjects() -> [PerceivedObject]
    func unproject(normalizedPoint: CGPoint) -> Vec2?
    func capturedFrameJPEG() -> Data?
    /// Resolve a free-text description ("the green chair") to a normalized point in the
    /// current view, or `nil` if nothing matches. Has a default (substring-match)
    /// implementation below; conform your own for open-vocabulary/attribute grounding.
    func groundObject(query: String) -> CGPoint?
    /// Openings into unexplored space (frontier detection over the scene mesh). Default
    /// implementation returns [] for perception sources with no mapping capability.
    func explorationFrontiers() -> [Frontier]
}

extension RoverPerception {
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
}

/// Default `RoverPerception`: on-device COCO detection over the live ARKit frame.
@MainActor
public final class ARPerceptionSource: RoverPerception {
    private let ar: ARSessionManager
    private let detector: Detector?

    public init(ar: ARSessionManager, detector: Detector?) {
        self.ar = ar
        self.detector = detector
    }

    public var pose: Pose2D? { ar.pose }

    public func detectObjects() -> [PerceivedObject] {
        guard let detector, let buffer = ar.latestPixelBuffer else { return [] }
        return detector.detect(buffer, orientation: ar.imageOrientation).map {
            PerceivedObject(label: $0.label,
                            confidence: $0.confidence,
                            normalizedPoint: CGPoint(x: $0.boundingBox.midX, y: $0.boundingBox.midY))
        }
    }

    public func unproject(normalizedPoint: CGPoint) -> Vec2? {
        ar.unproject(normalizedPoint: normalizedPoint)
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
/// There is deliberately no special-cased command grammar here (no "and back" parsing, no
/// place-naming syntax). The operator's words and the rover's `MissionMemory` are just
/// inputs to whichever `RoverBrain` is current; behaviors like returning to a remembered
/// pose emerge from the brain reading that memory, not from code in this class.
@Observable
@MainActor
public final class MissionAgent {
    public enum Phase: Equatable { case idle, thinking, acting, waitingForAnswer }
    private enum VisualTargetScanResult {
        case found(Vec2)
        case notFound
        case cancelled
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
    private let askTimeout: TimeInterval
    private let brainDecisionTimeout: TimeInterval
    private let blockedHeadingRecoveryAngle: Double
    private let blockedHeadingRecoveryTimeout: TimeInterval
    private let visualTargetConfidenceThreshold: Float
    private let visualTargetScanAngle: Double
    private let visualTargetScanDelay: TimeInterval
    private let maxVisualTargetScanSteps: Int
    private let phaseDidChange: ((Phase) -> Void)?
    private let currentBrain: () -> RoverBrain?
    private let brainErrorLogger: (Error, MissionContext) -> Void

    private var lastAnswerWasInconclusive = false
    private var nextCandidateNumber = 1
    private var isHandlingMission = false
    private var missionGeneration = 0
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
                blockedHeadingRecoveryTimeout: TimeInterval = 1.2,
                visualTargetConfidenceThreshold: Float = 0.90,
                visualTargetScanAngle: Double = .pi / 6,
                visualTargetScanDelay: TimeInterval = 1,
                maxVisualTargetScanSteps: Int = 12,
                phaseDidChange: ((Phase) -> Void)? = nil,
                brainErrorLogger: @escaping (Error, MissionContext) -> Void = { error, context in
                    BrainErrorFileLog.append(error: error, context: context)
                },
                currentBrain: @escaping () -> RoverBrain?) {
        self.motion = motion
        self.perception = perception
        self.voice = voice
        self.follower = follower
        self.battery = battery
        self.teamRadio = teamRadio
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
        self.brainErrorLogger = brainErrorLogger
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

        RuntimeFileLog.append("voice_command_received", fields: ["utterance": trimmedUtterance])
        if isEmergencyStopUtterance(trimmedUtterance) {
            missionGeneration += 1
            isHandlingMission = false
            phase = .acting
            motion.cancel()
            follower?.stopFollowing()
            phase = .idle
            RuntimeFileLog.append("voice_command_stop", fields: ["utterance": trimmedUtterance])
            return
        }

        guard !isHandlingMission else {
            voice.speak("I'm still working on the previous command. Say stop if you want me to cancel it.")
            RuntimeFileLog.append("voice_command_busy", fields: ["utterance": trimmedUtterance])
            return
        }
        guard let pose = perception.pose else {
            voice.speak("I don't have my bearings yet — give me a moment to look around.")
            RuntimeFileLog.append("voice_command_rejected", fields: [
                "utterance": trimmedUtterance,
                "reason": "missing_pose"
            ])
            return
        }

        isHandlingMission = true
        missionGeneration += 1
        let missionID = missionGeneration
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
        await runLoop(firstUtterance: trimmedUtterance, missionID: missionID)
    }

    // MARK: - Loop

    private func runLoop(firstUtterance: String?, missionID: Int) async {
        var nextUtterance = firstUtterance
        let missionUtterance = firstUtterance ?? ""
        var lockedVisualQuery: String?
        var visualTargetScanSteps = 0
        // Distinguishes "the brain itself failed" (missing/timed out/errored) from
        // genuinely exhausting every tick without the brain ever stopping — only the
        // latter should get the "used up my time" wrap-up; the brain-failure paths
        // already spoke their own message and shouldn't pile a second one on top.
        var brainFailed = false

        for tick in 0..<maxTicksPerUtterance {
            guard isCurrentMission(missionID) else {
                phase = .idle
                RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                return
            }
            guard let brain = currentBrain() else {
                voice.speak("Sorry, I can't think right now.")
                RuntimeFileLog.append("mission_failed", fields: [
                    "mission": "\(missionID)",
                    "reason": "missing_brain"
                ])
                brainFailed = true
                break
            }

            phase = .thinking
            let rememberedCountBefore = memory.rememberedObjects.count
            updateWorldModel()
            let newObjects = memory.rememberedObjects.count - rememberedCountBefore
            let ctx = makeContext(utterance: nextUtterance)
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
            do {
                output = try await nextBrainAction(brain, context: ctx)
            } catch is BrainDecisionTimeoutError {
                guard isCurrentMission(missionID) else {
                    phase = .idle
                    RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                    return
                }
                voice.speak("Sorry, I'm having trouble thinking right now.")
                RuntimeFileLog.append("mission_brain_timeout", fields: [
                    "mission": "\(missionID)",
                    "timeout": String(format: "%.2f", brainDecisionTimeout)
                ])
                brainFailed = true
                break
            } catch {
                guard isCurrentMission(missionID) else {
                    phase = .idle
                    RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                    return
                }
                brainErrorLogger(error, ctx)
                voice.speak("Sorry, I'm having trouble thinking right now.")
                RuntimeFileLog.append("mission_brain_error", fields: [
                    "mission": "\(missionID)",
                    "error": error.localizedDescription
                ])
                brainFailed = true
                break
            }
            guard isCurrentMission(missionID) else {
                phase = .idle
                RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                return
            }
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

            phase = .acting
            let decision = output.decision
            let poseBefore = perception.pose?.position
            var outcome = ""

            switch decision {
            case .navigate(let target):
                let effectiveTarget = effectiveNavigationTarget(target,
                                                                lockedVisualQuery: &lockedVisualQuery,
                                                                missionID: missionID)
                guard let goal = resolve(effectiveTarget, missionID: missionID) else {
                    switch await scanForUnresolvedVisualTarget(effectiveTarget,
                                                               missionID: missionID,
                                                               scanSteps: &visualTargetScanSteps) {
                    case .found(let scannedGoal):
                        visualTargetScanSteps = 0
                        navigate(to: scannedGoal, for: effectiveTarget)
                        await waitForMotionToSettle()
                        guard isCurrentMission(missionID) else {
                            phase = .idle
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
                            phase = .idle
                            return
                        }
                        continue
                    case .cancelled:
                        phase = .idle
                        RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                        return
                    case .notFound:
                        break
                    }
                    guard isCurrentMission(missionID) else {
                        phase = .idle
                        RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                        return
                    }
                    voice.speak("I couldn't quite figure out where that is.")
                    if recordTick(decision: decision, poseBefore: poseBefore, newObjects: newObjects,
                                   outcome: "navigate → couldn't resolve target") {
                        phase = .idle
                        return
                    }
                    continue
                }
                visualTargetScanSteps = 0
                navigate(to: goal, for: effectiveTarget)
                await waitForMotionToSettle()
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
                guard let candidate = explorationCandidates.first(where: { $0.id == candidateId }) else {
                    voice.speak("I'm not sure which opening that is anymore.")
                    if recordTick(decision: decision, poseBefore: poseBefore, newObjects: newObjects,
                                   outcome: "explore(\(candidateId)) → unknown opening id") {
                        phase = .idle
                        return
                    }
                    continue
                }
                motion.navigate(to: candidate.worldPoint)
                await waitForMotionToSettle()
                if await recoverOrStopMissionIfMotionFailed(missionID: missionID,
                                                            recoverFromBlockedHeading: true,
                                                            blockedCandidateId: candidateId) { return }
                outcome = describeMotionOutcome(label: "explore(\(candidateId))", poseBefore: poseBefore)

            case .lookAround(let angle):
                await motion.rotate(by: angle)
                if await recoverOrStopMissionIfMotionFailed(missionID: missionID) { return }
                let poseAfter = perception.pose?.position
                let stayedPut = distance(poseBefore, poseAfter) < 0.05
                outcome = "lookAround(\(fmt(angle))) → " + (stayedPut
                    ? (newObjects > 0 ? "pose unchanged, saw \(newObjects) new thing(s)" : "pose unchanged, nothing new seen")
                    : "pose changed")

            case .ask(let question):
                phase = .waitingForAnswer
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
                    phase = .idle
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
                motion.cancel()
                follower?.stopFollowing()
                phase = .idle
                return

            case .done:
                phase = .idle
                RuntimeFileLog.append("mission_done", fields: ["mission": "\(missionID)"])
                return
            }

            if recordTick(decision: decision, poseBefore: poseBefore, newObjects: newObjects, outcome: outcome) {
                phase = .idle
                return
            }
        }

        if !brainFailed {
            voice.speak("I've used up the time I had for this and want to check in rather than keep going. "
                + wrapUpFindings())
        }
        phase = .idle
        RuntimeFileLog.append("mission_finished", fields: ["mission": "\(missionID)"])
    }

    private func nextBrainAction(_ brain: RoverBrain, context: MissionContext) async throws -> BrainOutput {
        try await withCheckedThrowingContinuation { continuation in
            let race = BrainDecisionRace(continuation: continuation)
            let decisionTask = Task { @MainActor in
                do {
                    let output = try await brain.nextAction(context)
                    race.finish(.success(output))
                } catch {
                    race.finish(.failure(error))
                }
            }
            Task { @MainActor in
                try? await Task.sleep(for: .seconds(brainDecisionTimeout))
                decisionTask.cancel()
                race.finish(.failure(BrainDecisionTimeoutError(timeout: brainDecisionTimeout)))
            }
        }
    }

    private func isCurrentMission(_ missionID: Int) -> Bool {
        missionGeneration == missionID
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
            if let world = perception.unproject(normalizedPoint: object.normalizedPoint) {
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
            guard let point = lockedVisualTargetPoint(query: q, missionID: missionID) else { return nil }
            return perception.unproject(normalizedPoint: point)
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

    private func lockedVisualTargetPoint(query: String, missionID: Int) -> CGPoint? {
        let objects = perception.detectObjects()
        guard let match = Self.bestVisualTargetMatch(query: query,
                                                     objects: objects,
                                                     minimumConfidence: visualTargetConfidenceThreshold) else {
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
        return match.normalizedPoint
    }

    static func bestVisualTargetMatch(query: String,
                                      objects: [PerceivedObject],
                                      minimumConfidence: Float) -> PerceivedObject? {
        guard !normalizedVisualQueryTokens(query).isEmpty else { return nil }
        return objects
            .filter { $0.confidence >= minimumConfidence }
            .filter { visualQueryMatches(query: query, label: $0.label) }
            .max { $0.confidence < $1.confidence }
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
        case "tvs", "television", "televisions":
            return "tv"
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
                                               scanSteps: inout Int) async -> VisualTargetScanResult {
        guard case .visualQuery(let query) = target else { return .notFound }
        while scanSteps < maxVisualTargetScanSteps {
            switch await waitForVisualTarget(query: query, missionID: missionID) {
            case .found(let goal):
                return .found(goal)
            case .noVisibleObjects:
                break
            case .cancelled:
                return .cancelled
            case .timedOut:
                break
            }
            guard isCurrentMission(missionID) else { return .cancelled }

            scanSteps += 1
            let angle = visualTargetScanAngle(forScanStep: scanSteps)
            RuntimeFileLog.append("mission_target_scan_step", fields: [
                "mission": "\(missionID)",
                "target": query,
                "step": "\(scanSteps)",
                "max": "\(maxVisualTargetScanSteps)",
                "angle": String(format: "%.0fdeg", angle * 180 / .pi)
            ])
            await motion.rotateForScan(by: angle)
            guard isCurrentMission(missionID) else { return .cancelled }

            switch await waitForVisualTarget(query: query, missionID: missionID) {
            case .found(let goal):
                return .found(goal)
            case .cancelled:
                return .cancelled
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
    }

    private func waitForVisualTarget(query: String, missionID: Int) async -> VisualTargetWaitResult {
        if visualTargetScanDelay <= 0 {
            guard isCurrentMission(missionID) else { return .cancelled }
            let objects = perception.detectObjects()
            guard !objects.isEmpty else { return .timedOut }
            if let point = lockedVisualTargetPoint(query: query, objects: objects, missionID: missionID),
               let goal = perception.unproject(normalizedPoint: point) {
                return .found(goal)
            }
            return .timedOut
        }

        let deadline = Date().addingTimeInterval(visualTargetScanDelay)
        var sawVisibleObjects = false
        while Date() < deadline {
            guard isCurrentMission(missionID) else { return .cancelled }
            let objects = perception.detectObjects()
            guard !objects.isEmpty else {
                try? await Task.sleep(for: .seconds(visualTargetPollInterval()))
                continue
            }
            sawVisibleObjects = true
            if let point = lockedVisualTargetPoint(query: query, objects: objects, missionID: missionID),
               let goal = perception.unproject(normalizedPoint: point) {
                RuntimeFileLog.append("mission_target_scan_wait_match", fields: [
                    "mission": "\(missionID)",
                    "target": query
                ])
                return .found(goal)
            }
            try? await Task.sleep(for: .seconds(visualTargetPollInterval()))
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

    private func visualTargetPollInterval() -> TimeInterval {
        min(0.2, max(0.01, visualTargetScanDelay / 5))
    }

    private func lockedVisualTargetPoint(query: String,
                                         objects: [PerceivedObject],
                                         missionID: Int) -> CGPoint? {
        guard let match = Self.bestVisualTargetMatch(query: query,
                                                     objects: objects,
                                                     minimumConfidence: visualTargetConfidenceThreshold) else {
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
        return match.normalizedPoint
    }

    private func visualTargetScanAngle(forScanStep step: Int) -> Double {
        guard step > 0 else { return 0 }
        if step == 1 { return visualTargetScanAngle }
        if step.isMultiple(of: 2) {
            return -visualTargetScanAngle * 2
        } else {
            return visualTargetScanAngle * 2
        }
    }

    private func navigate(to goal: Vec2, for target: NavigationTarget) {
        if case .visualQuery = target {
            motion.navigate(to: goal,
                            stoppingAtForwardClearance: RoverConfig.visualTargetStopDistance)
        } else {
            motion.navigate(to: goal)
        }
    }

    private func finishMissionIfVisualTargetArrived(_ target: NavigationTarget,
                                                    missionID: Int,
                                                    missionUtterance: String) -> Bool {
        guard case .arrived = motion.state, case .visualQuery = target else { return false }
        guard !Self.hasFollowUpIntent(missionUtterance), !Self.hasFollowUpIntent(plan ?? "") else {
            return false
        }
        phase = .idle
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

        guard await alignHeadingForReturn(to: start.position, missionID: missionID) else {
            return true
        }
        guard await navigateReturn(to: start.position, missionID: missionID) else { return true }

        phase = .idle
        plan = "Primary target reached; returned to mission start."
        RuntimeFileLog.append("mission_return_completed", fields: [
            "mission": "\(missionID)",
            "goal_x": String(format: "%.2f", start.position.x),
            "goal_y": String(format: "%.2f", start.position.y)
        ])
        return true
    }

    private func alignHeadingForReturn(to goal: Vec2, missionID: Int) async -> Bool {
        guard let pose = perception.pose else {
            failReturnMission(missionID: missionID,
                              reason: "I lost my position before I could return.")
            return false
        }

        let offset = goal - pose.position
        guard offset.length > 0.05 else { return true }

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
            guard isCurrentMission(missionID) else {
                phase = .idle
                RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                return false
            }

            let angle = min(abs(remaining), maximumStep) * (remaining < 0 ? -1 : 1)
            step += 1
            RuntimeFileLog.append("mission_return_alignment_step", fields: [
                "mission": "\(missionID)",
                "step": "\(step)",
                "angle_deg": String(format: "%.0f", angle * 180 / .pi)
            ])
            await motion.rotateForScan(by: angle)

            if case .failed(let reason) = motion.state {
                failReturnMission(missionID: missionID, reason: reason)
                return false
            }
            remaining -= angle
        }

        RuntimeFileLog.append("mission_return_alignment_completed", fields: [
            "mission": "\(missionID)",
            "steps": "\(step)"
        ])
        return true
    }

    private func navigateReturn(to goal: Vec2, missionID: Int) async -> Bool {
        let maximumAttempts = 2

        for attempt in 1...maximumAttempts {
            RuntimeFileLog.append("mission_return_navigation_attempt", fields: [
                "mission": "\(missionID)",
                "attempt": "\(attempt)",
                "max": "\(maximumAttempts)"
            ])
            motion.navigate(to: goal)
            await waitForMotionToSettle()

            guard isCurrentMission(missionID) else {
                phase = .idle
                RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
                return false
            }

            if case .arrived = motion.state { return true }

            if case .failed(let reason) = motion.state,
               Self.isBlockedHeading(reason),
               attempt < maximumAttempts {
                RuntimeFileLog.append("mission_return_recovery", fields: [
                    "mission": "\(missionID)",
                    "attempt": "\(attempt)",
                    "reason": reason,
                    "recovery": recoveryDescription
                ])
                await rotateForBlockedHeadingRecovery(missionID: missionID)
                if case .failed(let recoveryReason) = motion.state {
                    failReturnMission(missionID: missionID, reason: recoveryReason)
                    return false
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
            return false
        }

        failReturnMission(missionID: missionID,
                          reason: "I couldn't find a clear route back to the start.")
        return false
    }

    private func failReturnMission(missionID: Int, reason: String) {
        motion.cancel()
        voice.speak(reason)
        phase = .idle
        RuntimeFileLog.append("mission_motion_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason
        ])
        RuntimeFileLog.append("mission_return_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason
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
            await waitForMotionToSettle()
            guard isCurrentMission(missionID) else {
                phase = .idle
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
            phase = .idle
            RuntimeFileLog.append("mission_target_recovery_exhausted", fields: [
                "mission": "\(missionID)",
                "target": query
            ])
            return true

        case .cancelled:
            phase = .idle
            RuntimeFileLog.append("mission_cancelled", fields: ["mission": "\(missionID)"])
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

    private func waitForMotionToSettle() async {
        while motion.state == .driving {
            try? await Task.sleep(for: .seconds(RoverConfig.commandInterval))
        }
        RuntimeFileLog.append("motion_settled", fields: ["state": motion.state.description])
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
            await rotateForBlockedHeadingRecovery(missionID: missionID)
            guard case .failed(let recoveryReason) = motion.state else { return false }
            voice.speak(recoveryReason)
            phase = .idle
            RuntimeFileLog.append("mission_motion_failed", fields: [
                "mission": "\(missionID)",
                "reason": recoveryReason
            ])
            return true
        }
        voice.speak(reason)
        phase = .idle
        RuntimeFileLog.append("mission_motion_failed", fields: [
            "mission": "\(missionID)",
            "reason": reason
        ])
        return true
    }

    private func markExplorationCandidateVisited(_ id: String, reason: String) {
        guard let index = explorationCandidates.firstIndex(where: { $0.id == id }) else { return }
        explorationCandidates[index].status = .visited
        RuntimeFileLog.append("mission_exploration_candidate_blocked", fields: [
            "candidate": id,
            "reason": reason,
            "status": ExplorationCandidate.Status.visited.rawValue
        ])
    }

    private var recoveryDescription: String {
        String(format: "rotate_%.0fdeg", blockedHeadingRecoveryAngle * 180 / .pi)
    }

    private func rotateForBlockedHeadingRecovery(missionID: Int) async {
        let angle = blockedHeadingRecoveryAngle
        let timeout = blockedHeadingRecoveryTimeout
        let rotation = Task { @MainActor in
            await motion.rotateForScan(by: angle)
        }

        try? await Task.sleep(for: .seconds(timeout))
        if motion.state == .driving {
            RuntimeFileLog.append("mission_blocked_heading_recovery_timeout", fields: [
                "mission": "\(missionID)",
                "recovery": recoveryDescription,
                "timeout": String(format: "%.2f", timeout)
            ])
            motion.cancel()
        } else {
            RuntimeFileLog.append("mission_blocked_heading_recovery_settled", fields: [
                "mission": "\(missionID)",
                "recovery": recoveryDescription
            ])
        }
        await rotation.value
    }

    private static func isBlockedHeading(_ reason: String) -> Bool {
        reason.localizedCaseInsensitiveContains("Obstacle ahead")
    }

    private static func isNavigationStalled(_ reason: String) -> Bool {
        reason.localizedCaseInsensitiveContains("Navigation stalled")
    }

    private func makeContext(utterance: String?) -> MissionContext {
        MissionContext(utterance: utterance,
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

    private func followStateDescription() -> String? {
        guard let follower, follower.followingSpec != nil else { return nil }
        return follower.followState.description
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
            voice.speak("Following the \(resolved.spec.query). Say stop when you want me to hold.")
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
                guard let world = perception.unproject(normalizedPoint: object.normalizedPoint) else {
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

@MainActor
private final class BrainDecisionRace {
    private var didFinish = false
    private let continuation: CheckedContinuation<BrainOutput, Error>

    init(continuation: CheckedContinuation<BrainOutput, Error>) {
        self.continuation = continuation
    }

    func finish(_ result: Result<BrainOutput, Error>) {
        guard !didFinish else { return }
        didFinish = true
        continuation.resume(with: result)
    }
}
