import Foundation
import CoreGraphics
import RoverNav

/// One perceived object, as seen by whichever detector produced it — the on-device COCO
/// `Detector`, or a cloud VLM's open-vocabulary grounding.
public struct PerceivedObject: Equatable, Sendable {
    public var label: String
    public var confidence: Float
    /// Normalized Vision-space point (bottom-left origin), suitable for
    /// `ARSessionManager.unproject(normalizedPoint:)`.
    public var normalizedPoint: CGPoint

    public init(label: String, confidence: Float, normalizedPoint: CGPoint) {
        self.label = label
        self.confidence = confidence
        self.normalizedPoint = normalizedPoint
    }
}

/// The rover's mission memory: what the operator said (and where the rover was when they
/// said it), plus every object it has seen pinned to a world coordinate. There's no naming
/// grammar and no special-cased return-to-origin — a brain reads this itself to work out
/// things like "go home", "and back", or "the chair we passed earlier".
public struct MissionMemory: Sendable {
    public struct Turn: Sendable {
        public var utterance: String
        public var pose: Pose2D
        public var timestamp: Date

        public init(utterance: String, pose: Pose2D, timestamp: Date = Date()) {
            self.utterance = utterance
            self.pose = pose
            self.timestamp = timestamp
        }
    }

    /// Object permanence: a detection pinned to the nav plane. Once the camera turns away
    /// the detection is gone, but this record — "chair at (1.2, 3.4)" — survives, so a
    /// brain can navigate back to it via `.worldPoint` with no re-detection needed.
    public struct RememberedObject: Equatable, Sendable {
        public var label: String
        public var worldPoint: Vec2
        public var lastSeenAt: Date
        public var timesSeen: Int

        public init(label: String, worldPoint: Vec2, lastSeenAt: Date = Date(), timesSeen: Int = 1) {
            self.label = label
            self.worldPoint = worldPoint
            self.lastSeenAt = lastSeenAt
            self.timesSeen = timesSeen
        }
    }

    public private(set) var turns: [Turn] = []
    public private(set) var rememberedObjects: [RememberedObject] = []
    /// Where the rover was when the very first utterance of the mission was recorded.
    public private(set) var missionStartPose: Pose2D?

    /// Same label within this distance = the same physical object (updated, not duplicated).
    static let objectMergeRadius = 0.75

    public init() {}

    /// Starts a new operator command while retaining long-term conversation and object
    /// memory. Return-style instructions must use the pose where this command began, not
    /// the first pose observed during the lifetime of the app's `MissionAgent`.
    public mutating func beginMission(utterance: String, at pose: Pose2D) {
        missionStartPose = pose
        turns.append(Turn(utterance: utterance, pose: pose))
    }

    public mutating func record(utterance: String, at pose: Pose2D) {
        if missionStartPose == nil { missionStartPose = pose }
        turns.append(Turn(utterance: utterance, pose: pose))
    }

    public mutating func rememberObject(label: String, at worldPoint: Vec2, now: Date = Date()) {
        if let i = rememberedObjects.firstIndex(where: {
            $0.label.caseInsensitiveCompare(label) == .orderedSame &&
            $0.worldPoint.distance(to: worldPoint) < Self.objectMergeRadius
        }) {
            rememberedObjects[i].worldPoint = worldPoint
            rememberedObjects[i].lastSeenAt = now
            rememberedObjects[i].timesSeen += 1
        } else {
            rememberedObjects.append(RememberedObject(label: label, worldPoint: worldPoint, lastSeenAt: now))
        }
    }
}

/// An opening into unexplored space (from `FrontierFinder`), with a stable id and a
/// visited flag maintained by `MissionAgent` across ticks. What a visit *revealed* ("that
/// was just a hallway") isn't modeled — that lives in the brain's reading of what it saw
/// after driving there; this list only answers "where could I still go look, and where
/// have I already been".
public struct ExplorationCandidate: Equatable, Sendable {
    public enum Status: String, Equatable, Sendable { case unexplored, visited }

    public var id: String
    public var worldPoint: Vec2
    public var widthMeters: Double
    public var status: Status

    public init(id: String, worldPoint: Vec2, widthMeters: Double, status: Status = .unexplored) {
        self.id = id
        self.worldPoint = worldPoint
        self.widthMeters = widthMeters
        self.status = status
    }
}

/// Where a `navigate` decision should go.
public enum NavigationTarget: Equatable, Sendable {
    /// A point in the current camera frame to unproject via LiDAR depth — how a brain
    /// points at something it just grounded itself (e.g. the cloud VLM, which can already
    /// see the image and point directly).
    case imagePoint(CGPoint)
    /// An already-resolved world-plane point — e.g. read back out of `MissionMemory` for
    /// "go home"/"and back", or a goal computed earlier.
    case worldPoint(Vec2)
    /// A free-text description of what to drive toward (can include attributes/possessives
    /// — "the green chair", "my backpack") that the brain itself can't visually ground.
    /// Delegated to `RoverPerception.groundObject(query:)`, so grounding quality depends on
    /// whatever perception implementation `MissionAgent` was given — a plain substring
    /// match by default, or something smarter (e.g. an on-device CLIP-style embedding
    /// model) if the app provides one.
    case visualQuery(String)
}

/// The one action a `RoverBrain` chooses per think-tick. Deliberately small and closed so
/// `MissionAgent` remains the sole place that turns a decision into motor commands — no
/// brain output reaches `RoverControl` directly.
public enum RoverDecision: Equatable, Sendable {
    case navigate(NavigationTarget)
    /// Drive to an `ExplorationCandidate` (by id) to see what's beyond it — the primitive
    /// for "the chair must be in another room, go check the doorway". Which opening to
    /// check, and what to conclude on arrival, is entirely the brain's judgment.
    case explore(candidateId: String)
    /// Turn to look for something not currently in view. Radians, CCW positive; a brain can
    /// ask for more than one look (each up to `2 * .pi`) across successive ticks.
    case lookAround(angle: Double)
    /// Keep station on something that is moving — a person walking, a cart being towed.
    /// Unlike `navigate`, this is a *mode*: it starts and keeps running until the target is
    /// lost, the time budget runs out, or a later decision stops it. The brain is still
    /// asked for a decision periodically while it runs, so it can narrate, ask, or break
    /// off; repeating `.follow` for the same target is a no-op rather than a restart.
    case follow(NavigationTarget)
    /// Speak a clarifying question, then wait for a reply before the next decision.
    case ask(String)
    /// Speak, with no expectation of a reply.
    case say(String)
    /// Claim a room/area (by id, from `MissionContext.teamContext.rooms`) for a
    /// multi-rover team mission — broadcast to teammates over the mesh via
    /// `MissionAgent`'s `RoverTeamRadio`. Capability "collaboration": shared intent and
    /// market allocation are the brain's own reasoning over `teamContext`, not a fixed
    /// protocol the app runs on the brain's behalf.
    case claimRoom(String)
    case stop
    /// The mission is satisfied; stop and go idle.
    case done
}

/// A single rover or area a multi-rover team can claim, and the team's current knowledge
/// of who has claimed what and who's still responding. Present in `MissionContext` only
/// for team missions; `nil` for a solo rover.
public struct TeamContext: Equatable, Sendable {
    public struct Room: Equatable, Sendable {
        public var id: String
        public var worldPoint: Vec2
        public init(id: String, worldPoint: Vec2) {
            self.id = id
            self.worldPoint = worldPoint
        }
    }
    public struct Teammate: Equatable, Sendable {
        public var id: String
        public var alive: Bool
        public var claimedRoomIds: [String]
        public init(id: String, alive: Bool, claimedRoomIds: [String]) {
            self.id = id
            self.alive = alive
            self.claimedRoomIds = claimedRoomIds
        }
    }

    public var myId: String
    public var rooms: [Room]
    public var teammates: [Teammate]

    public init(myId: String, rooms: [Room], teammates: [Teammate]) {
        self.myId = myId
        self.rooms = rooms
        self.teammates = teammates
    }
}

/// What a brain returns each tick: the action, plus an optional rewrite of the mission
/// plan. The plan is a free-form string the *brain* authors and maintains ("1. find the
/// green chair — done. 2. return to start."); `MissionAgent` just stores it and echoes it
/// back in the next `MissionContext`, so multi-leg intent survives across ticks without
/// any step schema or completion tracking in code.
public struct BrainOutput: Equatable, Sendable {
    public var decision: RoverDecision
    public var updatedPlan: String?

    public init(decision: RoverDecision, updatedPlan: String? = nil) {
        self.decision = decision
        self.updatedPlan = updatedPlan
    }
}

/// Snapshot of the world + conversation handed to a `RoverBrain` each think-tick.
/// Deliberately data-only so different brains — an on-device text reasoner, a cloud vision
/// reasoner — can consume the same shape, each using whatever subset it can.
public struct MissionContext: Sendable {
    /// What the operator just said, if this tick was triggered by a fresh utterance.
    public var utterance: String?
    /// Downscaled JPEG of the rover's current view, for brains that can see. `nil` when no
    /// frame is available yet (or offline brains that don't use it).
    public var frameJPEG: Data?
    /// Closed-set objects detected on-device right now — always available; feeds
    /// `OnDeviceBrain` directly and gives a cloud brain a cheap candidate list alongside
    /// the raw frame.
    public var visibleObjects: [PerceivedObject]
    public var pose: Pose2D?
    public var navState: NavigationController.State
    public var memory: MissionMemory
    /// Openings into unexplored space (from frontier detection), with visited status
    /// maintained across ticks — the brain's menu for "where could the thing be".
    public var explorationCandidates: [ExplorationCandidate]
    /// The mission plan as last written by a brain (see `BrainOutput.updatedPlan`), echoed
    /// back verbatim every tick. `nil` until a brain writes one.
    public var plan: String?
    /// Set when a prior `.ask` produced no usable reply (timeout, or "I don't know") — lets
    /// a brain proceed best-effort instead of repeating the same question.
    public var lastAnswerWasInconclusive: Bool
    /// Percent, 0-100. `nil` if the platform doesn't expose a battery (or hasn't reported
    /// yet) — a brain that wants capability "self-model and calibrated uncertainty"
    /// (comparing remaining range against return distance) needs this; most brains can
    /// simply ignore it.
    public var batteryPercent: Double?
    /// Present only for a multi-rover team mission (capability "collaboration"): the
    /// claimable rooms and the team's current knowledge of who has claimed what and
    /// who's still responding. `nil` for a solo mission.
    public var teamContext: TeamContext?
    /// What follow mode is doing right now ("following", "searching", …), or `nil` when
    /// nothing is being followed. A follow runs between ticks rather than inside one, so
    /// without this the brain has no way to tell a running follow from one that already
    /// ended.
    public var followState: String?
    /// Human-readable "action → outcome" lines, oldest first, for the last few ticks
    /// (see `MissionAgent`'s ring buffer) — lets a brain notice it's repeating itself or
    /// failed to follow through on a narrated intention, which nothing else in the
    /// context conveys (a single tick's snapshot looks identical whether it's the first
    /// attempt or the fortieth).
    public var recentActions: [String]

    public init(utterance: String? = nil,
                frameJPEG: Data? = nil,
                visibleObjects: [PerceivedObject] = [],
                pose: Pose2D? = nil,
                navState: NavigationController.State = .idle,
                memory: MissionMemory = MissionMemory(),
                explorationCandidates: [ExplorationCandidate] = [],
                plan: String? = nil,
                lastAnswerWasInconclusive: Bool = false,
                batteryPercent: Double? = nil,
                teamContext: TeamContext? = nil,
                followState: String? = nil,
                recentActions: [String] = []) {
        self.utterance = utterance
        self.frameJPEG = frameJPEG
        self.visibleObjects = visibleObjects
        self.pose = pose
        self.navState = navState
        self.memory = memory
        self.explorationCandidates = explorationCandidates
        self.plan = plan
        self.lastAnswerWasInconclusive = lastAnswerWasInconclusive
        self.batteryPercent = batteryPercent
        self.teamContext = teamContext
        self.followState = followState
        self.recentActions = recentActions
    }
}

/// A reasoner that turns a `MissionContext` into the next action (plus an optional plan
/// rewrite). Two implementations, chosen by `MissionAgent` per think-tick based on
/// connectivity: `CloudBrain` (primary, open-vocabulary) and `OnDeviceBrain` (fallback,
/// best-effort).
@MainActor
public protocol RoverBrain: AnyObject {
    func nextAction(_ context: MissionContext) async throws -> BrainOutput
}

public enum RoverBrainError: LocalizedError {
    case unavailable

    public var errorDescription: String? {
        "This brain is not available right now."
    }
}
