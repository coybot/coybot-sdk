import Foundation
import CoreGraphics
import RoverNav
import PhroverKit

/// Primary brain: sends the current frame + mission context to the reference `eco/aws`
/// `/rover/act` route (a vision + tool-use LLM) and gets back the next `RoverDecision`.
/// This is where open-vocabulary grounding ("the green chair", "in front of you") lives —
/// the response carries a normalized point in the frame that was sent, and `MissionAgent`
/// unprojects that through LiDAR depth locally. `CloudBrain` never sees depth and never
/// drives a motor directly; it only proposes a decision.
@MainActor
public final class CloudBrain: RoverBrain {
    private let baseURL: URL
    private let session: URLSession = .shared
    private var tokenProvider: (@Sendable () async -> String?)?

    public init(config: PhroverCloudConfig) {
        guard let url = URL(string: config.apiEndpoint) else {
            fatalError("Invalid API endpoint URL in PhroverCloudConfig: \(config.apiEndpoint)")
        }
        self.baseURL = url
    }

    public func setTokenProvider(_ provider: @escaping @Sendable () async -> String?) {
        tokenProvider = provider
    }

    public func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        var request = URLRequest(url: baseURL.appendingPathComponent("rover/act"))
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        if let token = await tokenProvider?() {
            request.setValue("Bearer \(token)", forHTTPHeaderField: "Authorization")
        }
        request.httpBody = try JSONEncoder().encode(ActRequest(context))

        let (data, response) = try await session.data(for: request)
        guard let http = response as? HTTPURLResponse, (200...299).contains(http.statusCode) else {
            throw CloudBrainError.serverError
        }
        let act = try JSONDecoder().decode(ActResponse.self, from: data)
        guard let decision = act.decision else {
            throw CloudBrainError.malformedDecision
        }
        return BrainOutput(decision: decision, updatedPlan: act.updatedPlan)
    }
}

public enum CloudBrainError: LocalizedError {
    case serverError
    case malformedDecision

    public var errorDescription: String? {
        switch self {
        case .serverError: return "Cloud mission brain unavailable."
        case .malformedDecision: return "Cloud mission brain returned an unrecognized decision."
        }
    }
}

// MARK: - Wire format

private struct ActRequest: Encodable {
    struct WirePose: Encodable { let x: Double; let y: Double; let yaw: Double }
    struct WireObject: Encodable { let label: String; let confidence: Float; let x: Double; let y: Double }
    struct WireTurn: Encodable { let utterance: String; let pose: WirePose }
    struct WireRememberedObject: Encodable { let label: String; let x: Double; let y: Double; let timesSeen: Int }
    struct WireMemory: Encodable {
        let turns: [WireTurn]
        let missionStartPose: WirePose?
        let rememberedObjects: [WireRememberedObject]
    }
    struct WireCandidate: Encodable {
        let id: String; let x: Double; let y: Double; let widthMeters: Double; let status: String
    }
    struct WireTeamRoom: Encodable { let id: String; let x: Double; let y: Double }
    struct WireTeammate: Encodable { let id: String; let alive: Bool; let claimedRoomIds: [String] }
    struct WireTeamContext: Encodable {
        let myId: String
        let rooms: [WireTeamRoom]
        let teammates: [WireTeammate]
    }

    let utterance: String?
    let frameJPEGBase64: String?
    let visibleObjects: [WireObject]
    let pose: WirePose?
    let navState: String
    let memory: WireMemory
    let explorationCandidates: [WireCandidate]
    let plan: String?
    let lastAnswerWasInconclusive: Bool
    let teamContext: WireTeamContext?
    let batteryPercent: Double?
    let followState: String?
    let recentActions: [String]

    init(_ context: MissionContext) {
        utterance = context.utterance
        frameJPEGBase64 = context.frameJPEG?.base64EncodedString()
        visibleObjects = context.visibleObjects.map {
            WireObject(label: $0.label, confidence: $0.confidence, x: $0.normalizedPoint.x, y: $0.normalizedPoint.y)
        }
        pose = context.pose.map { WirePose(x: $0.position.x, y: $0.position.y, yaw: $0.yaw) }
        navState = Self.wireNavState(context.navState)
        memory = WireMemory(
            turns: context.memory.turns.map {
                WireTurn(utterance: $0.utterance, pose: WirePose(x: $0.pose.position.x, y: $0.pose.position.y, yaw: $0.pose.yaw))
            },
            missionStartPose: context.memory.missionStartPose.map { WirePose(x: $0.position.x, y: $0.position.y, yaw: $0.yaw) },
            rememberedObjects: context.memory.rememberedObjects.map {
                WireRememberedObject(label: $0.label, x: $0.worldPoint.x, y: $0.worldPoint.y, timesSeen: $0.timesSeen)
            })
        explorationCandidates = context.explorationCandidates.map {
            WireCandidate(id: $0.id, x: $0.worldPoint.x, y: $0.worldPoint.y,
                          widthMeters: $0.widthMeters, status: $0.status.rawValue)
        }
        plan = context.plan
        followState = context.followState
        lastAnswerWasInconclusive = context.lastAnswerWasInconclusive
        teamContext = context.teamContext.map { tc in
            WireTeamContext(
                myId: tc.myId,
                rooms: tc.rooms.map { WireTeamRoom(id: $0.id, x: $0.worldPoint.x, y: $0.worldPoint.y) },
                teammates: tc.teammates.map { WireTeammate(id: $0.id, alive: $0.alive, claimedRoomIds: $0.claimedRoomIds) })
        }
        batteryPercent = context.batteryPercent
        recentActions = context.recentActions
    }

    private static func wireNavState(_ state: NavigationController.State) -> String {
        switch state {
        case .idle: return "idle"
        case .planning: return "planning"
        case .driving: return "driving"
        case .arrived: return "arrived"
        case .failed(let reason): return "failed: \(reason)"
        }
    }
}

/// What `/rover/act` returns: one action, plus whichever of the optional fields that
/// action needs. Kept flat (rather than a Swift-style nested enum) since it's crossing a
/// JSON boundary to a non-Swift backend.
private struct ActResponse: Decodable {
    let action: String
    let targetKind: String?   // "imagePoint" | "worldPoint" | "visualQuery" — navigate/follow
    let x: Double?
    let y: Double?
    let angle: Double?        // radians — only when action == "lookAround"
    let text: String?         // question (ask) or spoken text (say)
    let candidateId: String?  // only when action == "explore"
    let updatedPlan: String?  // optional plan rewrite, any action

    var decision: RoverDecision? {
        switch action {
        case "navigate":
            guard let x, let y else { return nil }
            switch targetKind {
            case "worldPoint": return .navigate(.worldPoint(Vec2(x, y)))
            default: return .navigate(.imagePoint(CGPoint(x: x, y: y)))
            }
        case "follow":
            // A follow may be named ("the person in the hat") as well as pointed at: a
            // moving target is often easier to describe than to pin to a pixel that will
            // be a round-trip stale by the time it lands.
            if targetKind == "visualQuery", let text, !text.isEmpty {
                return .follow(.visualQuery(text))
            }
            guard let x, let y else {
                guard let text, !text.isEmpty else { return nil }
                return .follow(.visualQuery(text))
            }
            switch targetKind {
            case "worldPoint": return .follow(.worldPoint(Vec2(x, y)))
            default: return .follow(.imagePoint(CGPoint(x: x, y: y)))
            }
        case "explore":
            guard let candidateId, !candidateId.isEmpty else { return nil }
            return .explore(candidateId: candidateId)
        case "claimRoom":
            // Reuses the same `candidateId` wire field as explore — both are just "an id
            // from a list the request offered" (rooms vs. openings), no need for a
            // second field crossing the same JSON boundary.
            guard let candidateId, !candidateId.isEmpty else { return nil }
            return .claimRoom(candidateId)
        case "lookAround":
            return .lookAround(angle: angle ?? .pi / 2)
        case "ask":
            return .ask(text ?? "Could you say more about where you'd like me to go?")
        case "say":
            return .say(text ?? "")
        case "stop":
            return .stop
        case "done":
            return .done
        default:
            return nil
        }
    }
}
