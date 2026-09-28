import Foundation
import FoundationModels
import RoverNav

/// Raw schema the on-device Foundation Model fills — a free-text visual description (not
/// limited to `MissionContext.visibleObjects`' closed-set labels; grounding it happens
/// outside the model, via `RoverPerception.groundObject`), a loose phrase to match against
/// `MissionMemory`, or plain dialog. `OnDeviceBrain` maps this into the shared
/// `RoverDecision`.
@Generable
struct OnDeviceDecision {
    @Guide(description: "What the rover should do next")
    var action: OnDeviceAction

    @Guide(description: "When action is navigateToObject: a short phrase describing what to drive toward, as if pointing it out to someone looking at the same view — can include colors, attributes, or possessives (\"the green chair\", \"my backpack\"), not just an object category. When action is follow: who or what to follow, the same way (\"the person with the hat\"; \"person\" for \"follow me\"). Empty otherwise.")
    var visualQuery: String

    @Guide(description: "When action is navigateToMemory: a short phrase describing which past location to return to (e.g. 'home', 'where we started', 'the kitchen'), matched loosely against recent conversation. Empty otherwise.")
    var memoryQuery: String

    @Guide(description: "When action is ask: the clarifying question to ask the operator. Empty otherwise.")
    var question: String

    @Guide(description: "When action is say: a short spoken response. Empty otherwise.")
    var spokenText: String

    @Guide(description: "When action is lookAround: degrees to rotate, counterclockwise positive. Use positive degrees for 'turn left' and negative degrees for 'turn right'. 90 to 360 is typical for a scan.")
    var lookAroundDegrees: Double

    @Guide(description: "When action is explore: the id of an opening from the openings list to go check (e.g. 'opening_1'). Prefer unexplored ones. Empty otherwise.")
    var exploreCandidateId: String

    @Guide(description: "Your running mission plan, rewritten in full whenever it changes (e.g. '1. find the green chair — done. 2. return to start.'). Empty to keep the current plan unchanged.")
    var updatedPlan: String
}

@Generable
enum OnDeviceAction: String, CaseIterable {
    case navigateToObject
    case follow
    case navigateToMemory
    case explore
    case lookAround
    case ask
    case say
    case stop
    case done
}

@MainActor
protocol OnDeviceBrainResponder {
    func nextAction(prompt: String, context: MissionContext) async throws -> BrainOutput
}

/// Fallback brain: Apple's on-device Foundation Model, reasoning over a *text* summary of
/// what's visible. The model itself only sees COCO labels (no color/attribute
/// understanding — it can't tell "green" from "red"), but it can still emit a free-text
/// `visualQuery` naming attributes the operator mentioned; whether that's actually
/// resolved goes to `RoverPerception.groundObject`, which may or may not understand more
/// than the model does. Keeps the rover responsive with no network at all, at the cost of
/// the richer understanding `CloudBrain` provides when online.
@MainActor
public final class OnDeviceBrain: RoverBrain {
    private let isAvailableProvider: () -> Bool
    private let makeResponder: () -> OnDeviceBrainResponder

    /// - Parameter adapter: an optional custom-trained adapter (Apple's adapter training
    ///   toolkit produces `.fmadapter` packages) specializing the base on-device model for
    ///   rover missions. `nil` uses the stock system model.
    public init(adapter: SystemLanguageModel.Adapter? = nil) {
        let model = adapter.map { SystemLanguageModel(adapter: $0) } ?? SystemLanguageModel.default
        isAvailableProvider = {
            if case .available = model.availability { return true }
            return false
        }
        makeResponder = {
            FoundationModelsOnDeviceResponder(model: model, instructions: Self.instructions)
        }
    }

    init(isAvailable: @escaping () -> Bool, makeResponder: @escaping () -> OnDeviceBrainResponder) {
        self.isAvailableProvider = isAvailable
        self.makeResponder = makeResponder
    }

    public var isAvailable: Bool {
        isAvailableProvider()
    }

    public func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        guard isAvailable else { throw RoverBrainError.unavailable }
        let prompt = promptText(context)
        return try await makeResponder().nextAction(prompt: prompt, context: context)
    }

    static let instructions = """
            You are the on-device brain of a small autonomous ground rover. Decide the \
            single next action given what the operator said, what's currently visible, \
            remembered objects, unexplored openings, and your mission plan. Keep a short \
            running plan for multi-step requests: rewrite the whole plan into updatedPlan \
            whenever it changes (mark finished steps done), and leave it empty to keep the \
            current plan. For navigateToObject, describe what to drive toward in your own \
            words as visualQuery — including any colors, attributes, or possessives the \
            operator mentioned ("the green chair", "my backpack") — even if that exact \
            object isn't in the visible-objects list, since visible-objects only reports \
            coarse categories and grounding the full description happens outside you. If \
            the target isn't visible here, the remembered-objects list may already have it \
            at a known position (use navigateToMemory or the position), or it may be in an \
            unexplored part of the space: choose explore with an opening id from the \
            openings list to go check — prefer unexplored openings, and if one you checked \
            turned out empty, try the next. Use lookAround to scan in place, including \
            explicit commands like "turn left" or "turn right" (left is positive degrees, \
            right is negative degrees). Use follow only when the operator asks you to keep \
            moving with someone ("follow me", "come with me", "follow the guy with the \
            hat"); it continues on its own until they say stop. A question, a turn, or \
            "go to"/"drive over to" someone is not follow — driving to a person once is \
            navigateToObject. "Follow me" means the person talking to you: use visualQuery \
            "person". When you are already following, keep going by choosing follow with \
            exactly the description you are following, unless the operator asked for \
            something else. Ask only if \
            you genuinely need clarification and haven't already asked; if a previous \
            question went unanswered, do your best with what you have rather than asking \
            again. Choose done once the operator's request is fully satisfied — including \
            any later steps of your plan, like returning after fetching something.
            """

    func promptText(_ context: MissionContext) -> String {
        var lines: [String] = []
        if let utterance = context.utterance { lines.append("Operator just said: \"\(utterance)\"") }
        if let plan = context.plan { lines.append("Current plan: \(plan)") }
        if let pose = context.pose {
            lines.append(String(format: "Current position: (%.1f, %.1f)", pose.position.x, pose.position.y))
        }
        lines.append(context.visibleObjects.isEmpty
            ? "Nothing recognized nearby right now."
            : "Visible now: " + context.visibleObjects
                .map { "\($0.label) (\(Int($0.confidence * 100))% confidence)" }
                .joined(separator: ", "))
        if !context.memory.rememberedObjects.isEmpty {
            lines.append("Objects remembered from earlier (may not be visible now): "
                + context.memory.rememberedObjects
                    .map { String(format: "%@ at (%.1f, %.1f)", $0.label, $0.worldPoint.x, $0.worldPoint.y) }
                    .joined(separator: ", "))
        }
        if !context.explorationCandidates.isEmpty {
            lines.append("Openings to unexplored space: "
                + context.explorationCandidates
                    .map { String(format: "%@ at (%.1f, %.1f), %.1fm wide [%@]",
                                  $0.id, $0.worldPoint.x, $0.worldPoint.y, $0.widthMeters, $0.status.rawValue) }
                    .joined(separator: "; "))
        }
        if !context.memory.turns.isEmpty {
            lines.append("Recent conversation: " + context.memory.turns.suffix(5)
                .map { "\"\($0.utterance)\"" }.joined(separator: "; "))
        }
        if let followState = context.followState {
            lines.append("You are currently \(followState). To keep following, choose follow with that same description.")
        }
        if !context.recentActions.isEmpty {
            lines.append("Your recent actions and what happened: " + context.recentActions.suffix(4).joined(separator: "; "))
        }
        if context.lastAnswerWasInconclusive {
            lines.append("Your last question went unanswered — proceed with your best guess instead of asking again.")
        }
        return lines.joined(separator: "\n")
    }

    static func map(_ raw: OnDeviceDecision, context: MissionContext) -> RoverDecision {
        switch raw.action {
        case .navigateToObject:
            // A remembered object matching the description beats a fresh visual search —
            // it works even when the target isn't in view (object permanence).
            if let remembered = context.memory.rememberedObjects.first(where: {
                raw.visualQuery.lowercased().contains($0.label.lowercased())
            }), context.visibleObjects.isEmpty {
                return .navigate(.worldPoint(remembered.worldPoint))
            }
            return .navigate(.visualQuery(raw.visualQuery))

        case .follow:
            let query = raw.visualQuery.trimmingCharacters(in: .whitespacesAndNewlines)
            return .follow(.visualQuery(query.isEmpty ? "person" : query))

        case .explore:
            return .explore(candidateId: raw.exploreCandidateId)

        case .navigateToMemory:
            guard let pose = bestMemoryMatch(raw.memoryQuery, in: context.memory) else {
                return .say("I don't remember where that is.")
            }
            return .navigate(.worldPoint(pose.position))

        case .lookAround:
            let degrees = raw.lookAroundDegrees == 0 ? 90 : raw.lookAroundDegrees
            return .lookAround(angle: degrees * .pi / 180)

        case .ask:
            return .ask(raw.question.isEmpty ? "Could you say more about where you'd like me to go?" : raw.question)

        case .say:
            return .say(raw.spokenText)

        case .stop:
            return .stop

        case .done:
            return .done
        }
    }

    /// Best-effort text matching only — no semantic understanding offline. "home"/"start"/
    /// "back" resolve to the mission's starting pose; a remembered object's label resolves
    /// to where it was seen (object permanence); anything else is matched against past
    /// utterances by substring containment.
    fileprivate static func bestMemoryMatch(_ query: String, in memory: MissionMemory) -> Pose2D? {
        let q = query.lowercased()
        if ["home", "start", "began", "back"].contains(where: q.contains) {
            return memory.missionStartPose
        }
        if let object = memory.rememberedObjects.last(where: { q.contains($0.label.lowercased()) }) {
            return Pose2D(position: object.worldPoint, yaw: 0)
        }
        return memory.turns.last { $0.utterance.lowercased().contains(q) || q.contains($0.utterance.lowercased()) }?.pose
    }
}

@MainActor
private final class FoundationModelsOnDeviceResponder: OnDeviceBrainResponder {
    private let session: LanguageModelSession

    init(model: SystemLanguageModel, instructions: String) {
        session = LanguageModelSession(model: model, instructions: instructions)
    }

    func nextAction(prompt: String, context: MissionContext) async throws -> BrainOutput {
        let result = try await session.respond(to: prompt, generating: OnDeviceDecision.self)
        let raw = result.content
        return BrainOutput(decision: OnDeviceBrain.map(raw, context: context),
                           updatedPlan: raw.updatedPlan.isEmpty ? nil : raw.updatedPlan)
    }
}
