import Foundation

@MainActor
public protocol OperatorMission: AnyObject {
    func handle(_ text: String) async
    func cancelCurrentMissionAndWait() async throws
}

@MainActor
public protocol OperatorFollow: AnyObject {
    var state: FollowMeState { get }
    func start() async -> Bool
    func stop() async -> Bool
    func inhibitMotion()
}

extension MissionAgent: OperatorMission {}
extension FollowMeCoordinator: OperatorFollow, OperatorFollowTimedStart {}

/// Optional timing companion; existing public follow conformers remain compatible.
@MainActor
protocol OperatorFollowTimedStart: OperatorFollow {
    func start(commandReceivedAt: TimeInterval) async -> Bool
}

public enum OperatorSubmission: Equatable, Sendable {
    case accepted
    case rejected(String)
}

public enum OperatorCommandKind: Equatable, Sendable {
    case localStop, localFollow, mission

    public static func classify(_ text: String) -> Self {
        let phrase = text.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
            .trimmingCharacters(in: CharacterSet(charactersIn: ".!?"))
            .trimmingCharacters(in: .whitespacesAndNewlines)
        // Match the emergency vocabulary already accepted by Coybot's mission
        // agent. Follow ownership must not turn a stop utterance into a rejected
        // mission and leave the rover driving.
        let tokens = Set(phrase.components(separatedBy: CharacterSet.alphanumerics.inverted)
            .filter { !$0.isEmpty })
        if !tokens.isDisjoint(with: ["stop", "halt", "cancel"]) { return .localStop }
        if ["follow me", "start following", "start following me"].contains(phrase) { return .localFollow }
        return .mission
    }
}

/// Reserves motion ownership before suspension so delayed starts cannot outrun stop.
@MainActor
public final class OperatorCommandRouter {
    private enum Owner: Equatable { case idle, mission, transitioning, follow, stopping, blocked }
    private let mission: any OperatorMission
    private let follow: any OperatorFollow
    private let mayStartFollow: @MainActor () -> Bool
    private let clock: any FollowMeClock
    private let diagnosticEmitter: FollowDiagnosticEmitter
    private var owner: Owner = .idle
    private var generation: UInt64 = 0
    private var missionTask: Task<Void, Never>?
    private var pendingMissionStop: Task<Bool, Never>?
    private var stopTask: Task<OperatorSubmission, Never>?
    private var followStartTask: Task<OperatorSubmission, Never>?
    private var commandSequence: UInt64 = 0

    public init(mission: any OperatorMission, follow: any OperatorFollow,
                mayStartFollow: @escaping @MainActor () -> Bool = { true },
                clock: any FollowMeClock = SystemFollowClock(),
                eventSink: @escaping @MainActor (String, [String: String]) -> Void = { RuntimeFileLog.append($0, fields: $1) }) {
        self.mission = mission
        self.follow = follow
        self.mayStartFollow = mayStartFollow
        self.clock = clock
        self.diagnosticEmitter = FollowDiagnosticEmitter(streamID: "operator-router-\(UUID().uuidString)",
            monotonic: { clock.now }, utc: { Date() }, sink: eventSink)
    }

    public func submit(_ text: String, finalizedTextReceivedAt: TimeInterval? = nil) async -> OperatorSubmission {
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return .rejected("Enter a request first.") }
        let kind = OperatorCommandKind.classify(text)
        commandSequence &+= 1
        let commandID = commandSequence
        let receivedAt = finalizedTextReceivedAt ?? clock.now
        let origin = finalizedTextReceivedAt == nil ? "submitted_text_receipt" : "finalized_text_receipt"
        emitCommand("operator_command.received", kind: kind, commandID: commandID, receivedAt: receivedAt, origin: origin)
        if kind == .localStop { return await stop() }
        if kind == .localFollow {
            guard owner != .blocked && owner != .stopping else {
                return .rejected("Rover stop could not be confirmed.")
            }
            if let followStartTask { return await followStartTask.value }
            if owner == .follow || follow.state.isActive { return .accepted }
            guard owner == .mission || mayStartFollow() else {
                return .rejected("Stop the other rover activity before following.")
            }
            generation &+= 1
            let token = generation
            owner = .transitioning
            let task = Task { @MainActor in
                await self.startFollow(token: token, kind: kind, commandID: commandID,
                                       receivedAt: receivedAt, origin: origin)
            }
            followStartTask = task
            let result = await task.value
            if generation == token { followStartTask = nil }
            return result
        }
        guard owner == .idle && !follow.state.isActive else {
            return .rejected(owner == .mission
                ? "I’m still working on the previous command. Stop it before sending another request."
                : "Stop following before sending another request.")
        }
        generation &+= 1
        let token = generation
        owner = .mission
        missionTask = Task { [weak self, mission] in
            await mission.handle(text)
            if let self, self.generation == token, self.owner == .mission { self.owner = .idle }
        }
        return .accepted
    }

    private func startFollow(token: UInt64, kind: OperatorCommandKind, commandID: UInt64,
                             receivedAt: TimeInterval, origin: String) async -> OperatorSubmission {
        guard generation == token else { return .rejected("Follow request cancelled.") }
        emitCommand("operator_command.ownership_stop_requested", kind: kind, commandID: commandID, receivedAt: receivedAt, origin: origin)
        missionTask?.cancel()
        let stopping = Task { [mission] in
            do { try await mission.cancelCurrentMissionAndWait(); return true }
            catch { return false }
        }
        pendingMissionStop = stopping
        let confirmed = await stopping.value
        guard generation == token else { return .rejected("Follow request cancelled.") }
        pendingMissionStop = nil
        guard confirmed else {
            owner = .blocked
            emitCommand("operator_command.ownership_stop_failed", kind: kind, commandID: commandID, receivedAt: receivedAt, origin: origin)
            return .rejected("Rover stop could not be confirmed.")
        }
        emitCommand("operator_command.ownership_stop_confirmed", kind: kind, commandID: commandID, receivedAt: receivedAt, origin: origin)
        emitCommand("operator_command.follow_start_requested", kind: kind, commandID: commandID, receivedAt: receivedAt, origin: origin)
        let started: Bool
        if let timed = follow as? any OperatorFollowTimedStart { started = await timed.start(commandReceivedAt: receivedAt) }
        else { started = await follow.start() }
        guard generation == token else { return .rejected("Follow request cancelled.") }
        guard started else {
            owner = .idle
            return .rejected("Could not start following.")
        }
        owner = .follow
        return .accepted
    }

    private func emitCommand(_ event: String, kind: OperatorCommandKind, commandID: UInt64,
                             receivedAt: TimeInterval, origin: String) {
        let now = clock.now
        let valid = receivedAt.isFinite && now.isFinite && receivedAt <= now
        diagnosticEmitter.emit(.init(event: event, context: .init(sessionGeneration: generation,
            phase: String(describing: owner)), payload: [
                "command_id": .number(Double(commandID)), "command_kind": .string(String(describing: kind)),
                "command_received_at_s": .number(receivedAt), "command_elapsed_s": valid ? .number(now - receivedAt) : .null,
                "receipt_origin": .string(origin), "receipt_time_availability": .string(valid ? "available" : "invalid"),
                "physical_utterance_time_availability": .string("not_measured"), "timing_clock": .string("system_uptime")]))
    }

    public func stop() async -> OperatorSubmission {
        if let stopTask { return await stopTask.value }
        generation &+= 1
        followStartTask = nil
        owner = .stopping
        follow.inhibitMotion()
        missionTask?.cancel()
        let pending = pendingMissionStop
        let task = Task { [mission, follow] () -> OperatorSubmission in
            if let pending {
                guard await pending.value else { return .rejected("Rover stop could not be confirmed.") }
            } else {
                do { try await mission.cancelCurrentMissionAndWait() }
                catch { return .rejected("Rover stop could not be confirmed.") }
            }
            guard await follow.stop() else { return .rejected("Rover stop could not be confirmed.") }
            return .accepted
        }
        stopTask = task
        let result = await task.value
        stopTask = nil
        pendingMissionStop = nil
        owner = result == .accepted ? .idle : .blocked
        return result
    }
}
