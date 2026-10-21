import Foundation

struct FollowTurnBurstEpoch: Sendable {
    let entry: TimeInterval
    let deadline: TimeInterval
}

/// Immutable, operation-scoped sender entry facts. The synchronous final guard has no actor hop.
struct FollowTurnBurstAuthorization: Sendable {
    let operationID: UInt64
    private let fixedEpoch: FollowTurnBurstEpoch?
    private let preparedFence: FollowTurnBurstFence?
    private var epoch: FollowTurnBurstEpoch {
        // Prepared authority is never passed to a sender until its one-time arm.
        guard let value = fixedEpoch ?? preparedFence?.epoch else {
            preconditionFailure("Follow burst sender invoked before arming")
        }
        return value
    }
    var sendEntryUptime: TimeInterval { epoch.entry }
    var deadline: TimeInterval { epoch.deadline }
    let uptime: @Sendable () -> TimeInterval
    let isAuthorized: @Sendable () -> Bool
    let authorizeAttempt: (@Sendable () async -> Bool)?
    let didEnterAttempt: (@Sendable (FollowTurnTransportAttempt) -> Void)?
    let transportCapture: FollowTurnTransportCapture?

    init(operationID: UInt64, sendEntryUptime: TimeInterval, requestedBudget: TimeInterval,
         uptime: @escaping @Sendable () -> TimeInterval,
         isAuthorized: @escaping @Sendable () -> Bool = { true },
         authorizeAttempt: (@Sendable () async -> Bool)? = nil,
         didEnterAttempt: (@Sendable (FollowTurnTransportAttempt) -> Void)? = nil,
         transportCapture: FollowTurnTransportCapture? = nil) {
        self.operationID = operationID
        self.fixedEpoch = .init(entry: sendEntryUptime, deadline: sendEntryUptime + requestedBudget)
        self.preparedFence = nil
        self.uptime = uptime
        self.isAuthorized = isAuthorized
        self.authorizeAttempt = authorizeAttempt
        self.didEnterAttempt = didEnterAttempt
        self.transportCapture = transportCapture
    }

    init(operationID: UInt64, preparedFence: FollowTurnBurstFence,
         uptime: @escaping @Sendable () -> TimeInterval,
         authorizeAttempt: (@Sendable () async -> Bool)? = nil,
         didEnterAttempt: @escaping @Sendable (FollowTurnTransportAttempt) -> Void,
         transportCapture: FollowTurnTransportCapture) {
        self.operationID = operationID
        self.fixedEpoch = nil
        self.preparedFence = preparedFence
        self.uptime = uptime
        self.isAuthorized = { preparedFence.authorized(at: uptime()) }
        self.authorizeAttempt = authorizeAttempt
        self.didEnterAttempt = didEnterAttempt
        self.transportCapture = transportCapture
    }
}

enum FollowTurnBurstTransportDenial: Error {
    case expired, fenced, cancelled, invalidBudget
}

enum FollowTurnTargetStop: String, Sendable { case tolerance, crossing }

struct FollowTurnBurstSendReceipt {
    let targetStop: FollowTurnTargetStop?
    let transportAttempts: [FollowTurnTransportAttempt]
    let sendEntryUptime: TimeInterval
    let deadline: TimeInterval
    let responseUptime: TimeInterval
    let result: RoverCommandDiagnosticResult
    let stopObligation: Bool
    let stopObligationUptime: Double?

    var definitePreSendExpiry: Bool {
        result.failure as? FollowTurnBurstTransportDenial == .expired &&
            result.receipt.outcome == "expired" && result.receipt.attempts == 0 &&
            result.receipt.acknowledged == false && result.receipt.httpStatus == nil &&
            transportAttempts.isEmpty
    }

    var definiteUnsentTargetStop: Bool {
        targetStop != nil && result.failure as? FollowTurnBurstTransportDenial == .fenced &&
            result.receipt.outcome == "fenced" && result.receipt.attempts == 0 &&
            result.receipt.acknowledged == false && result.receipt.httpStatus == nil && transportAttempts.isEmpty
    }

    init(transportAttempts: [FollowTurnTransportAttempt] = [], sendEntryUptime: TimeInterval,
         deadline: TimeInterval, responseUptime: TimeInterval, result: RoverCommandDiagnosticResult,
           stopObligation: Bool, stopObligationUptime: Double? = nil, targetStop: FollowTurnTargetStop? = nil) {
        self.targetStop = targetStop
        self.transportAttempts = transportAttempts
        self.sendEntryUptime = sendEntryUptime
        self.deadline = deadline
        self.responseUptime = responseUptime
        self.result = result
        self.stopObligation = stopObligation
        self.stopObligationUptime = stopObligationUptime
    }
}

struct FollowTurnTransportAttempt: Sendable {
    let operationID: UInt64
    let attempt: Int
    let entryUptime: TimeInterval
}

struct FollowTurnTransportTiming: Sendable {
    enum Boundary: String, Sendable {
        case actorEntry = "actor_entry", authorizationStart = "authorization_start"
        case authorizationEnd = "authorization_end", eligibility, requestStart = "request_start"
    }
    let boundary: Boundary
    let uptime: TimeInterval
    var attempt: Int? = nil
    var eligible: Bool? = nil
}

final class FollowTurnTransportCapture: @unchecked Sendable {
    private let lock = NSLock()
    private var entries: [FollowTurnTransportAttempt] = []
    private var timingEntries: [FollowTurnTransportTiming] = []
    init() {
        entries.reserveCapacity(3)
        timingEntries.reserveCapacity(16)
    }
    func record(_ entry: FollowTurnTransportAttempt) { lock.withLock { entries.append(entry) } }
    var attempts: [FollowTurnTransportAttempt] { lock.withLock { entries } }
    func recordTiming(_ entry: FollowTurnTransportTiming) {
        lock.withLock { if timingEntries.count < 16 { timingEntries.append(entry) } }
    }
    var timings: [FollowTurnTransportTiming] { lock.withLock { timingEntries } }
}

struct FollowTurnBurstPendingStatus: Sendable {
    let operationID: UUID
    let deadline: TimeInterval
    let stopObligation: Bool
}

/// Synchronous invalidation can cross the sender's actor boundary without an authorizing await.
final class FollowTurnBurstFence: @unchecked Sendable {
    let identity = UUID()
    private let lock = NSLock()
    private var armedEpoch: FollowTurnBurstEpoch?
    private var permitted = true
    private var obligated = false
    private var obligationTime: Double?
    private var validity: Range<TimeInterval>?
    private var targetStopReason: FollowTurnTargetStop?
    var targetStop: FollowTurnTargetStop? { lock.withLock { targetStopReason } }
    /// Only the first, currently authorized inhibition can be target completion.
    /// Later source events cannot relabel an earlier expiry or safety fence.
    func inhibitForTarget(_ reason: FollowTurnTargetStop, at uptime: Double) {
        lock.withLock {
            if permitted, let epoch = armedEpoch, uptime.isFinite,
               uptime >= epoch.entry, uptime < epoch.deadline,
               validity?.contains(uptime) == true { targetStopReason = reason }
            permitted = false
            obligated = true
            obligationTime = min(obligationTime ?? .infinity, uptime)
        }
    }
    var epoch: FollowTurnBurstEpoch? { lock.withLock { armedEpoch } }
    func arm(entry: TimeInterval, budget: TimeInterval) -> FollowTurnBurstEpoch? {
        lock.withLock {
            guard permitted, armedEpoch == nil else { return nil }
            let value = FollowTurnBurstEpoch(entry: entry, deadline: entry + budget)
            guard entry.isFinite, value.deadline.isFinite, value.deadline > entry else { return nil }
            armedEpoch = value
            return value
        }
    }
    var authorized: Bool { lock.withLock { permitted && armedEpoch != nil } }
    /// Only primitive, MainActor-validated facts cross into the transport actor.
    /// Expiry is evaluated on every attempt even when MainActor is busy.
    func publishValidity(from uptime: TimeInterval, untilExclusive expiry: TimeInterval) -> Bool {
        lock.withLock {
            guard permitted, uptime.isFinite, !expiry.isNaN, expiry > uptime else { return false }
            validity = uptime..<expiry
            return true
        }
    }
    func authorized(at uptime: TimeInterval) -> Bool {
        lock.withLock {
            guard permitted, let epoch = armedEpoch, uptime.isFinite, let validity else { return false }
            return uptime >= epoch.entry && uptime < epoch.deadline && validity.contains(uptime)
        }
    }
    var status: FollowTurnBurstPendingStatus? {
        lock.withLock { armedEpoch.map { .init(operationID: identity, deadline: $0.deadline, stopObligation: obligated) } }
    }
    var stopObligationUptime: Double? { lock.withLock { obligationTime } }
    func inhibit(at uptime: Double? = nil) {
        lock.withLock {
            permitted = false
            obligated = true
            if let uptime { obligationTime = min(obligationTime ?? .infinity, uptime) }
        }
    }
}

/// Two controller-owned monitors, dormant until one immutable epoch is armed.
/// Finishing an abandoned preparation releases every waiter without inventing a deadline.
@MainActor
final class FollowTurnBurstArming {
    private(set) var epoch: FollowTurnBurstEpoch?
    private var finished = false
    private var waiters: [CheckedContinuation<FollowTurnBurstEpoch?, Never>] = []
    init() { waiters.reserveCapacity(2) }
    func wait() async -> FollowTurnBurstEpoch? {
        if finished || Task.isCancelled { return nil }
        if let epoch { return epoch }
        return await withCheckedContinuation {
            precondition(waiters.count < 2)
            waiters.append($0)
        }
    }
    func arm(_ epoch: FollowTurnBurstEpoch) {
        guard !finished, self.epoch == nil else { return }
        self.epoch = epoch
        resume(epoch)
    }
    func finish() {
        finished = true
        resume(nil)
    }
    private func resume(_ epoch: FollowTurnBurstEpoch?) {
        let pending = waiters
        waiters.removeAll()
        for waiter in pending { waiter.resume(returning: epoch) }
    }
}

@MainActor
final class FollowTurnBurstDrain {
    private var completed = false
    private var waiters: [CheckedContinuation<Void, Never>] = []
    func wait() async {
        if completed { return }
        await withCheckedContinuation { waiters.append($0) }
    }
    func finish() {
        completed = true
        let pending = waiters
        waiters.removeAll()
        for waiter in pending { waiter.resume() }
    }
}

enum FollowTurnBurstTransportScope {
    @TaskLocal static var authorization: FollowTurnBurstAuthorization?
}
