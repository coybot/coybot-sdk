import Foundation
import RoverNav
@testable import PhroverKit

@MainActor
final class ManualSilentSearchClock: SilentSearchClock {
    private struct Waiter {
        let id: UUID
        let deadline: SilentSearchInstant
        let continuation: CheckedContinuation<Void, Error>
    }

    private(set) var wallNowMilliseconds: Int64
    private(set) var monotonicNow: SilentSearchInstant
    private var waiters: [Waiter] = []
    var pendingDeadlines: [SilentSearchInstant] { waiters.map(\.deadline) }

    init(wallNowMilliseconds: Int64 = 0, monotonicNow: SilentSearchInstant = 0) {
        self.wallNowMilliseconds = wallNowMilliseconds
        self.monotonicNow = monotonicNow
    }

    func sleep(until deadline: SilentSearchInstant) async throws {
        if deadline <= monotonicNow { return }
        let id = UUID()
        try await withTaskCancellationHandler {
            try Task.checkCancellation()
            try await withCheckedThrowingContinuation { continuation in
                waiters.append(Waiter(id: id, deadline: deadline, continuation: continuation))
            }
        } onCancel: {
            Task { @MainActor [weak self] in self?.cancelWaiter(id) }
        }
    }

    func advance(nanoseconds: Int64) {
        precondition(nanoseconds >= 0)
        monotonicNow += nanoseconds
        wallNowMilliseconds += nanoseconds / 1_000_000
        let ready = waiters.filter { $0.deadline <= monotonicNow }
        waiters.removeAll { $0.deadline <= monotonicNow }
        ready.forEach { $0.continuation.resume() }
    }

    private func cancelWaiter(_ id: UUID) {
        guard let index = waiters.firstIndex(where: { $0.id == id }) else { return }
        waiters.remove(at: index).continuation.resume(throwing: CancellationError())
    }
}

@MainActor
final class FakeSilentSearchReadiness: SilentSearchReadinessChecking {
    var snapshot: SilentSearchReadiness

    init(_ snapshot: SilentSearchReadiness = .notReady) {
        self.snapshot = snapshot
    }
}

@MainActor
final class FakeSilentSearchCalibration: SilentSearchCalibrating {
    private var continuation: AsyncStream<SilentSearchCalibrationEvent>.Continuation?
    private(set) var requests: [(markerID: String, generation: UInt64)] = []
    private(set) var cancelCount = 0

    func events(markerID: String, sessionGeneration: UInt64) -> AsyncStream<SilentSearchCalibrationEvent> {
        requests.append((markerID, sessionGeneration))
        return AsyncStream { continuation = $0 }
    }

    func send(_ event: SilentSearchCalibrationEvent) {
        continuation?.yield(event)
    }

    func cancel() {
        cancelCount += 1
        continuation?.finish()
        continuation = nil
    }
}

@MainActor
final class FakeSilentSearchOpticalExchange: SilentSearchOpticalExchanging {
    enum Failure: Error { case cancelled }

    private var incoming: [Data] = []
    private var scanContinuations: [CheckedContinuation<Data, Error>] = []
    private(set) var presentedPayloads: [Data] = []
    var peer: FakeSilentSearchOpticalExchange?
    var shouldRelay: (Data) -> Bool = { _ in true }
    var suspendPresent = false
    private var presentContinuations: [CheckedContinuation<Void, Error>] = []
    var suspendedPresentationCount: Int { presentContinuations.count }

    func present(payload: Data) async throws {
        presentedPayloads.append(payload)
        if shouldRelay(payload) { peer?.deliver(payload) }
        guard suspendPresent else { return }
        try await withCheckedThrowingContinuation { presentContinuations.append($0) }
    }

    func scan(until deadline: SilentSearchInstant) async throws -> Data {
        if !incoming.isEmpty { return incoming.removeFirst() }
        return try await withCheckedThrowingContinuation { scanContinuations.append($0) }
    }

    func cancel() {
        let scans = scanContinuations
        scanContinuations.removeAll()
        scans.forEach { $0.resume(throwing: Failure.cancelled) }
        let presentations = presentContinuations
        presentContinuations.removeAll()
        presentations.forEach { $0.resume(throwing: Failure.cancelled) }
    }

    func resumePresentations() {
        let continuations = presentContinuations
        presentContinuations.removeAll()
        continuations.forEach { $0.resume() }
    }

    func sendToScanner(_ payload: Data) {
        deliver(payload)
    }

    private func deliver(_ payload: Data) {
        if scanContinuations.isEmpty {
            incoming.append(payload)
        } else {
            scanContinuations.removeFirst().resume(returning: payload)
        }
    }
}

@MainActor
final class FakeSilentSearchExplorer: SilentSearchExploring {
    var selection: SectorExplorerSelection = .exhausted
    var selections: [SectorExplorerSelection] = []
    private(set) var selectionCount = 0
    private(set) var visitedIDs: [String] = []
    private(set) var rejections: [(String, SectorFrontierRejectionReason)] = []

    func nextCandidate() async -> SectorExplorerSelection {
        selectionCount += 1
        return selections.isEmpty ? selection : selections.removeFirst()
    }

    func markVisited(_ stableID: String) { visitedIDs.append(stableID) }

    func markRejected(_ stableID: String, reason: SectorFrontierRejectionReason) {
        rejections.append((stableID, reason))
    }
}

@MainActor
final class FakeSilentSearchTargetObserver: SilentSearchTargetObserving {
    weak var clock: ManualSilentSearchClock?
    var result: SilentSearchTargetObservationResult = .pending
    var results: [SilentSearchTargetObservationResult] = []
    private(set) var deadlines: [SilentSearchInstant] = []
    var suspend = false
    var pendingFrameInterval: Int64 = 700_000_000
    private var continuations: [CheckedContinuation<SilentSearchTargetObservationResult, Never>] = []

    func observeNextFrame(until deadline: SilentSearchInstant) async -> SilentSearchTargetObservationResult {
        deadlines.append(deadline)
        if suspend {
            return await withCheckedContinuation { continuations.append($0) }
        }
        let next = results.isEmpty ? result : results.removeFirst()
        if next == .pending, let clock, clock.monotonicNow < deadline {
            clock.advance(nanoseconds: min(pendingFrameInterval, deadline - clock.monotonicNow))
        }
        return next
    }

    func resume(with result: SilentSearchTargetObservationResult = .pending) {
        let pending = continuations
        continuations.removeAll()
        pending.forEach { $0.resume(returning: result) }
    }
}

@MainActor
final class FakeSilentSearchMotion: SilentSearchMotion {
    var currentMissionPose: MissionPose?
    var currentMissionPath: [MissionPoint] = []
    var result: SilentSearchMotionResult = .arrived
    var results: [SilentSearchMotionResult] = []
    var suspendNavigation = false
    var suspendRotation = false
    var suspendStop = false
    var clearPathOnStop = false
    private(set) var stopCount = 0
    private(set) var navigationRequests: [(MissionPoint, SilentSearchMotionPolicy)] = []
    private(set) var rotationRequests: [(heading: Double, tolerance: Double)] = []
    var updatePoseOnArrival = false
    var onStop: (() -> Void)?
    private var navigationContinuations: [CheckedContinuation<SilentSearchMotionResult, Never>] = []
    private var rotationContinuations: [CheckedContinuation<SilentSearchMotionResult, Never>] = []
    private var stopContinuations: [CheckedContinuation<Void, Never>] = []

    func navigate(to target: MissionPoint, policy: SilentSearchMotionPolicy) async -> SilentSearchMotionResult {
        navigationRequests.append((target, policy))
        if !results.isEmpty {
            let next = results.removeFirst()
            if next == .arrived, updatePoseOnArrival {
                currentMissionPose = MissionPose(position: target, heading: currentMissionPose?.heading ?? 0)
            }
            return next
        }
        guard suspendNavigation else {
            if result == .arrived, updatePoseOnArrival {
                currentMissionPose = MissionPose(position: target, heading: currentMissionPose?.heading ?? 0)
            }
            return result
        }
        return await withCheckedContinuation { navigationContinuations.append($0) }
    }

    func rotate(to heading: Double, tolerance: Double) async -> SilentSearchMotionResult {
        rotationRequests.append((heading, tolerance))
        if suspendRotation {
            return await withCheckedContinuation { rotationContinuations.append($0) }
        }
        if result == .arrived, updatePoseOnArrival, let pose = currentMissionPose {
            currentMissionPose = MissionPose(position: pose.position, heading: heading)
        }
        return result
    }

    func stop() async {
        stopCount += 1
        onStop?()
        if clearPathOnStop { currentMissionPath = [] }
        if suspendStop {
            await withCheckedContinuation { stopContinuations.append($0) }
        }
        let pendingNavigation = navigationContinuations
        navigationContinuations.removeAll()
        pendingNavigation.forEach { $0.resume(returning: .cancelled) }
        let pendingRotation = rotationContinuations
        rotationContinuations.removeAll()
        pendingRotation.forEach { $0.resume(returning: .cancelled) }
    }

    func resumeStops() {
        let pending = stopContinuations
        stopContinuations.removeAll()
        pending.forEach { $0.resume() }
    }
}

@MainActor
final class FakeSilentSearchSafetyMonitor: SilentSearchSafetyMonitoring {
    private var continuations: [UUID: AsyncStream<SilentSearchSafetyEvent>.Continuation] = [:]

    func events() -> AsyncStream<SilentSearchSafetyEvent> {
        let id = UUID()
        return AsyncStream { continuation in
            continuations[id] = continuation
            continuation.onTermination = { [weak self] _ in
                Task { @MainActor in self?.continuations[id] = nil }
            }
        }
    }

    func send(_ event: SilentSearchSafetyEvent) {
        continuations.values.forEach { $0.yield(event) }
    }

    var consumerCount: Int { continuations.count }
}

final class RecordingSilentSearchEventSink: SilentSearchEventSink, @unchecked Sendable {
    struct Entry: Equatable {
        let event: String
        let fields: [String: String]
    }

    private(set) var entries: [Entry] = []

    func record(event: String, fields: [String: String]) {
        entries.append(Entry(event: event, fields: fields))
    }
}

@MainActor
struct SilentSearchTestHarness {
    let clock: ManualSilentSearchClock
    let readiness = FakeSilentSearchReadiness()
    let calibration = FakeSilentSearchCalibration()
    let optical: FakeSilentSearchOpticalExchange
    let explorer = FakeSilentSearchExplorer()
    let targetObserver = FakeSilentSearchTargetObserver()
    let motion = FakeSilentSearchMotion()
    let safety = FakeSilentSearchSafetyMonitor()
    let events = RecordingSilentSearchEventSink()

    init(
        clock: ManualSilentSearchClock = ManualSilentSearchClock(),
        optical: FakeSilentSearchOpticalExchange = FakeSilentSearchOpticalExchange()
    ) {
        self.clock = clock
        self.optical = optical
        targetObserver.clock = clock
    }

    func coordinator() -> SilentSearchCoordinator {
        coordinator(calibration: calibration)
    }

    func coordinator(calibration: any SilentSearchCalibrating) -> SilentSearchCoordinator {
        SilentSearchCoordinator(dependencies: SilentSearchDependencies(
            clock: clock,
            readiness: readiness,
            calibration: calibration,
            opticalExchange: optical,
            explorer: explorer,
            targetObserver: targetObserver,
            motion: motion,
            safety: safety,
            events: events
        ))
    }
}
