import Foundation
import RoverNav

enum FollowTurnSourceScope {
    @TaskLocal static var required = false
}

/// Cancellation reaches a queued timer synchronously, before the actor hop
/// that resumes its source waiter. A cancelled timer must not invoke sleep.
final class FollowTurnSourceWaitToken: @unchecked Sendable {
    private let lock = NSLock()
    private var active = true
    var isActive: Bool { lock.withLock { active } }
    func cancel() { lock.withLock { active = false } }
}

enum FollowTurnSourceResult {
    case sample(NavigationPoseSample)
    case failed(NavigationFailure)
    case cancelled
}

/// The executor passes its existing checkpoint; settle/source waits never reset it.
struct FollowTurnWaitingProgress {
    var watchdog = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
    var distanceToGoal: Double = 0
    var hasSentCommand = false

    func remaining(at date: Date) -> TimeInterval? {
        let snapshot = watchdog.diagnosticSnapshot(distanceToGoal: distanceToGoal, now: date)
        return snapshot.elapsed.map { max(0, snapshot.interval - $0) }
    }

    func expired(at date: Date) -> Bool { remaining(at: date).map { $0 <= 0 } ?? false }
}

/// One Date-based progress epoch for the entire frozen-target operation.
/// Only distinct healthy source observations can move its 0.05-rad checkpoint.
@MainActor
final class FollowTurnRuntimeState {
    let targetYaw: Double
    let tolerance: Double
    let settleWait: Double
    let generation: UInt64
    var progress = FollowTurnWaitingProgress()
    var lastAck: Date?
    private(set) var failure: NavigationFailure?
    private(set) var sourceRejection: String?
    private var previous: NavigationPoseSample

    init(targetYaw: Double, initial: NavigationPoseSample, tolerance: Double, date: Date, settleWait: Double = 0.300) {
        self.targetYaw = targetYaw
        self.tolerance = tolerance
        self.settleWait = settleWait
        generation = initial.frameID!.generation
        previous = initial
        progress.distanceToGoal = abs(FollowReacquisitionPlanner.wrap(targetYaw - initial.pose!.yaw))
        if progress.distanceToGoal > tolerance {
            _ = progress.watchdog.observe(distanceToGoal: progress.distanceToGoal, now: date, commanded: true)
        }
    }

    func expire(at date: Date) {
        if failure == nil, progress.expired(at: date) { failure = .stalled }
    }

    func fail(_ reason: NavigationFailure) { if failure == nil { failure = reason } }

    func failTracking(_ rejection: String?) {
        guard failure == nil else { return }
        sourceRejection = rejection
        failure = .trackingLost
    }

    func observe(_ sample: NavigationPoseSample, uptime: Double, date: Date) {
        guard failure == nil else { return }
        if let rejection = sample.rejection(at: uptime, expectedGeneration: generation, requireEnriched: true) {
            failTracking(rejection)
            return
        }
        guard sample.frameID!.sequence > previous.frameID!.sequence,
              sample.sourceTimestamp! > previous.sourceTimestamp! else { expire(at: date); return }
        previous = sample
        progress.distanceToGoal = abs(FollowReacquisitionPlanner.wrap(targetYaw - sample.pose!.yaw))
        if progress.watchdog.observe(distanceToGoal: progress.distanceToGoal, now: date, commanded: true) {
            failure = .stalled
        }
    }
}

/// Immutable confirmation facts in the AR/system-uptime domain.
struct FollowTurnStopFence: Sendable {
    let identity: UUID
    let operationGeneration: UInt
    let context: FollowMotionOperationContext
    let acknowledgementUptime: TimeInterval
    let sourceGeneration: UInt64?
    let highestSequence: UInt64?
    let highestSourceTimestamp: TimeInterval?
}

struct FollowTurnSourceHighWater: Sendable {
    let frameID: ARFrameID
    let sourceTimestamp: TimeInterval?
}

struct FollowTurnSourceHealth: Sendable {
    let generation: UInt64
    let trackingQuality: ARTrackingQuality
}

/// Operation-local bounded ingress, independent of source-provider reads.
struct FollowTurnSourceGate {
    private(set) var latest: NavigationPoseSample?
    private(set) var highestSequence: UInt64?
    private(set) var highestTimestamp: TimeInterval?
    private(set) var generation: UInt64?
    private var lastConsumed: NavigationPoseSample?
    private var stationaryWindow: [NavigationPoseSample] = []

    mutating func include(_ highWater: FollowTurnSourceHighWater) {
        if generation != highWater.frameID.generation {
            generation = highWater.frameID.generation
            highestSequence = nil
            highestTimestamp = nil
        }
        highestSequence = max(highestSequence ?? 0, highWater.frameID.sequence)
        if let timestamp = highWater.sourceTimestamp, timestamp.isFinite {
            highestTimestamp = max(highestTimestamp ?? timestamp, timestamp)
        }
    }

    mutating func ingest(_ sample: NavigationPoseSample) {
        observeStationarity(sample)
        latest = sample
        if let id = sample.frameID {
            if generation != id.generation {
                generation = id.generation
                highestSequence = nil
                highestTimestamp = nil
            }
            highestSequence = max(highestSequence ?? 0, id.sequence)
        }
        if let time = sample.sourceTimestamp, time.isFinite {
            highestTimestamp = max(highestTimestamp ?? time, time)
        }
    }

    private mutating func observeStationarity(_ sample: NavigationPoseSample) {
        guard let timestamp = sample.sourceTimestamp,
              sample.rejection(at: timestamp, requireEnriched: true) == nil, let pose = sample.pose else {
            stationaryWindow = []
            return
        }
        guard let previous = latest, let previousID = previous.frameID, let id = sample.frameID,
              previousID.generation == id.generation, let previousTime = previous.sourceTimestamp,
              let previousPose = previous.pose else { stationaryWindow = [sample]; return }
        if id == previousID, timestamp == previousTime, pose.yaw == previousPose.yaw,
           pose.position.distance(to: previousPose.position) == 0 { return }
        guard previousID.sequence < UInt64.max, id.sequence == previousID.sequence + 1,
              timestamp > previousTime, timestamp - previousTime <= 0.100,
              !stationaryWindow.isEmpty,
              stationaryWindow.allSatisfy({ old in
                  guard let oldPose = old.pose else { return false }
                  return abs(FollowReacquisitionPlanner.wrap(pose.yaw - oldPose.yaw)) <= 0.01 &&
                      pose.position.distance(to: oldPose.position) <= 0.01
              }) else {
            stationaryWindow = [sample]
            return
        }
        stationaryWindow.append(sample)
        if stationaryWindow.count > 32 { stationaryWindow.removeFirst() }
    }

    /// Search can shorten the existing 300 ms wait only with a contiguous,
    /// bounded-drift AR window spanning at least 100 ms after the stop ACK.
    private func settled(after fence: FollowTurnStopFence, at uptime: Double, minimumSettle: Double) -> Bool {
        if uptime >= fence.acknowledgementUptime + 0.300 { return true }
        guard minimumSettle >= 0.100, minimumSettle < 0.300,
              uptime >= fence.acknowledgementUptime + minimumSettle,
              let start = stationaryWindow.first(where: { sample in
                  guard let id = sample.frameID, let time = sample.sourceTimestamp else { return false }
                  return id.generation == fence.sourceGeneration && id.sequence > (fence.highestSequence ?? 0) &&
                      time > fence.acknowledgementUptime && time > (fence.highestSourceTimestamp ?? -.infinity)
              })?.sourceTimestamp,
              let end = latest?.sourceTimestamp else { return false }
        return end - start >= minimumSettle
    }

    func fence(at uptime: TimeInterval, operationGeneration: UInt,
               context: FollowMotionOperationContext = .unknown) -> FollowTurnStopFence {
        .init(identity: UUID(), operationGeneration: operationGeneration, context: context,
              acknowledgementUptime: uptime, sourceGeneration: generation,
              highestSequence: highestSequence, highestSourceTimestamp: highestTimestamp)
    }

    mutating func consume(after fence: FollowTurnStopFence, at uptime: TimeInterval, minimumSettle: Double = 0.300) -> NavigationPoseSample? {
        guard settled(after: fence, at: uptime, minimumSettle: minimumSettle),
              let sample = latest, let id = sample.frameID, let timestamp = sample.sourceTimestamp,
              sample.rejection(at: uptime, expectedGeneration: fence.sourceGeneration, requireEnriched: true) == nil,
              timestamp > fence.acknowledgementUptime,
              id.sequence > (fence.highestSequence ?? 0),
              timestamp > (fence.highestSourceTimestamp ?? -.infinity) else { return nil }
        if let consumed = lastConsumed, consumed.frameID?.generation == id.generation {
            guard id.sequence > (consumed.frameID?.sequence ?? 0),
                  timestamp > (consumed.sourceTimestamp ?? -.infinity) else { return nil }
        }
        lastConsumed = sample
        return sample
    }

    /// Read-only explanation of the current stopped admission fence.
    func diagnosticRejection(after fence: FollowTurnStopFence, at uptime: Double, minimumSettle: Double = 0.300) -> String? {
        guard let sample = latest else { return "missing_source" }
        if let reason = sample.rejection(at: uptime, expectedGeneration: fence.sourceGeneration, requireEnriched: true) { return reason }
        if !settled(after: fence, at: uptime, minimumSettle: minimumSettle) { return "settle_pending" }
        if sample.sourceTimestamp! <= fence.acknowledgementUptime { return "not_strictly_post_ack" }
        if sample.frameID!.sequence <= (fence.highestSequence ?? 0) { return "below_stop_frame_fence" }
        if sample.sourceTimestamp! <= (fence.highestSourceTimestamp ?? -.infinity) { return "below_stop_time_fence" }
        if let consumed = lastConsumed, consumed.frameID?.generation == sample.frameID!.generation,
           sample.frameID!.sequence <= (consumed.frameID?.sequence ?? 0) || sample.sourceTimestamp! <= (consumed.sourceTimestamp ?? -.infinity) {
            return "not_advancing_consumed_source"
        }
        return nil
    }
}
