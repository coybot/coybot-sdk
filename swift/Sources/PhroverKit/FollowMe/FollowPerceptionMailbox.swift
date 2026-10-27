import Foundation

/// One newest completed frame, plus the strongest latched unhealthy transition. Terminal
/// lifecycle events take priority and close the subscription. Slow consumers
/// cannot accumulate a history of already-expired inference results.
@MainActor
final class FollowPerceptionMailbox {
    private var latest: FollowFrameBatch?
    private var healthTransition: FollowFrameBatch?
    private var previousTracking: ARTrackingQuality?
    private var previousPoseAvailable: Bool?
    private var previousDepthAvailable: Bool?
    private var terminal: FollowPerceptionEvent?
    private var closed = false
    private var waiter: CheckedContinuation<FollowPerceptionEvent?, Never>?
    private(set) var coalescedFrames = 0

    func publish(_ batch: FollowFrameBatch) {
        guard !closed else { return }
        let lostTracking = batch.trackingQuality != .normal &&
            (batch.trackingQuality != previousTracking || previousPoseAvailable == nil)
        let lostPose = batch.pose == nil && previousPoseAvailable != false
        let lostDepth = !batch.depthAvailable && previousDepthAvailable == true
        let invalidGeometry = batch.pose.map { !$0.position.x.isFinite || !$0.position.y.isFinite || !$0.yaw.isFinite } ?? false
        if lostTracking || lostPose || lostDepth || invalidGeometry || !batch.timestamp.isFinite,
           healthTransition == nil || FollowFrameHealth.safetyTransitionPriority(batch) > FollowFrameHealth.safetyTransitionPriority(healthTransition!) {
            healthTransition = batch
        }
        previousTracking = batch.trackingQuality
        previousPoseAvailable = batch.pose != nil
        previousDepthAvailable = batch.depthAvailable
        if latest != nil { coalescedFrames += 1 }
        latest = batch
        resumeWaitingConsumer()
    }

    func finish(with event: FollowPerceptionEvent) {
        guard !closed else { return }
        closed = true
        latest = nil
        healthTransition = nil
        terminal = event
        resumeWaitingConsumer()
    }

    func cancel() {
        closed = true
        latest = nil
        healthTransition = nil
        terminal = nil
        let suspended = waiter
        waiter = nil
        suspended?.resume(returning: nil)
    }

    func stream(onCancel: @escaping @Sendable () -> Void = {}) -> AsyncStream<FollowPerceptionEvent> {
        let lifetime = FollowPerceptionStreamLifetime {
            onCancel()
            Task { @MainActor [self] in cancel() }
        }
        return AsyncStream(unfolding: { [self, lifetime] in
            _ = lifetime // Own producers for exactly the returned stream's lifetime.
            return await next()
        }, onCancel: { lifetime.cancel() })
    }

    private func take() -> FollowPerceptionEvent? {
        if let terminal { self.terminal = nil; return terminal }
        if let healthTransition {
            self.healthTransition = nil
            if latest?.frameID == healthTransition.frameID { latest = nil }
            return .frame(healthTransition)
        }
        if let latest { self.latest = nil; return .frame(latest) }
        return nil
    }

    private func resumeWaitingConsumer() {
        guard let suspended = waiter else { return }
        let event = take()
        guard event != nil || closed else { return }
        waiter = nil
        suspended.resume(returning: event)
    }

    private func next() async -> FollowPerceptionEvent? {
        await withTaskCancellationHandler {
            guard !Task.isCancelled else { return nil }
            if let event = take() { return event }
            guard !closed else { return nil }
            return await withCheckedContinuation { waiter = $0 }
        } onCancel: {
            Task { @MainActor [self] in cancel() }
        }
    }
}

private final class FollowPerceptionStreamLifetime: @unchecked Sendable {
    private let lock = NSLock()
    private var onCancel: (@Sendable () -> Void)?
    init(_ onCancel: @escaping @Sendable () -> Void) { self.onCancel = onCancel }
    func cancel() {
        let action = lock.withLock {
            let action = onCancel
            onCancel = nil
            return action
        }
        action?()
    }
    deinit { cancel() }
}
