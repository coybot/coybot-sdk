import CoreVideo
import Foundation
import simd

@MainActor
final class DepthSafetyProcessor {
    /// ARKit buffers are retained and only read by the projector.
    struct Input: @unchecked Sendable {
        let rawDepthMap: CVPixelBuffer
        let intrinsics: simd_float3x3
        let imageResolution: CGSize?
        let cameraTransform: simd_float4x4
        let timestamp: TimeInterval
        /// Earlier frames are comparable only while world tracking stays normal.
        var trackingNormal = true
    }

    private let evaluate: @Sendable (Input) -> DepthSafetySnapshot
    private let didProcess: @MainActor (DepthSafetySnapshot) -> Void
    private let worker = ProjectionWorker()
    private var lastTimingLog = -Double.infinity
    private var generation: UInt64 = 0
    private var memory = DepthVisibilityMemory()

    nonisolated static func project(_ input: Input) -> DepthSafetySnapshot {
        DepthSafetyEvaluator.ingest(rawDepthMap: input.rawDepthMap, intrinsics: input.intrinsics,
            intrinsicsImageSize: input.imageResolution, cameraTransform: input.cameraTransform,
            timestamp: input.timestamp, calibration: RoverConfig.cameraMountCalibration,
            geometry: RoverConfig.collisionGeometry)
    }

    init(evaluate: @escaping @Sendable (Input) -> DepthSafetySnapshot = DepthSafetyProcessor.project,
         didProcess: @escaping @MainActor (DepthSafetySnapshot) -> Void) {
        self.evaluate = evaluate
        self.didProcess = didProcess
    }

    func submit(_ input: Input) {
        // A structural tracking fault is not a coalescible camera sample.
        // Revoke at ingress so a newer normal waiter cannot erase the loss,
        // and an in-flight pre-loss projection cannot repopulate memory.
        guard input.trackingNormal else { invalidate(); return }
        worker.submit(input, generation: generation, evaluate: evaluate) { [weak self] in
            Task(priority: .userInitiated) { @MainActor [weak self] in
                guard let self, let result = self.worker.takeResult() else { return }
                let snapshot = result.snapshot
                let started = result.started, finished = result.finished
                // Resets, interruptions and missing raw depth revoke old results, even
                // if their native computation could not be cancelled mid-frame.
                let resumed = ProcessInfo.processInfo.systemUptime
                guard result.generation == self.generation else { return }
                var delivered = snapshot
                self.memory.attach(to: &delivered)
                let publicationStarted = ProcessInfo.processInfo.systemUptime
                self.didProcess(delivered)
                let published = ProcessInfo.processInfo.systemUptime
                if published - self.lastTimingLog >= 1 {
                    self.lastTimingLog = published
                    RuntimeFileLog.append("depth_pipeline_timing", fields: [
                        "thermal_state": String(ProcessInfo.processInfo.thermalState.rawValue),
                        "low_power_mode": String(ProcessInfo.processInfo.isLowPowerModeEnabled),
                        "source_timestamp_s": String(snapshot.timestamp),
                        "capture_to_submit_s": String(result.submitted - snapshot.timestamp),
                        "submit_to_worker_s": String(started - result.submitted),
                        "projection_s": String(finished - started),
                        "worker_to_actor_s": String(resumed - finished),
                        "memory_attach_s": String(publicationStarted - resumed),
                        "publication_s": String(published - publicationStarted),
                        "worker_to_publish_s": String(published - finished),
                        "capture_to_publish_s": String(published - snapshot.timestamp)])
                }
            }
        }
    }

    func invalidate() {
        generation &+= 1
        worker.invalidate(generation: generation)
        memory.clear()
    }

    /// All mailbox state is protected by `lock`; only the serial queue evaluates.
    /// Publication is coalesced independently so a busy main actor cannot stall
    /// projection or accumulate an unbounded queue of completed snapshots.
    private final class ProjectionWorker: @unchecked Sendable {
        struct Job {
            let input: Input
            let generation: UInt64
            let submitted: Double
        }
        struct Result {
            let snapshot: DepthSafetySnapshot
            let generation: UInt64
            let submitted: Double
            let started: Double
            let finished: Double
        }
        private let lock = NSLock()
        private let queue = DispatchQueue(label: "us.coybot.depth-safety", qos: .userInteractive)
        private var generation: UInt64 = 0
        private var pending: Job?
        private var latest: Result?
        private var running = false
        private var publicationScheduled = false

        func submit(_ input: Input, generation: UInt64,
                    evaluate: @escaping @Sendable (Input) -> DepthSafetySnapshot,
                    notify: @escaping @Sendable () -> Void) {
            let start = lock.withLock {
                pending = Job(input: input, generation: generation, submitted: ProcessInfo.processInfo.systemUptime)
                if running { return false }
                running = true
                return true
            }
            guard start else { return }
            queue.async { [self] in
                while let job = next() {
                    let started = ProcessInfo.processInfo.systemUptime
                    let snapshot = evaluate(job.input)
                    let result = Result(snapshot: snapshot, generation: job.generation, submitted: job.submitted,
                        started: started, finished: ProcessInfo.processInfo.systemUptime)
                    let shouldNotify = lock.withLock {
                        guard job.generation == self.generation else { return false }
                        latest = result
                        guard !publicationScheduled else { return false }
                        publicationScheduled = true
                        return true
                    }
                    if shouldNotify { notify() }
                }
            }
        }

        private func next() -> Job? {
            lock.withLock {
                guard let job = pending else { running = false; return nil }
                pending = nil
                return job
            }
        }

        func takeResult() -> Result? {
            lock.withLock {
                defer { latest = nil; publicationScheduled = false }
                return latest
            }
        }

        func invalidate(generation: UInt64) {
            lock.withLock {
                self.generation = generation
                pending = nil
                latest = nil
                // Keep the scheduled notification: it will drain either nothing
                // or the newest valid generation, without an extra queued task.
            }
        }
    }
}
