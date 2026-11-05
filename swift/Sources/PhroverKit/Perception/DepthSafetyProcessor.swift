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
    private var pending: (input: Input, generation: UInt64, submitted: Double)?
    private var lastTimingLog = -Double.infinity
    private var task: Task<Void, Never>?
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
        // Bound retained camera buffers to the running capture and the newest waiter.
        pending = (input, generation, ProcessInfo.processInfo.systemUptime)
        guard task == nil else { return }
        task = Task { [self] in
            defer { task = nil }
            while let job = pending {
                pending = nil
                let evaluate = evaluate
                let (snapshot, started, finished) = await Task.detached(priority: .userInitiated) {
                    let started = ProcessInfo.processInfo.systemUptime
                    let result = evaluate(job.input)
                    return (result, started, ProcessInfo.processInfo.systemUptime)
                }.value
                // Resets, interruptions and missing raw depth revoke old results, even
                // if their native computation could not be cancelled mid-frame.
                guard job.generation == generation else { continue }
                var delivered = snapshot
                memory.attach(to: &delivered)
                didProcess(delivered)
                let published = ProcessInfo.processInfo.systemUptime
                if published - lastTimingLog >= 1 {
                    lastTimingLog = published
                    RuntimeFileLog.append("depth_pipeline_timing", fields: [
                        "source_timestamp_s": String(job.input.timestamp),
                        "capture_to_submit_s": String(job.submitted - job.input.timestamp),
                        "submit_to_worker_s": String(started - job.submitted),
                        "projection_s": String(finished - started),
                        "worker_to_publish_s": String(published - finished),
                        "capture_to_publish_s": String(published - job.input.timestamp)])
                }
            }
        }
    }

    func invalidate() {
        generation &+= 1
        pending = nil
        memory.clear()
    }
}
