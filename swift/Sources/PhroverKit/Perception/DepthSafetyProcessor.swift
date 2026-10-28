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
    private var pending: (input: Input, generation: UInt64)?
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
        // Bound retained camera buffers to the running capture and the newest waiter.
        pending = (input, generation)
        guard task == nil else { return }
        task = Task { [self] in
            defer { task = nil }
            while let job = pending {
                pending = nil
                let evaluate = evaluate
                let snapshot = await Task.detached(priority: .userInitiated) {
                    evaluate(job.input)
                }.value
                // Resets, interruptions and missing raw depth revoke old results, even
                // if their native computation could not be cancelled mid-frame.
                guard job.generation == generation else { continue }
                var delivered = snapshot
                if job.input.trackingNormal { memory.attach(to: &delivered) } else { memory.clear() }
                didProcess(delivered)
            }
        }
    }

    func invalidate() {
        generation &+= 1
        pending = nil
        memory.clear()
    }
}
