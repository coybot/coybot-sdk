import Foundation
@testable import PhroverKit

/// Construct the native worker callback outside MainActor isolation.
final class FollowInferenceCallCounter: @unchecked Sendable {
    private let lock = NSLock()
    private var count = 0
    var value: Int { lock.withLock { count } }
    func makeDetector(inferenceQueue: DispatchQueue? = nil) -> Detector {
        Detector(supportedLabels: ["person"], detectionHandler: { [self] _ in
            lock.withLock { count += 1 }
            return []
        }, inferenceQueue: inferenceQueue)
    }
}
