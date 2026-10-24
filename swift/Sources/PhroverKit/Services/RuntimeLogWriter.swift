import Foundation

final class RuntimeLogWriter: @unchecked Sendable {
    private let queue = DispatchQueue(label: "us.astral.runtime-log", qos: .utility)
    private let lock = NSLock()
    private let capacity: Int
    private var pending = 0
    private var dropped = 0
    private let reportDropped: @Sendable (Int) -> Void
    init(capacity: Int = 256, reportDropped: @escaping @Sendable (Int) -> Void = { _ in }) {
        self.capacity = max(1, capacity)
        self.reportDropped = reportDropped
    }
    @discardableResult func submit(_ work: @escaping @Sendable () -> Void) -> Bool {
        lock.lock()
        guard pending < capacity else { dropped += 1; lock.unlock(); return false }
        pending += 1
        // Enqueue while holding the short bookkeeping lock to preserve producer
        // order. Work and disk IO execute only on the dedicated serial worker.
        queue.async {
            work()
            let lost = self.lock.withLock { self.pending -= 1; let lost = self.dropped; self.dropped = 0; return lost }
            if lost > 0 { self.reportDropped(lost) }
        }
        lock.unlock()
        return true
    }
    func flush() async { await withCheckedContinuation { continuation in queue.async { continuation.resume() } } }
}
