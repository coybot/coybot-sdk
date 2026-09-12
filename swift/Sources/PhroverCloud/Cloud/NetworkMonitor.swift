import Foundation
import Network

/// Thin wrapper over `NWPathMonitor` giving a synchronous "am I online right now" snapshot
/// — used by `HybridBrain` to pick cloud vs. on-device per think-tick without every caller
/// needing to stand up its own path monitor.
public final class NetworkMonitor: Sendable {
    public static let shared = NetworkMonitor()

    private let monitor = NWPathMonitor()
    private let lock = NSLock()
    nonisolated(unsafe) private var _isOnline = true

    public init() {
        monitor.pathUpdateHandler = { [weak self] path in
            guard let self else { return }
            self.lock.lock()
            self._isOnline = path.status == .satisfied
            self.lock.unlock()
        }
        monitor.start(queue: DispatchQueue(label: "coybot-sdk.NetworkMonitor"))
    }

    public var isOnline: Bool {
        lock.lock()
        defer { lock.unlock() }
        return _isOnline
    }
}
