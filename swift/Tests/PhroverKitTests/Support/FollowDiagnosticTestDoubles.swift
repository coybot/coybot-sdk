import Foundation
@testable import PhroverKit

/// Independent clocks: advancing host time does not change UTC unless requested.
@MainActor
final class FollowDiagnosticTestClock {
    var monotonic: Double = 0
    var utc = Date(timeIntervalSince1970: 0)
    var requestedWaits: [TimeInterval] = []

    func sleep(_ duration: TimeInterval) async throws {
        requestedWaits.append(duration)
        monotonic += duration
    }
}

@MainActor
final class FollowDiagnosticRecordingSink {
    private(set) var records: [(event: String, fields: [String: String])] = []

    func append(_ event: String, fields: [String: String]) {
        records.append((event, fields))
    }
}

@MainActor
final class FollowDiagnosticSuspension {
    private(set) var entered = false
    private var waiter: CheckedContinuation<Void, Never>?
    func suspend() async { await withCheckedContinuation { entered = true; waiter = $0 } }
    func waitUntilEntered() async { while !entered { await Task.yield() } }
    func release() { waiter?.resume(); waiter = nil }
}
