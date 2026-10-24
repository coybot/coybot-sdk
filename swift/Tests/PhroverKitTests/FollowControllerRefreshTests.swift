import Foundation
import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowControllerRefreshTests: XCTestCase {
    private final class CachedPerception: RoverPerception {
        var pose: Pose2D? = .init(position: .zero, yaw: 0)
        var refreshed = false
        var refreshes = 0
        var reads = 0
        var suspendRefresh = false
        var refreshWaiter: CheckedContinuation<Void, Never>?
        func refreshDetections() async {
            refreshes += 1
            if suspendRefresh {
                await withCheckedContinuation { refreshWaiter = $0 }
            }
            refreshed = true
        }
        func detectObjects() -> [PerceivedObject] {
            reads += 1
            guard refreshed else { return [] }
            refreshed = false
            return [.init(label: "person", confidence: 0.95,
                          normalizedPoint: CGPoint(x: 0.5, y: 0.5))]
        }
        func unproject(normalizedPoint: CGPoint) -> Vec2? { Vec2(3, 0) }
        func capturedFrameJPEG() -> Data? { nil }
    }

    private final class Drive: RoverDrive {
        var sends = 0
        let forwardClearance: Double = 5
        func send(v: Double, w: Double) async throws { sends += 1 }
        func stop() async throws {}
        func lastAck() async -> Date? { Date() }
    }

    func testAcquisitionAndEveryFollowTickRefreshTheDetectionCache() async {
        let perception = CachedPerception()
        let drive = Drive()
        let follow = FollowController(perception: perception, drive: drive, tickInterval: 0.001)
        follow.follow(TargetSpec(query: "person"))
        let deadline = ContinuousClock.now + .seconds(1)
        while drive.sends < 2, ContinuousClock.now < deadline {
            try? await Task.sleep(for: .milliseconds(1))
        }
        XCTAssertGreaterThanOrEqual(drive.sends, 2)
        XCTAssertEqual(follow.state, .following)
        XCTAssertEqual(perception.refreshes, perception.reads)
        XCTAssertGreaterThanOrEqual(perception.refreshes, 2)
        follow.cancel()
    }

    func testCancellationDuringRefreshCannotAcquireOrSendAfterItReturns() async {
        let perception = CachedPerception()
        perception.suspendRefresh = true
        let drive = Drive()
        let follow = FollowController(perception: perception, drive: drive, tickInterval: 0.001)
        follow.follow(TargetSpec(query: "person"))
        for _ in 0..<2000 where perception.refreshWaiter == nil { await Task.yield() }
        XCTAssertNotNil(perception.refreshWaiter)
        follow.cancel()
        perception.refreshWaiter?.resume()
        perception.refreshWaiter = nil
        for _ in 0..<50 { await Task.yield() }
        XCTAssertEqual(perception.reads, 0)
        XCTAssertEqual(drive.sends, 0)
        XCTAssertEqual(follow.state, .idle)
        XCTAssertNil(follow.track)
    }
}
