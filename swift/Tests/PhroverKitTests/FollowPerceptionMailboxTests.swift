import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPerceptionMailboxTests: XCTestCase {
    private func frame(_ sequence: UInt64, tracking: ARTrackingQuality? = .normal, depth: Bool = true) -> FollowFrameBatch {
        .init(frameID: .init(generation: 1, sequence: sequence), timestamp: Double(sequence),
            pose: .init(position: .zero, yaw: 0), depthAvailable: depth, people: [], trackingQuality: tracking)
    }

    func testHealthyBacklogKeepsOnlyNewestCaptureWithoutChangingTimestamp() async {
        let mailbox = FollowPerceptionMailbox()
        for sequence in UInt64(1)...100 { mailbox.publish(frame(sequence)) }
        var iterator = mailbox.stream().makeAsyncIterator()
        guard case .frame(let newest)? = await iterator.next() else { return XCTFail() }
        XCTAssertEqual(newest.frameID.sequence, 100)
        XCTAssertEqual(newest.timestamp, 100)
        XCTAssertEqual(mailbox.coalescedFrames, 99)
    }

    func testTrackingAndDepthLossCannotBeOverwrittenByNewHealthyFrames() async {
        for loss in [frame(2, tracking: .limited), frame(2, tracking: nil), frame(2, depth: false)] {
            let mailbox = FollowPerceptionMailbox()
            mailbox.publish(frame(1))
            mailbox.publish(loss)
            for sequence in UInt64(3)...100 { mailbox.publish(frame(sequence)) }
            var iterator = mailbox.stream().makeAsyncIterator()
            guard case .frame(let unsafe)? = await iterator.next(),
                  case .frame(let newest)? = await iterator.next() else { return XCTFail() }
            XCTAssertEqual(unsafe.frameID, loss.frameID)
            XCTAssertEqual(unsafe.trackingQuality, loss.trackingQuality)
            XCTAssertEqual(unsafe.depthAvailable, loss.depthAvailable)
            XCTAssertEqual(newest.frameID.sequence, 100)
        }
    }

    func testLifecycleFailureWinsOverBacklogAndClosesBeforeLaterFrames() async {
        let mailbox = FollowPerceptionMailbox()
        mailbox.publish(frame(1))
        mailbox.publish(frame(2, tracking: .limited))
        mailbox.finish(with: .failed("reset"))
        mailbox.publish(frame(3))
        var iterator = mailbox.stream().makeAsyncIterator()
        guard case .failed(let message)? = await iterator.next() else { return XCTFail() }
        XCTAssertEqual(message, "reset")
        let end = await iterator.next()
        XCTAssertNil(end)
    }

    func testDepthLossCannotHideSubsequentTrackingLossDuringAcquisition() async {
        let mailbox = FollowPerceptionMailbox()
        mailbox.publish(frame(1))
        mailbox.publish(frame(2, depth: false))
        mailbox.publish(frame(3, tracking: .limited))
        mailbox.publish(frame(4))
        var iterator = mailbox.stream().makeAsyncIterator()
        guard case .frame(let unsafe)? = await iterator.next(),
              case .frame(let latest)? = await iterator.next() else { return XCTFail() }
        XCTAssertEqual(unsafe.frameID.sequence, 3)
        XCTAssertEqual(unsafe.trackingQuality, .limited)
        XCTAssertTrue(unsafe.depthAvailable, "Retain the actual stronger fault's frame; do not merge geometry")
        XCTAssertEqual(latest.frameID.sequence, 4)
    }

    func testCancellationUnblocksSuspendedConsumerAndReleasesProducerOnce() async {
        let mailbox = FollowPerceptionMailbox()
        let released = expectation(description: "release producer")
        released.assertForOverFulfill = true
        let task = Task {
            var iterator = mailbox.stream { released.fulfill() }.makeAsyncIterator()
            return await iterator.next()
        }
        for _ in 0..<20 { await Task.yield() }
        task.cancel()
        let result = await task.value
        XCTAssertNil(result)
        await fulfillment(of: [released], timeout: 1)
        mailbox.publish(frame(1))
    }
}
