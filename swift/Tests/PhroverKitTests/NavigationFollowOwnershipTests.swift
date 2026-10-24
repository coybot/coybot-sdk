import Foundation
import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class NavigationFollowOwnershipTests: XCTestCase {
    @MainActor
    private final class Transport {
        var events: [String] = []
        var sendWaiter: CheckedContinuation<Void, Never>?
        var stopWaiter: CheckedContinuation<Void, Never>?
        var suspendSend = false
        var suspendStop = false
        var failStops = false

        func send(_ command: WheelCommand) async {
            events.append("send")
            if suspendSend {
                suspendSend = false
                await withCheckedContinuation { sendWaiter = $0 }
            }
            events.append("send-return")
        }

        func stop() async throws {
            events.append("stop")
            if suspendStop {
                suspendStop = false
                await withCheckedContinuation { stopWaiter = $0 }
            }
            if failStops { throw URLError(.timedOut) }
            events.append("stop-return")
        }

        func releaseSend() { sendWaiter?.resume(); sendWaiter = nil }
        func releaseStop() { stopWaiter?.resume(); stopWaiter = nil }
    }

    private func makeNavigation(_ transport: Transport) -> NavigationController {
        NavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 5 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { await transport.send($0) }, stopRover: { try await transport.stop() },
            sleep: { _ in await Task.yield() })
    }

    private func spinUntil(_ condition: () -> Bool) async {
        for _ in 0..<4000 {
            if condition() { return }
            await Task.yield()
        }
        XCTAssertTrue(condition())
    }

    func testGenericRotationWaitsForSuspendedFollowSendAndConfirmedStop() async {
        let transport = Transport()
        transport.suspendSend = true
        let nav = makeNavigation(transport)
        let follow = Task { await nav.navigateForFollow(to: Vec2(5, 0), stoppingAtForwardClearance: 1.25) }
        await spinUntil { transport.sendWaiter != nil }
        let eventsBeforeReplacement = transport.events
        let rotation = Task { await nav.rotateAndWait(by: 0.5) }
        for _ in 0..<50 { await Task.yield() }
        XCTAssertEqual(transport.events, eventsBeforeReplacement, "The replacement must drain the actual send before stopping")

        transport.releaseSend()
        let result1 = await follow.value
        XCTAssertEqual(result1, .cancelled)
        // The injected follow seam has no generic AR pose, but the generic operation
        // still must confirm its predecessor's stop before it can fail admission.
        let result2 = await rotation.value
        XCTAssertEqual(result2, .failed(.noPose))
        XCTAssertEqual(transport.events.filter { $0 == "send" }.count, 1)
        let sendReturn = transport.events.firstIndex(of: "send-return")!
        XCTAssertTrue(transport.events.dropFirst(sendReturn + 1).contains("stop-return"))
        XCTAssertFalse(nav.updateFollowGoal(Vec2(0, 5)))
    }

    func testFollowReplacementWaitsForIndependentStopConfirmation() async {
        let transport = Transport()
        let nav = makeNavigation(transport)
        let follow = Task { await nav.navigateForFollow(to: Vec2(5, 0), stoppingAtForwardClearance: 1.25) }
        await spinUntil { transport.events.contains("send-return") }
        transport.suspendStop = true
        let stop = Task { try await nav.stopAndConfirm() }
        await spinUntil { transport.stopWaiter != nil }
        let sendsAtStop = transport.events.filter { $0 == "send" }.count
        let replacement = Task { await nav.navigateForFollow(to: Vec2(0, 5), stoppingAtForwardClearance: 1.25) }
        for _ in 0..<50 { await Task.yield() }
        XCTAssertEqual(transport.events.filter { $0 == "send" }.count, sendsAtStop)

        transport.releaseStop()
        _ = try? await stop.value
        let result3 = await follow.value
        XCTAssertEqual(result3, .cancelled)
        await spinUntil { transport.events.filter { $0 == "send" }.count > sendsAtStop }
        XCTAssertEqual(nav.state, .driving, "Old stop completion must not clear replacement ownership")
        await nav.stopAndWait()
        let result4 = await replacement.value
        XCTAssertEqual(result4, .cancelled)
        XCTAssertEqual(nav.state, .idle)
    }

    func testFailedFollowStopPreventsGenericRotationAdmission() async {
        let transport = Transport()
        let nav = makeNavigation(transport)
        let follow = Task { await nav.navigateForFollow(to: Vec2(5, 0), stoppingAtForwardClearance: 1.25) }
        await spinUntil { transport.events.contains("send-return") }
        transport.failStops = true
        let result = await nav.rotateAndWait(by: 0.5)
        XCTAssertEqual(result, .failed(.commandFailed), "Unconfirmed follow stop must fail before generic pose admission")
        let result5 = await follow.value
        XCTAssertEqual(result5, .cancelled)
        XCTAssertEqual(nav.safetyState, .failed(.commandFailed))
        transport.failStops = false
        try? await nav.stopAndConfirm()
    }

    func testFailedExplicitStopBarsGenericMotionBeforeAnyFollowRequest() async {
        let transport = Transport()
        transport.failStops = true
        let nav = makeNavigation(transport)
        do {
            try await nav.stopAndConfirm()
            XCTFail("Expected the explicit stop to fail")
        } catch {}
        let result = await nav.rotateAndWait(by: 0.5)
        XCTAssertEqual(result, .failed(.commandFailed),
            "Generic motion must retain the failed-confirmation barrier even without a preceding follow request")
        XCTAssertEqual(nav.safetyState, .failed(.commandFailed))
        XCTAssertFalse(transport.events.contains("send"))
        transport.failStops = false
        try? await nav.stopAndConfirm()
    }

    func testSupersededQueuedStopStillReportsTransportFailure() async {
        let transport = Transport()
        transport.suspendSend = true
        let nav = makeNavigation(transport)
        let follow = Task { await nav.navigateForFollow(to: Vec2(5, 0), stoppingAtForwardClearance: 1.25) }
        await spinUntil { transport.sendWaiter != nil }
        let confirmation = Task { try await nav.stopAndConfirm() }
        for _ in 0..<50 { await Task.yield() }
        let replacement = Task { await nav.navigateForFollow(to: Vec2(0, 5), stoppingAtForwardClearance: 1.25) }
        for _ in 0..<50 { await Task.yield() }
        transport.failStops = true
        transport.releaseSend()

        do {
            try await confirmation.value
            XCTFail("Superseded explicit confirmation must not report success without an acknowledged stop")
        } catch {
            XCTAssertEqual((error as? URLError)?.code, .timedOut)
        }
        let oldResult = await follow.value
        let replacementResult = await replacement.value
        XCTAssertEqual(oldResult, .cancelled)
        XCTAssertEqual(replacementResult, .failed(.commandFailed))
        XCTAssertEqual(transport.events.filter { $0 == "send" }.count, 1)
        transport.failStops = false
        try? await nav.stopAndConfirm()
    }

    func testFollowFailureStreamKeepsTheFollowRequestAttribution() async {
        let nav = NavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 5 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in throw URLError(.timedOut) }, stopRover: {},
            sleep: { _ in await Task.yield() })
        var failures = nav.followMotionFailures().makeAsyncIterator()
        let context = FollowMotionRequestContext(sessionGeneration: 7, requestToken: 13,
            purpose: .followGoal, phase: "following")
        let result = await nav.performFollowMotion(.following(Vec2(5, 0), 1.25), context: context)
        let failure = await failures.next()
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(failure?.context.request, context)
        XCTAssertEqual(failure?.context.controllerOperationID, result.context.controllerOperationID)
        XCTAssertEqual(failure?.reason, .commandFailed)
    }

    func testGenericFailureUsesUnknownAttributionAndCurrentSnapshot() async {
        let transport = Transport()
        let nav = makeNavigation(transport)
        _ = await nav.navigateForFollow(to: .zero, stoppingAtForwardClearance: 1.25)
        var failures = nav.followMotionFailures().makeAsyncIterator()
        // The public controller's generic AR source has no pose. Its follow runtime
        // has a valid injected pose, so this failure must come from the facade.
        let result = await nav.rotateAndWait(by: 0.5)
        XCTAssertEqual(result, .failed(.noPose))
        let failure = await failures.next()
        XCTAssertEqual(failure?.context, .unknown)
        XCTAssertEqual(failure?.reason, .noPose)

        var newSubscription = nav.followMotionFailures().makeAsyncIterator()
        let snapshot = await newSubscription.next()
        XCTAssertEqual(snapshot?.context, .unknown)
        XCTAssertEqual(snapshot?.reason, .noPose,
            "New observers must see current generic safety state, not the engine's stale idle snapshot")
    }

    func testCancelledFollowCallerDrainsItsStopBeforeReturning() async {
        let transport = Transport()
        let nav = makeNavigation(transport)
        var returned = false
        let follow = Task {
            let result = await nav.navigateForFollow(to: Vec2(5, 0), stoppingAtForwardClearance: 1.25)
            returned = true
            return result
        }
        await spinUntil { transport.events.contains("send-return") }
        transport.suspendStop = true
        follow.cancel()
        await spinUntil { transport.stopWaiter != nil }
        for _ in 0..<50 { await Task.yield() }
        XCTAssertFalse(returned)
        XCTAssertFalse(nav.updateFollowGoal(Vec2(0, 5)))
        transport.releaseStop()
        let result6 = await follow.value
        XCTAssertEqual(result6, .cancelled)
        XCTAssertTrue(returned)
        XCTAssertEqual(nav.state, .idle)
    }
}
