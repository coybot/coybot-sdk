import XCTest
@testable import PhroverKit

@MainActor
final class OperatorCommandRouterTests: XCTestCase {
    func testLifecycleInhibitionPreemptsDelayedConfirmedStart() async {
        let mission = RecordingOperatorMission()
        mission.suspendStop = true
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)
        let starting = Task { await router.submit("follow me", searchClearance:
            .init(sourceGeneration: 1, origin: .zero, confirmedAt: 0)) }
        await mission.waitForStop()
        router.inhibitMotion()
        mission.releaseStop()
        let result = await starting.value
        XCTAssertEqual(result, .rejected("Follow request cancelled."))
        XCTAssertFalse(follow.state.isActive)
    }

    func testConfirmationPreparationDoesNotStopOtherTabActivity() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow, mayStartFollow: { false })
        let result = await router.prepareInitialFollowSearch()
        XCTAssertEqual(result, .rejected("Stop the other rover activity before following."))
        XCTAssertTrue(mission.events.isEmpty)
        XCTAssertFalse(follow.state.isActive)
    }

    func testBusyMissionRejectionDescribesTheRunningMission() async {
        let mission = RecordingOperatorMission()
        let router = OperatorCommandRouter(mission: mission, follow: RecordingOperatorFollow())
        _ = await router.submit("first request")
        let result = await router.submit("second request")
        XCTAssertEqual(result, .rejected("I’m still working on the previous command. Stop it before sending another request."))
    }

    func testIdleRouterStopStillConfirmsSharedMissionMotion() async {
        let mission = RecordingOperatorMission()
        mission.stopFails = true
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)
        let result = await router.submit("stop")
        XCTAssertEqual(mission.events, ["stop"])
        XCTAssertEqual(result, .rejected("Rover stop could not be confirmed."))
        let next = await router.submit("go to kitchen")
        XCTAssertNotEqual(next, .accepted)
    }

    func testOverlappingFollowRequestsShareFailedAdmission() async {
        let mission = RecordingOperatorMission()
        mission.suspendStop = true
        mission.stopFails = true
        let router = OperatorCommandRouter(mission: mission, follow: RecordingOperatorFollow())
        let first = Task { await router.submit("follow me") }
        await mission.waitForStop()
        let secondEntered = expectation(description: "second follow submitted")
        let second = Task { secondEntered.fulfill(); return await router.submit("follow me") }
        await fulfillment(of: [secondEntered], timeout: 1)
        mission.releaseStop()
        let firstResult = await first.value
        let secondResult = await second.value
        XCTAssertEqual(firstResult, .rejected("Rover stop could not be confirmed."))
        XCTAssertEqual(secondResult, firstResult)
        XCTAssertEqual(mission.events, ["stop"])
    }

    func testNormalizedLocalStopFencesFollowAndBlocksNewMissionUntilConfirmed() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)
        _ = await router.submit("follow me")
        follow.suspendStop = true
        let stopping = Task { await router.submit("  STOP FOLLOWING ME!  ") }
        await follow.waitForStop()
        XCTAssertEqual(follow.inhibitions, 1)
        XCTAssertFalse(follow.state.isActive, "Fence locally before awaiting motor acknowledgement")
        let pending = await router.submit("go to the kitchen")
        XCTAssertNotEqual(pending, .accepted)
        XCTAssertEqual(mission.events, ["stop", "stop"], "Both follow admission and operator stop confirm shared motion without calling the brain")
        follow.releaseStop()
        let confirmed = await stopping.value
        XCTAssertEqual(confirmed, .accepted)
        let next = await router.submit("go to the kitchen")
        XCTAssertEqual(next, .accepted)
        for _ in 0..<20 { await Task.yield() }
        XCTAssertEqual(mission.events, ["stop", "stop", "handle:go to the kitchen"])
    }

    func testFollowPhraseCancelsMissionBeforeStartingFollow() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)

        let response = await router.submit(" Follow Me! ")

        XCTAssertEqual(response, .accepted)
        XCTAssertEqual(mission.events, ["stop"])
        XCTAssertEqual(follow.starts, 1)
        XCTAssertEqual(follow.missionStopsAtStart, 1)
    }

    func testOrdinaryCommandIsForwardedUnchanged() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)

        let response = await router.submit("  please go to the door  ")
        XCTAssertEqual(response, .accepted)
        await Task.yield()
        XCTAssertEqual(mission.events, ["handle:  please go to the door  "])
        XCTAssertEqual(follow.starts, 0)
    }

    func testEmergencyStopVocabularyInhibitsActiveFollowWithoutMissionDispatch() async {
        for phrase in ["emergency stop", "halt", "cancel", "stop now", "please stop at the door"] {
            let mission = RecordingOperatorMission()
            let follow = RecordingOperatorFollow()
            let router = OperatorCommandRouter(mission: mission, follow: follow)
            let start = await router.submit("follow me")
            XCTAssertEqual(start, .accepted)

            let response = await router.submit(phrase)

            XCTAssertEqual(response, .accepted, phrase)
            XCTAssertGreaterThan(follow.inhibitions, 0, phrase)
            XCTAssertEqual(follow.stopAttempts, 1, phrase)
            XCTAssertFalse(mission.events.contains("handle:\(phrase)"), phrase)
        }
    }

    func testStopPreemptsDelayedFollowStart() async {
        let mission = RecordingOperatorMission()
        mission.suspendStop = true
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)

        let starting = Task { await router.submit("follow me") }
        await mission.waitForStop()
        XCTAssertEqual(follow.starts, 0)
        let stopping = Task { await router.submit("stop") }
        await Task.yield()
        mission.releaseStop()
        _ = await starting.value
        _ = await stopping.value
        XCTAssertEqual(follow.starts, 0)
    }

    func testBlankAndOrdinaryDuringFollowAreRejected() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)
        let blank = await router.submit("  ")
        let followRequest = await router.submit("follow me")
        let ordinary = await router.submit("go to the kitchen")
        XCTAssertNotEqual(blank, .accepted)
        XCTAssertEqual(followRequest, .accepted)
        XCTAssertNotEqual(ordinary, .accepted)
        XCTAssertEqual(follow.starts, 1)
        XCTAssertEqual(mission.events, ["stop"])
    }

    func testFollowDoesNotStartWhileAnotherTabOwnsMotion() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow,
                                           mayStartFollow: { false })
        let response = await router.submit("follow me")
        XCTAssertNotEqual(response, .accepted)
        XCTAssertEqual(follow.starts, 0)
        XCTAssertTrue(mission.events.isEmpty)
    }

    func testRepeatedStopWaitsForMotorAcknowledgement() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)
        _ = await router.submit("follow me")
        follow.suspendStop = true
        let first = Task { await router.stop() }
        await follow.waitForStop()
        var secondFinished = false
        let second = Task { let result = await router.stop(); secondFinished = true; return result }
        for _ in 0..<20 { await Task.yield() }
        XCTAssertFalse(secondFinished, "A second tab request must not assume stop is confirmed")
        follow.releaseStop()
        let firstResult = await first.value
        let secondResult = await second.value
        XCTAssertEqual(firstResult, .accepted)
        XCTAssertEqual(secondResult, .accepted)
    }

    func testFailedStopCanBeRetriedAfterMotorLinkRecovers() async {
        let follow = RecordingOperatorFollow()
        let router = OperatorCommandRouter(mission: RecordingOperatorMission(), follow: follow)
        _ = await router.submit("follow me")
        follow.stopSucceeds = false
        let failed = await router.stop()
        XCTAssertNotEqual(failed, .accepted)
        follow.stopSucceeds = true
        let retried = await router.stop()
        XCTAssertEqual(retried, .accepted)
        XCTAssertEqual(follow.stopAttempts, 2)
    }

    func testDelayedFailedStartCannotClearFailedStopBlock() async {
        let mission = RecordingOperatorMission()
        let follow = RecordingOperatorFollow()
        follow.suspendStart = true
        follow.startSucceeds = false
        follow.stopSucceeds = false
        let router = OperatorCommandRouter(mission: mission, follow: follow)

        let starting = Task { await router.submit("follow me") }
        await follow.waitForStart()
        let stopResult = await router.stop()
        XCTAssertNotEqual(stopResult, .accepted)
        follow.releaseStart()
        let startResult = await starting.value
        XCTAssertNotEqual(startResult, .accepted)

        let next = await router.submit("go to the kitchen")
        XCTAssertNotEqual(next, .accepted, "An unconfirmed stop must keep motion ownership blocked")
        XCTAssertFalse(mission.events.contains("handle:go to the kitchen"))
    }
}

@MainActor
private final class RecordingOperatorMission: OperatorMission {
    var events: [String] = []
    var suspendStop = false
    var stopFails = false
    private var stopWaiter: CheckedContinuation<Void, Never>?
    private var requestedWaiter: CheckedContinuation<Void, Never>?
    private var stopRequested = false

    func handle(_ text: String) async { events.append("handle:\(text)") }
    func cancelCurrentMissionAndWait() async throws {
        events.append("stop")
        stopRequested = true
        requestedWaiter?.resume()
        requestedWaiter = nil
        if suspendStop { await withCheckedContinuation { stopWaiter = $0 } }
        if stopFails { throw NSError(domain: "stop", code: 1) }
    }
    func waitForStop() async {
        if stopRequested { return }
        await withCheckedContinuation { requestedWaiter = $0 }
    }
    func releaseStop() { suspendStop = false; stopWaiter?.resume(); stopWaiter = nil }
}

@MainActor
private final class RecordingOperatorFollow: OperatorFollow {
    var inhibitions = 0
    func inhibitMotion() { inhibitions += 1; state = .stopped }
    var state: FollowMeState = .idle
    var starts = 0
    var suspendStart = false
    var startSucceeds = true
    var missionStopsAtStart = 1
    var suspendStop = false
    var stopSucceeds = true
    private(set) var stopAttempts = 0
    private var stopWaiter: CheckedContinuation<Void, Never>?
    private var startWaiter: CheckedContinuation<Void, Never>?
    private var startRequestedWaiter: CheckedContinuation<Void, Never>?
    private var startRequested = false
    private var requestedWaiter: CheckedContinuation<Void, Never>?
    private var stopRequested = false
    func start() async -> Bool {
        starts += 1
        startRequested = true
        startRequestedWaiter?.resume()
        startRequestedWaiter = nil
        if suspendStart { await withCheckedContinuation { startWaiter = $0 } }
        if startSucceeds { state = .searching }
        return startSucceeds
    }
    func waitForStart() async {
        if startRequested { return }
        await withCheckedContinuation { startRequestedWaiter = $0 }
    }
    func releaseStart() { suspendStart = false; startWaiter?.resume(); startWaiter = nil }
    func stop() async -> Bool {
        stopAttempts += 1
        stopRequested = true
        requestedWaiter?.resume()
        requestedWaiter = nil
        if suspendStop { await withCheckedContinuation { stopWaiter = $0 } }
        if stopSucceeds { state = .stopped }
        return stopSucceeds
    }
    func waitForStop() async {
        if stopRequested { return }
        await withCheckedContinuation { requestedWaiter = $0 }
    }
    func releaseStop() { suspendStop = false; stopWaiter?.resume(); stopWaiter = nil }
}
