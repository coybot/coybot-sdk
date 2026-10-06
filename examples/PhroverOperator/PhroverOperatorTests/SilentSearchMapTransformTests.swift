import CoreGraphics
import PhroverKit
import XCTest
@testable import PhroverOperator

@MainActor
final class SilentSearchMapTransformTests: XCTestCase {
    func testLiveSafetyMonitorMapsTypedMovementFailuresWithoutParsingMessages() async {
        let navigation = AsyncStream.makeStream(of: NavigationSafetyState.self)
        let link = AsyncStream.makeStream(of: RoverCommandLinkReadiness.self)
        let monitor = LiveSilentSearchSafetyMonitor(
            navigationStates: { navigation.stream },
            commandLinkReadiness: { link.stream }
        )
        let events = monitor.events()

        navigation.continuation.yield(.moving)
        navigation.continuation.yield(.failed(.tipping))

        let event = await firstEvent(from: events)
        XCTAssertEqual(event, .reactiveSafetyFailed)
    }

    func testLiveSafetyMonitorEmitsTransportForMovementFailureAndIdleLinkLoss() async {
        let navigation = AsyncStream.makeStream(of: NavigationSafetyState.self)
        let link = AsyncStream.makeStream(of: RoverCommandLinkReadiness.self)
        let monitor = LiveSilentSearchSafetyMonitor(
            navigationStates: { navigation.stream },
            commandLinkReadiness: { link.stream }
        )
        let events = monitor.events()
        let received = Task { await firstEvent(from: events) }

        navigation.continuation.yield(.failed(.commandFailed))
        let movementEvent = await received.value
        XCTAssertEqual(movementEvent, .transportFailed)

        let idleNavigation = AsyncStream.makeStream(of: NavigationSafetyState.self)
        let idleLink = AsyncStream.makeStream(of: RoverCommandLinkReadiness.self)
        let idleMonitor = LiveSilentSearchSafetyMonitor(
            navigationStates: { idleNavigation.stream },
            commandLinkReadiness: { idleLink.stream }
        )
        let idleEvents = idleMonitor.events()
        let idleReceived = Task { await firstEvent(from: idleEvents) }
        idleNavigation.continuation.yield(.idle)
        idleLink.continuation.yield(.unavailable)

        let idleEvent = await idleReceived.value
        XCTAssertEqual(idleEvent, .transportFailed)
    }

    func testLiveSafetyMonitorDoesNotDuplicateUnsafeTransitions() async {
        let navigation = AsyncStream.makeStream(of: NavigationSafetyState.self)
        let link = AsyncStream.makeStream(of: RoverCommandLinkReadiness.self)
        let monitor = LiveSilentSearchSafetyMonitor(
            navigationStates: { navigation.stream },
            commandLinkReadiness: { link.stream }
        )
        let events = monitor.events()
        let received = Task { () -> [SilentSearchSafetyEvent] in
            var iterator = events.makeAsyncIterator()
            var result: [SilentSearchSafetyEvent] = []
            if let event = await iterator.next() { result.append(event) }
            if let event = await iterator.next() { result.append(event) }
            return result
        }

        navigation.continuation.yield(.failed(.commandFailed))
        navigation.continuation.yield(.failed(.commandFailed))
        link.continuation.yield(.unavailable)
        navigation.continuation.yield(.failed(.tipping))

        let unsafeEvents = await received.value
        XCTAssertEqual(unsafeEvents, [.transportFailed, .reactiveSafetyFailed])
    }

    func testInterruptionEndDoesNotClaimNormalTracking() {
        XCTAssertNil(LiveSilentSearchSafetyMonitor.safetyEvent(for: .interruptionEnded(generation: 7)))
        XCTAssertEqual(
            LiveSilentSearchSafetyMonitor.safetyEvent(for: .interrupted(generation: 7)),
            .trackingLimited(generation: 7)
        )
    }

    func testFitUsesCommonScaleAndCentersMissionBounds() throws {
        let transform = SilentSearchMapTransform.fit(
            points: [CGPoint(x: -1, y: -2), CGPoint(x: 3, y: 2)],
            in: CGSize(width: 240, height: 120),
            padding: 10
        )

        XCTAssertEqual(transform.scale, 25, accuracy: 0.000_001)
        assertPoint(transform.canvasPoint(for: CGPoint(x: -1, y: -2)), x: 70, y: 110)
        assertPoint(transform.canvasPoint(for: CGPoint(x: 3, y: 2)), x: 170, y: 10)
    }

    func testMissionNorthMapsUpAndEastMapsRight() {
        let transform = SilentSearchMapTransform(
            missionCenter: .zero,
            canvasCenter: CGPoint(x: 50, y: 50),
            scale: 10
        )

        assertPoint(transform.canvasPoint(for: CGPoint(x: 1, y: 0)), x: 60, y: 50)
        assertPoint(transform.canvasPoint(for: CGPoint(x: 0, y: 1)), x: 50, y: 40)
    }

    func testEmptyAndDegenerateBoundsRemainFiniteAndCentered() {
        let empty = SilentSearchMapTransform.fit(points: [], in: CGSize(width: 100, height: 80), padding: 8)
        let point = SilentSearchMapTransform.fit(
            points: [CGPoint(x: 4, y: -3)], in: CGSize(width: 100, height: 80), padding: 8
        )

        XCTAssertTrue(empty.scale.isFinite)
        assertPoint(empty.canvasPoint(for: .zero), x: 50, y: 40)
        XCTAssertTrue(point.scale.isFinite)
        assertPoint(point.canvasPoint(for: CGPoint(x: 4, y: -3)), x: 50, y: 40)
    }

    private func assertPoint(_ point: CGPoint, x: CGFloat, y: CGFloat,
                             file: StaticString = #filePath, line: UInt = #line) {
        XCTAssertEqual(point.x, x, accuracy: 0.000_001, file: file, line: line)
        XCTAssertEqual(point.y, y, accuracy: 0.000_001, file: file, line: line)
    }

    private func firstEvent(
        from stream: AsyncStream<SilentSearchSafetyEvent>
    ) async -> SilentSearchSafetyEvent? {
        for await event in stream { return event }
        return nil
    }
}
