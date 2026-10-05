import XCTest
import Foundation
import PhroverKit
import RoverNav
@testable import PhroverCloud

@MainActor
final class HybridBrainTests: XCTestCase {
    func testUsesOnDeviceFirstWhileOnline() async throws {
        let order = CallOrder()
        let onDevice = RecordingBrain(name: "on_device", order: order, result: .success(.init(decision: .done)))
        let cloud = RecordingBrain(name: "cloud", order: order, result: .success(.init(decision: .say("cloud"))))
        let brain = HybridBrain(cloud: cloud, onDevice: onDevice, isOnline: { true })

        let output = try await brain.nextAction(MissionContext(utterance: "go to the chair"))

        XCTAssertEqual(output.decision, .done)
        XCTAssertEqual(onDevice.callCount, 1)
        XCTAssertEqual(cloud.callCount, 0)
        XCTAssertEqual(order.names, ["on_device"])
    }

    func testUsesCloudOnlyAfterOnDeviceUnavailable() async throws {
        let order = CallOrder()
        let onDevice = RecordingBrain(
            name: "on_device",
            order: order,
            result: .failure(RoverBrainError.onDeviceUnavailable(.modelNotReady))
        )
        let cloud = RecordingBrain(name: "cloud", order: order, result: .success(.init(decision: .done)))
        let brain = HybridBrain(cloud: cloud, onDevice: onDevice, isOnline: { true })

        let output = try await brain.nextAction(MissionContext())

        XCTAssertEqual(output.decision, .done)
        XCTAssertEqual(onDevice.callCount, 1)
        XCTAssertEqual(cloud.callCount, 1)
        XCTAssertEqual(order.names, ["on_device", "cloud"])
    }

    func testSelectionTelemetryIncludesMissionAndFallbackReason() async throws {
        let order = CallOrder()
        let onDevice = RecordingBrain(
            name: "on_device",
            order: order,
            result: .failure(RoverBrainError.onDeviceUnavailable(.modelNotReady))
        )
        let cloud = RecordingBrain(
            name: "cloud",
            order: order,
            result: .success(.init(decision: .done))
        )
        var events: [(name: String, fields: [String: String])] = []
        let brain = HybridBrain(
            cloud: cloud,
            onDevice: onDevice,
            isOnline: { true },
            missionTelemetry: { name, fields in events.append((name, fields)) }
        )

        _ = try await brain.nextAction(MissionContext(missionID: 42))

        XCTAssertEqual(events.map(\.name), ["mission_brain_selected"])
        XCTAssertEqual(events[0].fields["mission"], "42")
        XCTAssertEqual(events[0].fields["brain"], "cloud")
        XCTAssertEqual(events[0].fields["reason"], "on_device_failed")
    }

    func testUsesCloudAfterPrimaryStageTimeout() async throws {
        let order = CallOrder()
        let onDevice = RecordingBrain(
            name: "on_device",
            order: order,
            result: .success(.init(decision: .say("late"))),
            delay: .seconds(1)
        )
        let cloud = RecordingBrain(name: "cloud", order: order, result: .success(.init(decision: .done)))
        let brain = HybridBrain(
            cloud: cloud,
            onDevice: onDevice,
            primaryTimeout: .milliseconds(10),
            isOnline: { true }
        )
        let clock = ContinuousClock()
        let start = clock.now

        let output = try await brain.nextAction(MissionContext())

        XCTAssertEqual(output.decision, .done)
        XCTAssertLessThan(start.duration(to: clock.now), .milliseconds(500))
        await assertEventually("primary task observes timeout cancellation") {
            onDevice.wasCancelled
        }
        XCTAssertEqual(order.names, ["on_device", "cloud"])
    }

    func testLocalOnlyBrainWaitsPastPrimaryTimeoutForOnDeviceResult() async throws {
        let order = CallOrder()
        let onDevice = RecordingBrain(
            name: "on_device",
            order: order,
            result: .success(.init(decision: .done)),
            delay: .milliseconds(50)
        )
        let brain = HybridBrain(
            cloud: nil,
            onDevice: onDevice,
            primaryTimeout: .milliseconds(10),
            isOnline: { true }
        )

        let output = try await brain.nextAction(MissionContext())

        XCTAssertEqual(output.decision, .done)
        XCTAssertFalse(onDevice.wasCancelled)
        XCTAssertEqual(order.names, ["on_device"])
    }

    func testOfflineBrainWaitsPastPrimaryTimeoutForOnDeviceResult() async throws {
        let order = CallOrder()
        let onDevice = RecordingBrain(
            name: "on_device",
            order: order,
            result: .success(.init(decision: .done)),
            delay: .milliseconds(50)
        )
        let cloud = RecordingBrain(
            name: "cloud",
            order: order,
            result: .success(.init(decision: .say("cloud")))
        )
        let brain = HybridBrain(
            cloud: cloud,
            onDevice: onDevice,
            primaryTimeout: .milliseconds(10),
            isOnline: { false }
        )

        let output = try await brain.nextAction(MissionContext())

        XCTAssertEqual(output.decision, .done)
        XCTAssertFalse(onDevice.wasCancelled)
        XCTAssertEqual(cloud.callCount, 0)
        XCTAssertEqual(order.names, ["on_device"])
    }

    func testPrimaryTimeoutDoesNotWaitForNonCooperativeOnDeviceBrain() async throws {
        let order = CallOrder()
        let cancellationProbe = CancellationProbe()
        let onDevice = NonCooperativeBrain(
            name: "on_device",
            order: order,
            cancellationProbe: cancellationProbe
        )
        let cloudCalled = expectation(description: "cloud fallback starts within the primary timeout bound")
        let cloud = RecordingBrain(
            name: "cloud",
            order: order,
            result: .success(.init(decision: .done)),
            onCall: { cloudCalled.fulfill() }
        )
        let brain = HybridBrain(
            cloud: cloud,
            onDevice: onDevice,
            primaryTimeout: .milliseconds(10),
            isOnline: { true }
        )
        let clock = ContinuousClock()
        let start = clock.now
        let decisionTask = Task { @MainActor in
            try await brain.nextAction(MissionContext())
        }
        defer {
            decisionTask.cancel()
            onDevice.finish()
        }

        await fulfillment(of: [cloudCalled], timeout: 0.25)
        XCTAssertTrue(cancellationProbe.wasSignalled)
        onDevice.finish()
        let output = try await decisionTask.value

        XCTAssertEqual(output.decision, .done)
        XCTAssertLessThan(start.duration(to: clock.now), .milliseconds(500))
        XCTAssertEqual(order.names, ["on_device", "cloud"])
    }

    func testCallerCancellationCancelsPrimaryAndNeverStartsCloud() async {
        let order = CallOrder()
        let cancellationProbe = CancellationProbe()
        let onDevice = NonCooperativeBrain(
            name: "on_device",
            order: order,
            cancellationProbe: cancellationProbe
        )
        let cloud = RecordingBrain(name: "cloud", order: order, result: .success(.init(decision: .done)))
        let brain = HybridBrain(
            cloud: cloud,
            onDevice: onDevice,
            primaryTimeout: .seconds(10),
            isOnline: { true }
        )
        let completed = expectation(description: "cancelled HybridBrain call completes")
        let result = BrainResultBox()
        let decisionTask = Task { @MainActor in
            do {
                result.value = .success(try await brain.nextAction(MissionContext()))
            } catch {
                result.value = .failure(error)
            }
            completed.fulfill()
        }
        defer {
            decisionTask.cancel()
            onDevice.finish()
        }

        await onDevice.waitUntilEntered()
        decisionTask.cancel()
        await fulfillment(of: [completed], timeout: 0.25)

        let primaryObservedCancellation = cancellationProbe.wasSignalled
        onDevice.finish()
        await decisionTask.value

        XCTAssertTrue(primaryObservedCancellation)
        XCTAssertEqual(cloud.callCount, 0)
        XCTAssertEqual(order.names, ["on_device"])
        guard case .failure(let error)? = result.value else {
            return XCTFail("Expected caller cancellation to be rethrown")
        }
        XCTAssertTrue(error is CancellationError)
    }

    func testRethrowsWhenOnDeviceAndCloudBothFail() async {
        let order = CallOrder()
        let onDevice = RecordingBrain(name: "on_device", order: order, result: .failure(TestError.onDeviceFailed))
        let cloud = RecordingBrain(name: "cloud", order: order, result: .failure(TestError.cloudFailed))
        let brain = HybridBrain(cloud: cloud, onDevice: onDevice, isOnline: { true })

        do {
            _ = try await brain.nextAction(MissionContext())
            XCTFail("Expected the cloud failure to be rethrown")
        } catch {
            XCTAssertEqual(error as? TestError, .cloudFailed)
        }

        XCTAssertEqual(onDevice.callCount, 1)
        XCTAssertEqual(cloud.callCount, 1)
        XCTAssertEqual(order.names, ["on_device", "cloud"])
    }

    func testDoesNotCallCloudWhileOffline() async {
        let order = CallOrder()
        let onDevice = RecordingBrain(name: "on_device", order: order, result: .failure(TestError.onDeviceFailed))
        let cloud = RecordingBrain(name: "cloud", order: order, result: .success(.init(decision: .done)))
        let brain = HybridBrain(cloud: cloud, onDevice: onDevice, isOnline: { false })

        do {
            _ = try await brain.nextAction(MissionContext())
            XCTFail("Expected the on-device failure to be rethrown")
        } catch {
            XCTAssertEqual(error as? TestError, .onDeviceFailed)
        }

        XCTAssertEqual(onDevice.callCount, 1)
        XCTAssertEqual(cloud.callCount, 0)
        XCTAssertEqual(order.names, ["on_device"])
    }

    func testUnavailableOnDeviceWithoutConfiguredCloudRethrowsForMissionFallback() async {
        let order = CallOrder()
        let onDevice = RecordingBrain(
            name: "on_device",
            order: order,
            result: .failure(RoverBrainError.onDeviceUnavailable(.modelNotReady))
        )
        let brain = HybridBrain(cloud: nil, onDevice: onDevice, isOnline: { true })
        let logBefore = runtimeLog()

        do {
            _ = try await brain.nextAction(MissionContext())
            XCTFail("Expected the typed on-device failure to be rethrown")
        } catch {
            XCTAssertEqual(error as? RoverBrainError, .onDeviceUnavailable(.modelNotReady))
        }

        let selectionLog = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertEqual(onDevice.callCount, 1)
        XCTAssertEqual(order.names, ["on_device"])
        XCTAssertTrue(selectionLog.contains("mission_brain_selected brain=on_device"))
        XCTAssertTrue(selectionLog.contains("reason=on_device_failed"))
        XCTAssertFalse(selectionLog.contains("brain=cloud"))
    }

    func testUnavailableHybridBrainAndMissionAgentShareFallbackTelemetryOrder() async {
        let order = CallOrder()
        let onDevice = RecordingBrain(
            name: "on_device",
            order: order,
            result: .failure(RoverBrainError.onDeviceUnavailable(.modelNotReady))
        )
        var events: [(name: String, fields: [String: String])] = []
        let telemetry: MissionTelemetrySink = { events.append(($0, $1)) }
        let brain = HybridBrain(
            cloud: nil,
            onDevice: onDevice,
            isOnline: { false },
            missionTelemetry: telemetry
        )
        let motion = IntegrationMotion()
        let perception = IntegrationPerception()
        perception.objects = [
            PerceivedObject(
                label: "refrigerator",
                confidence: 0.99,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )
        ]
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: IntegrationVoice(),
            missionTelemetry: telemetry,
            currentBrain: { brain }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(events.map(\.name), [
            "mission_brain_selected",
            "mission_brain_selected",
            "mission_offline_fallback_started",
            "mission_attribute_match",
        ])
        XCTAssertEqual(events.map { $0.fields["mission"] }, ["1", "1", "1", "1"])
        XCTAssertEqual(events[0].fields["brain"], "on_device")
        XCTAssertEqual(events[0].fields["reason"], "on_device_failed")
        XCTAssertEqual(events[1].fields["brain"], "offline_object_fallback")
        XCTAssertEqual(events[1].fields["reason"], "brain_error")
        XCTAssertEqual(motion.navigateCalls, [Vec2(1, 0)])
    }

    private func runtimeLog() -> String {
        guard let url = RuntimeFileLog.logFileURL else { return "" }
        return (try? String(contentsOf: url, encoding: .utf8)) ?? ""
    }

    private func assertEventually(
        _ message: String,
        timeout: Duration = .milliseconds(250),
        condition: @MainActor () -> Bool
    ) async {
        let clock = ContinuousClock()
        let deadline = clock.now.advanced(by: timeout)
        while clock.now < deadline {
            if condition() { return }
            await Task.yield()
        }
        XCTAssertTrue(condition(), message)
    }
}

@MainActor
private final class RecordingBrain: RoverBrain {
    private let name: String
    private let order: CallOrder
    private let result: Result<BrainOutput, Error>
    private let delay: Duration?
    private let onCall: (() -> Void)?
    private(set) var callCount = 0
    private(set) var wasCancelled = false

    init(
        name: String,
        order: CallOrder,
        result: Result<BrainOutput, Error>,
        delay: Duration? = nil,
        onCall: (() -> Void)? = nil
    ) {
        self.name = name
        self.order = order
        self.result = result
        self.delay = delay
        self.onCall = onCall
    }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        callCount += 1
        order.names.append(name)
        onCall?()
        defer { wasCancelled = Task.isCancelled }
        if let delay {
            try await Task.sleep(for: delay)
        }
        return try result.get()
    }
}

@MainActor
private final class NonCooperativeBrain: RoverBrain {
    private let name: String
    private let order: CallOrder
    private let cancellationProbe: CancellationProbe
    private var outputContinuation: CheckedContinuation<BrainOutput, Never>?
    private var enteredContinuation: CheckedContinuation<Void, Never>?
    private(set) var callCount = 0

    init(name: String, order: CallOrder, cancellationProbe: CancellationProbe) {
        self.name = name
        self.order = order
        self.cancellationProbe = cancellationProbe
    }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        callCount += 1
        order.names.append(name)
        return await withTaskCancellationHandler {
            await withCheckedContinuation { continuation in
                outputContinuation = continuation
                enteredContinuation?.resume()
                enteredContinuation = nil
            }
        } onCancel: {
            cancellationProbe.signal()
        }
    }

    func waitUntilEntered() async {
        guard outputContinuation == nil else { return }
        await withCheckedContinuation { continuation in
            enteredContinuation = continuation
        }
    }

    func finish() {
        outputContinuation?.resume(returning: BrainOutput(decision: .say("late")))
        outputContinuation = nil
    }
}

private final class CancellationProbe: @unchecked Sendable {
    private let lock = NSLock()
    private var signalled = false

    var wasSignalled: Bool {
        lock.withLock { signalled }
    }

    func signal() {
        lock.withLock { signalled = true }
    }
}

@MainActor
private final class BrainResultBox {
    var value: Result<BrainOutput, Error>?
}

@MainActor
private final class IntegrationMotion: RoverMotion {
    var state: NavigationController.State = .idle
    private(set) var navigateCalls: [Vec2] = []

    func navigate(to goal: Vec2) {
        navigateCalls.append(goal)
        state = .arrived
    }

    func rotate(by angle: Double) async {
        state = .arrived
    }

    func cancel() {
        state = .idle
    }
}

@MainActor
private final class IntegrationPerception: RoverPerception {
    var pose: Pose2D? = Pose2D(position: .zero, yaw: 0)
    var objects: [PerceivedObject] = []

    func detectObjects() -> [PerceivedObject] { objects }
    func unproject(normalizedPoint: CGPoint) -> Vec2? { Vec2(1, 0) }
    func capturedFrameJPEG() -> Data? { nil }
}

@MainActor
private final class IntegrationVoice: RoverVoice {
    func speak(_ text: String) {}
    func ask(_ question: String, timeout: TimeInterval) async -> String? { nil }
}

@MainActor
private final class CallOrder {
    var names: [String] = []
}

private enum TestError: Error, Equatable {
    case onDeviceFailed
    case cloudFailed
}
