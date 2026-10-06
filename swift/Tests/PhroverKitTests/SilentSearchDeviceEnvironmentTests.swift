import Foundation
import XCTest
@testable import PhroverKit

@MainActor
final class SilentSearchDeviceEnvironmentTests: XCTestCase {
    func testReadyRequiresAllDeviceCapabilitiesAndFreshProbe() async {
        let state = DeviceState()
        let environment = makeEnvironment(state)

        await environment.refreshCommandLink()

        XCTAssertEqual(environment.snapshot, .ready(sessionGeneration: 7))
    }

    func testEachMissingCapabilityHasTypedReadinessRequirement() async {
        let cases: [(SilentSearchReadinessRequirement, (DeviceState) -> Void)] = [
            (.tracking, { $0.tracking = .limited }),
            (.lidar, { $0.lidar = false }),
            (.generation, { $0.generation = 0 }),
            (.detector, { $0.detectorLoaded = false }),
            (.detectorLabel, { $0.labels = ["table"] }),
        ]

        for (requirement, mutate) in cases {
            let state = DeviceState()
            mutate(state)
            let environment = makeEnvironment(state)
            await environment.refreshCommandLink()
            XCTAssertTrue(environment.snapshot.missingRequirements.contains(requirement), "\(requirement)")
        }
    }

    func testFailedOrOlderThanTwoSecondProbeMakesCommandLinkUnavailable() async {
        let state = DeviceState()
        let environment = makeEnvironment(state)
        await environment.refreshCommandLink()
        state.now = state.now.addingTimeInterval(2.001)

        XCTAssertTrue(environment.snapshot.missingRequirements.contains(.commandLink))

        state.probeFails = true
        await environment.refreshCommandLink()
        XCTAssertTrue(environment.snapshot.missingRequirements.contains(.commandLink))
    }

    func testProbeExactlyTwoSecondsOldRemainsFresh() async {
        let state = DeviceState()
        let environment = makeEnvironment(state)
        await environment.refreshCommandLink()
        state.now = state.now.addingTimeInterval(2)

        XCTAssertFalse(environment.snapshot.missingRequirements.contains(.commandLink))
    }

    func testSelectedModelLabelDrivesReadiness() async {
        let state = DeviceState()
        state.labels = ["table", "chair"]
        let environment = makeEnvironment(state)
        await environment.refreshCommandLink()

        XCTAssertEqual(environment.supportedTargetLabels, ["chair", "table"])
        environment.targetLabel = "table"
        XCTAssertFalse(environment.snapshot.missingRequirements.contains(.detectorLabel))
        environment.targetLabel = "person"
        XCTAssertTrue(environment.snapshot.missingRequirements.contains(.detectorLabel))
    }

    private func makeEnvironment(_ state: DeviceState) -> SilentSearchDeviceEnvironment {
        SilentSearchDeviceEnvironment(
            targetLabel: "chair",
            tracking: { state.tracking }, generation: { state.generation },
            lidarSupported: { state.lidar }, detectorLoaded: { state.detectorLoaded },
            detectorLabels: { state.labels }, now: { state.now },
            probeLink: {
                if state.probeFails { throw ProbeError.failed }
            }
        )
    }
}

@MainActor
private final class DeviceState {
    var tracking: ARTrackingQuality = .normal
    var generation: UInt64 = 7
    var lidar = true
    var detectorLoaded = true
    var labels: Set<String> = ["chair"]
    var now = Date(timeIntervalSince1970: 100)
    var probeFails = false
}

private enum ProbeError: Error { case failed }
