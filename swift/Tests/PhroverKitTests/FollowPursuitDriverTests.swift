import XCTest
import RoverNav
import simd
@testable import PhroverKit

@MainActor
final class FollowPursuitDriverTests: XCTestCase {
    func testClearPassageAlternativeRemainsValidWhenOpeningBecomesAdvisory() async {
        let f = Fixture()
        let person = Vec2(3, 0.8)
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
            position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
            centersContinuously: true) }
        var cells = Set<DepthSafetyCell>()
        for x in [-16, 9] {
            for z in 29...50 {
                for y in 2...8 { cells.insert(.init(lateralIndex: x, heightIndex: y, forwardIndex: z)) }
            }
        }
        var advisoryProbes = 0
        f.depthProvider = { command in
            if max(command.left, command.right) > 0.12 {
                return .init(state: .caution, clearance: 0.528, supportCount: 4,
                    sampleAge: 0.10, requiredStoppingDistance: 0.199,
                    motionClass: .classify(command), speedLimit: nil,
                    passage: .init(centerLateral: -0.12, lookAhead: 1.18, width: 0.96))
            }
            let result = DepthSafetyEvaluator.evaluate(.init(timestamp: f.time - 0.109,
                cells: cells, cellResolution: 0.04, validSampleRatio: 1,
                geometry: .init(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04,
                    maximumCollisionHeight: 0.50, lateralSafetyMargin: 0.08),
                projection: nil, failureReason: nil), command: command, now: f.time,
                visibilityPolicy: .astralCompatible, includeDiagnosticCoverage: false,
                minimumObstacleClearance: 0.15)
            XCTAssertEqual(result.state, .clear)
            XCTAssertEqual(result.passageRejectionReason, "distant_supported")
            XCTAssertNil(result.passage, "Advisory geometry must not force normal cruising to slow")
            advisoryProbes += 1
            return result
        }
        f.onSend = { command in
            XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
            XCTAssertGreaterThan(command.right, command.left)
            f.position = person
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(advisoryProbes, 1)
    }

    func testSteeringResponseRequiresContinuousIneffectiveAcknowledgedArcs() {
        for direction in [-1.0, 1.0] {
            for healthy in [false, true] {
                var response = FollowSteeringResponse()
                var requests: [Double] = []
                for step in 0...25 {
                    let time = Double(step) * 0.1
                    let peak = [0.35, 0.12, 0.08][step % 3]
                    let command = direction > 0 ? WheelCommand(left: peak / 2, right: peak) :
                        WheelCommand(left: peak, right: peak / 2)
                    if let request = response.observe(command: command,
                        yaw: healthy ? direction * time * 0.1 : 0,
                        bearing: direction * (healthy ? max(0.1, 0.35 - time * 0.06) : 0.35),
                        now: time, eligible: true) { requests.append(request) }
                }
                XCTAssertEqual(requests, healthy ? [] : [direction])
            }
        }
        var response = FollowSteeringResponse()
        for step in 0...30 {
            XCTAssertNil(response.observe(command: .init(left: 0.06, right: 0.12), yaw: 0,
                bearing: 0.35, now: Double(step), eligible: true), "Long delivery gaps cannot accumulate response evidence")
        }
    }

    func testNearTargetCreepWithNoTranslationSettlesIntoConfirmedHold() async {
        let f = Fixture()
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
            position: Vec2(1.7, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
            centersContinuously: true) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.stops, 1)
        XCTAssertTrue(f.events.contains("near_target_settled_hold"))
        XCTAssertFalse(f.commands.isEmpty)
        XCTAssertTrue(f.commands.allSatisfy { max($0.left, $0.right) <= 0.1 })
    }

    func testCreepResponseDoesNotHoldFarAwayOrWhileMakingProgress() {
        for eligible in [false, true] {
            var response = FollowCreepResponse()
            for step in 0...40 {
                XCTAssertFalse(response.observe(command: .init(left: 0.04, right: 0.04),
                    position: .init(Double(step) * 0.005, 0), now: Double(step) * 0.1, eligible: eligible))
            }
        }
    }

    func testWeakArcResponseStopsBeforeBoundedDepthCheckedAlignment() async {
        for fault in ["none", "blocked_rotation", "stop_failure", "cancel", "no_yaw"] {
            let f = Fixture()
            let person = Vec2(3 * cos(0.35), 3 * sin(0.35))
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                centersContinuously: true) }
            f.stopError = fault == "stop_failure"
            f.onStop = { if fault == "cancel" { f.allowed = false } }
            var rotated = false
            f.depthProvider = { command in
                let rotation = command.left * command.right < 0
                return .init(state: rotation && fault == "blocked_rotation" ? .stop : .clear,
                    clearance: 1, supportCount: 3, sampleAge: 0.01, requiredStoppingDistance: 0.15,
                    motionClass: .classify(command), speedLimit: nil,
                    obstacleSide: rotation ? nil : .right)
            }
            f.onSend = { command in
                if command.left * command.right < 0 {
                    XCTAssertGreaterThanOrEqual(f.stops, 1, "Translation must stop before alignment")
                    XCTAssertEqual(abs(command.left), RoverConfig.minimumRotateWheelSpeed)
                    rotated = true
                    if fault != "no_yaw" { f.yaw += 0.31 }
                } else if rotated {
                    XCTAssertGreaterThanOrEqual(f.stops, 2, "Alignment must stop before translation resumes")
                    f.position = person
                }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, fault == "none" ? .arrived :
                fault == "blocked_rotation" ? .failed(.obstacle) :
                fault == "stop_failure" ? .failed(.commandFailed) :
                fault == "cancel" ? .cancelled : .failed(.stalled), fault)
            XCTAssertEqual(rotated, fault == "none" || fault == "no_yaw", fault)
        }
    }

    func testWeakSteeringTriesBoundedClearApproachBeforeRotation() async {
        for scenario in ["straight", "gentle", "new_stop", "cancel", "settled_heading"] {
        let f = Fixture()
        let person = Vec2(3 * cos(0.35), 3 * sin(0.35))
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
            position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
            centersContinuously: true) }
        f.depthProvider = { command in
            var result = f.observation(age: 0.01)
            if scenario == "gentle", command.left == command.right { result.state = .stop }
            if scenario == "new_stop", f.events.contains("steering_approach_started") { result.state = .stop }
            return result
        }
        f.onStop = { if scenario == "cancel" { f.allowed = false } }
        f.onSend = { command in
            if scenario == "settled_heading", f.events.contains("steering_approach_completed") {
                XCTAssertFalse(f.events.contains("steering_alignment_started"))
                f.position = person
                return
            }
            if f.events.contains("steering_approach_started") || command.left * command.right < 0 {
                XCTAssertTrue(f.events.contains("steering_approach_started"))
                XCTAssertGreaterThan(command.left, 0)
                XCTAssertGreaterThan(command.right, 0)
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                if scenario == "gentle" { XCTAssertGreaterThan(command.right, command.left) }
                if scenario == "settled_heading" {
                    f.position = Vec2(0.16 * cos(0.35), 0.16 * sin(0.35))
                    f.yaw = 0.35
                } else {
                    f.position = person
                }
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, scenario == "cancel" ? .cancelled :
            scenario == "new_stop" ? .failed(.obstacle) : .arrived, scenario)
        XCTAssertEqual(f.events.contains("steering_approach_started"), scenario != "cancel")
        XCTAssertFalse(f.events.contains("steering_alignment_started"))
        if scenario == "settled_heading" { XCTAssertTrue(f.events.contains("steering_approach_completed")) }
        }
    }

    func testRejectedTransitionProbeRetainsOnlyAvoidancePreferenceForRecovery() async {
        let f = Fixture()
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
            position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25) }
        var recoveryProbes = 0
        var budgetGapInjected = false
        f.depthProvider = { command in
            guard !f.commands.isEmpty else { return f.observation(age: 0.01) }
            if f.stops > 0 {
                recoveryProbes += 1
                XCTAssertTrue(FollowPursuitDriver.isSlowObstacleArc(command),
                    "A rejected transition must not restart with a cruise evaluation")
            }
            let budgetGap = f.stops > 0 && !budgetGapInjected && f.events.contains("depth_stability_wait")
            if budgetGap { budgetGapInjected = true }
            f.time += 0.0173385
            return .init(state: .clear, clearance: 1, supportCount: 3,
                sampleAge: f.stops == 0 ? 0.1895745 : budgetGap ? 0.22 : 0.08, requiredStoppingDistance: 0.15,
                motionClass: .classify(command), speedLimit: nil, obstacleSide: .right)
        }
        f.onSend = { command in
            if f.commands.count == 2 {
                XCTAssertGreaterThanOrEqual(recoveryProbes, 3)
                XCTAssertGreaterThan(command.right, command.left)
                f.position = Vec2(3, 0)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.stops, 1)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertTrue(budgetGapInjected)
        XCTAssertTrue(f.events.contains("depth_stability_preserved"))
    }

    func testDistantOpeningKeepsCruiseUntilNearAdjustmentZone() async {
        let f = Fixture()
        let person = Vec2(3, 0)
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
            position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
            centersContinuously: true) }
        f.depthProvider = { command in
            var cells = Set<DepthSafetyCell>()
            let depths = f.commands.count < 3 ? 35...50 : 5...25
            for x in [-11, 10] {
                for z in depths {
                    for y in 2...8 { cells.insert(.init(lateralIndex: x, heightIndex: y, forwardIndex: z)) }
                }
            }
            let observation = DepthSafetyEvaluator.evaluate(.init(timestamp: f.time - 0.01,
                cells: cells, cellResolution: 0.04, validSampleRatio: 1,
                geometry: .init(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04,
                    maximumCollisionHeight: 0.50, lateralSafetyMargin: 0),
                projection: nil, failureReason: nil), command: command, now: f.time,
                visibilityPolicy: .astralCompatible, includeDiagnosticCoverage: false, minimumObstacleClearance: 0.15)
            if f.commands.count < 3 {
                XCTAssertNil(observation.passage)
                XCTAssertEqual(observation.passageRejectionReason, "distant_supported")
            }
            return observation
        }
        f.onSend = { command in
            if f.commands.count <= 3 {
                XCTAssertGreaterThan(max(command.left, command.right), 0.12, "Clear approach should retain normal speed")
            } else {
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                f.position = person
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 4)
    }

    func testConflictingPassageAvoidanceIsRecheckedAtNativeEntry() async {
        for side in [DepthObstacleSide.left, .right, .front, .both] {
            for fault in ["none", "stop", "opposing_flank"] {
                let f = ControllerFixture()
                f.usePlanningPolicy = true
                var atEntry = false
                f.beforeAttempt = { atEntry = true }
                f.depthOverride = { command in
                    .init(state: atEntry && fault == "stop" ? .stop : .clear,
                        clearance: 1, supportCount: 5, sampleAge: 0.01, requiredStoppingDistance: 0.15,
                        motionClass: .classify(command), speedLimit: nil,
                        obstacleSide: atEntry && fault == "opposing_flank" ? (command.left > command.right ? .right : .left) : side,
                        passage: .init(centerLateral: side == .right ? 0.42 : -0.42, lookAhead: 1.1, width: 0.92))
                }
                let feed = FollowTargetSpeedFeed()
                feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                    position: f.arrival, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                    centersContinuously: true)
                let result = await FollowTargetSpeedScope.$feed.withValue(feed) {
                    await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
                }
                XCTAssertTrue(atEntry)
                XCTAssertEqual(result.result, fault == "none" ? .arrived : .failed(.obstacle))
                XCTAssertEqual(f.commands.count, fault == "none" ? 1 : 0)
                XCTAssertEqual(result.stopOutcome, .confirmed)
            }
        }
    }

    func testPassageFlickerKeepsSlowSpeedAndConflictingGapYieldsToFlank() async {
        for conflict in [false, true] {
            let f = Fixture()
            let person = Vec2(3, 0)
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                centersContinuously: true) }
            f.depthProvider = { command in
                var observation = f.observation(age: 0.01)
                if conflict {
                    observation.obstacleSide = .right
                    observation.passage = .init(centerLateral: 0.42, lookAhead: 1.1, width: 0.92)
                } else if f.commands.isEmpty {
                    observation.passage = .init(centerLateral: -0.1, lookAhead: 1.1, width: 0.84)
                }
                return observation
            }
            f.onSend = { command in
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12 + 1e-9)
                XCTAssertGreaterThan(command.right, command.left, "Keep the left passage preference through detection gaps")
                if f.commands.count >= 8 { f.position = person }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(f.commands.count, 8)
        }
    }

    func testUnknownSideCautionTriesFreshSlowRouteButNeverOverridesStop() async {
        for scenario in ["slow", "straight", "blocked", "stop", "stale"] {
            let f = Fixture()
            let person = Vec2(3, 0.8)
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                centersContinuously: true) }
            var probes = 0
            f.depthProvider = { command in
                probes += 1
                if scenario == "stale", probes > 1 {
                    return .unavailable(.malformedDepth, sampleAge: 1, motionClass: .classify(command))
                }
                if scenario != "stop", scenario != "blocked",
                   max(command.left, command.right) <= 0.12,
                   scenario != "straight" || command.left == command.right {
                    return f.observation(age: 0.01)
                }
                return .init(state: scenario == "stop" ? .stop : .caution,
                    clearance: scenario == "stop" ? 0.1 : 0.54, supportCount: 3,
                    sampleAge: 0.11, requiredStoppingDistance: 0.2,
                    motionClass: .classify(command), speedLimit: 0.12)
            }
            f.onSend = { command in
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                if scenario == "straight" { XCTAssertEqual(command.left, command.right) }
                f.position = person
            }
            let result = await f.driver.run()
            let succeeds = scenario == "slow" || scenario == "straight"
            XCTAssertEqual(result, succeeds ? .arrived :
                scenario == "stale" ? .failed(.depthUnavailable(.malformedDepth)) : .failed(.obstacle), scenario)
            XCTAssertEqual(f.commands.count, succeeds ? 1 : 0, scenario)
            if scenario == "stop" { XCTAssertEqual(probes, 1) }
        }
    }

    func testFrontalWarningKeepsClearEscapeDespiteConflictingGap() async {
        for state in [DepthSafetyState.clear, .caution, .stop] {
            let f = Fixture()
            let person = Vec2(3, 0.8)
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                centersContinuously: true) }
            f.depthProvider = { command in
                var observation = f.observation(age: 0.01)
                observation.obstacleSide = .front
                observation.passage = .init(centerLateral: 0.1, lookAhead: 0.94, width: 0.76)
                observation.state = FollowPursuitDriver.isSlowObstacleArc(command) &&
                    command.right > command.left ? state : .caution
                return observation
            }
            f.onSend = { command in
                XCTAssertGreaterThan(command.right, command.left)
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                f.position = person
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, state == .clear ? .arrived : .failed(.obstacle))
            XCTAssertEqual(f.commands.count, state == .clear ? 1 : 0)
        }
    }

    func testMissingPassageCannotRenewDirectionRetention() async {
        let f = Fixture()
        let person = Vec2(3, 0)
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = { .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
            position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
            centersContinuously: true) }
        var observedAt: Double?
        f.depthProvider = { _ in
            var observation = f.observation(age: 0.01)
            if f.commands.isEmpty {
                observedAt = f.time
                observation.passage = .init(centerLateral: -0.24, lookAhead: 0.66, width: 1.2)
            }
            return observation
        }
        f.onSend = { command in
            if f.time - (observedAt ?? f.time) >= 1.5 {
                XCTAssertEqual(command.left, command.right, "Expired passage must release steering")
                f.position = person
            } else {
                XCTAssertGreaterThan(command.right, command.left)
            }
            f.time += 0.4
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertGreaterThan(f.commands.count, 2)
    }

    func testLiveDepthPassageCentersBetweenWallsInsteadOfFollowingOffAxisPerson() async {
        for offset in [-3, 0, 3] {
            let f = Fixture()
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            let person = Vec2(3 * cos(0.3), 3 * sin(0.3))
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: person, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                    centersContinuously: true)
            }
            var cells = Set<DepthSafetyCell>()
            for lateral in [-11 + offset, 10 + offset] {
                for forward in 5...25 {
                    for height in 2...8 {
                        cells.insert(.init(lateralIndex: lateral, heightIndex: height, forwardIndex: forward))
                    }
                }
            }
            var probes: [String] = []
            f.depthProvider = { command in
                let observation = DepthSafetyEvaluator.evaluate(.init(timestamp: f.time - 0.01, cells: cells, cellResolution: 0.04,
                    validSampleRatio: 1, geometry: .init(length: 0.30, width: 0.25,
                        minimumCollisionHeight: 0.04, maximumCollisionHeight: 0.50, lateralSafetyMargin: 0),
                    projection: nil, failureReason: nil), command: command, now: f.time,
                    visibilityPolicy: .astralCompatible, includeDiagnosticCoverage: false, minimumObstacleClearance: 0.15)
                probes.append("\(command): \(observation.state), side=\(String(describing: observation.obstacleSide)), passage=\(String(describing: observation.passage))")
                return observation
            }
            f.onSend = { command in
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                if offset == 0 {
                    XCTAssertEqual(command.left, command.right, accuracy: 1e-9,
                        "An off-axis person must not pull the rover into a doorway edge")
                } else {
                    XCTAssertEqual(command.left > command.right, offset > 0,
                        "Positive passage offset is rover-right")
                }
                f.position = person
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived, "offset=\(offset), probes=\(probes)")
            XCTAssertEqual(f.commands.count, 1, "offset=\(offset)")
        }
    }

    func testLastSeenApproachHasFixedTimeDistanceAndStandOffLimits() async {
        for limit in ["time", "distance", "stand_off", "obstacle", "depth", "source_reset", "cancel"] {
            let f = Fixture()
            if limit == "stand_off" { f.position = Vec2(0.8, 0) }
            let origin = f.position
            let observedAt = f.time
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: 1), timestamp: observedAt,
                    position: Vec2(2.5, 0), velocity: Vec2(10, 0), standOff: 1.5, minimumDistance: 1.25,
                    centersContinuously: true, lastSeenApproach: .init(origin: origin))
            }
            f.depthProvider = { command in
                if limit == "depth", !f.commands.isEmpty {
                    return .unavailable(.missingRawDepth, sampleAge: .infinity, motionClass: .classify(command))
                }
                if limit == "obstacle", !f.commands.isEmpty {
                    return .init(state: .stop, clearance: 0.1, supportCount: 5, sampleAge: 0.01,
                        requiredStoppingDistance: 0.15, motionClass: .classify(command), speedLimit: nil,
                        obstacleSide: .front)
                }
                return f.observation(age: 0.01)
            }
            f.onSend = { command in
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                XCTAssertGreaterThan(min(command.left, command.right), 0)
                switch limit {
                case "time": f.time = observedAt + 3.01
                case "distance": f.position = Vec2(0.51, 0)
                case "stand_off": f.position = Vec2(1, 0)
                case "source_reset": f.generation += 1
                case "cancel": f.allowed = false
                default: break
                }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, limit == "obstacle" ? .failed(.obstacle) :
                limit == "depth" ? .failed(.depthUnavailable(.missingRawDepth)) :
                limit == "source_reset" ? .failed(.trackingLost) : limit == "cancel" ? .cancelled : .arrived)
            XCTAssertEqual(f.commands.count, 1, limit)
            if limit == "depth" { XCTAssertEqual(f.stops, 1) }
        }
    }

    func testNativeEntryRechecksLastSeenDisplacementAndAuthorityChanges() async {
        for transition in ["before_depth", "during_depth", "measured_to_memory"] {
            let f = ControllerFixture()
            f.usePlanningPolicy = true
            let feed = FollowTargetSpeedFeed()
            feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                position: Vec2(2.5, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                centersContinuously: true,
                lastSeenApproach: transition == "measured_to_memory" ? nil : .init(origin: .zero))
            var atEntry = false, attempts = 0
            f.beforeAttempt = {
                atEntry = true
                attempts += 1
                if transition == "before_depth" { f.position = Vec2(0.51, 0) }
                if transition == "measured_to_memory" { feed.sample?.lastSeenApproach = .init(origin: .zero) }
            }
            f.depthOverride = { command in
                if atEntry, transition == "during_depth" { f.position = Vec2(0.51, 0) }
                return .init(state: .clear, clearance: 2, supportCount: 0, sampleAge: 0.01,
                    requiredStoppingDistance: 0.15, motionClass: .classify(command), speedLimit: nil)
            }
            let result = await FollowTargetSpeedScope.$feed.withValue(feed) {
                await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
            }
            XCTAssertEqual(result.result, .arrived)
            XCTAssertEqual(result.stopOutcome, .confirmed)
            XCTAssertEqual(f.commands.count, transition == "measured_to_memory" ? 1 : 0)
            if let command = f.commands.first {
                XCTAssertGreaterThan(attempts, 1, "Reject the queued fast measured command before memory approach")
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
            }
        }
    }

    func testAvoidanceStrengthensCenteringAsVerifiedTargetApproachesImageEdge() async {
        for side in [DepthObstacleSide.left, .right] {
            let f = Fixture()
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            let angles = [10.0, 20, 27, 35, 10]
            f.targetSample = {
                let angle = angles[min(f.commands.count, angles.count - 1)] * .pi / 180 * (side == .right ? 1 : -1)
                return .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: Vec2(3 * cos(angle), 3 * sin(angle)), velocity: .zero,
                    standOff: 1.5, minimumDistance: 1.25, centersContinuously: true)
            }
            var approved: [WheelCommand] = []
            f.depthProvider = { command in
                approved.append(command)
                f.time += 0.014871
                return .init(state: .clear, clearance: 1, supportCount: 0, sampleAge: f.commands.isEmpty ? 0.01 : 0.203627,
                    requiredStoppingDistance: 0.15, motionClass: .classify(command),
                    speedLimit: nil, obstacleSide: side)
            }
            f.onSend = { command in
                XCTAssertEqual(command.left > command.right, side == .left)
                XCTAssertEqual(approved.last, command, "Every curvature must receive exact depth approval")
                XCTAssertGreaterThan(min(command.left, command.right), 0)
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                if f.commands.count > 1 {
                    XCTAssertEqual(approved.count, 1, "Adaptive retained curve must fit the recorded single-assessment budget")
                }
                approved.removeAll()
                if f.commands.count == angles.count { f.position = Vec2(3, 0) }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(f.stops, 0)
            let yawRequests = f.commands.map { abs($0.left - $0.right) }
            guard yawRequests.count == angles.count else { XCTFail("Missing steering sequence"); continue }
            XCTAssertGreaterThan(yawRequests[1], yawRequests[0])
            XCTAssertGreaterThan(yawRequests[2], 2 * yawRequests[0], "Recorded 27° drift needs stronger centering")
            XCTAssertGreaterThanOrEqual(yawRequests[3], yawRequests[2])
            XCTAssertEqual(yawRequests[4], yawRequests[0], accuracy: 1e-9)
        }
    }

    func testAdaptiveAvoidanceRequiresClearNativeSweepAndRejectsNewOpposingFlank() async {
        for direction in [-1.0, 1.0] {
            for fault in 0...3 {
                let f = ControllerFixture()
                f.usePlanningPolicy = true
                f.arrival = Vec2(3 * cos(0.47), direction * 3 * sin(0.47))
                var atEntry = false
                f.beforeAttempt = { atEntry = true }
                f.depthOverride = { command in
                    let side: DepthObstacleSide = atEntry && fault == 3
                        ? (direction > 0 ? .left : .right) : .front
                    return .init(state: atEntry && fault == 1 ? .caution : atEntry && fault == 2 ? .stop : .clear,
                        clearance: 1, supportCount: 5, sampleAge: 0.01, requiredStoppingDistance: 0.15,
                        motionClass: .classify(command), speedLimit: nil, obstacleSide: side)
                }
                let feed = FollowTargetSpeedFeed()
                feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                    position: f.arrival, velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                    centersContinuously: true)
                let result = await FollowTargetSpeedScope.$feed.withValue(feed) {
                    await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
                }
                XCTAssertTrue(atEntry)
                XCTAssertEqual(result.result, fault == 0 ? .arrived : .failed(.obstacle))
                XCTAssertEqual(f.commands.count, fault == 0 ? 1 : 0)
                XCTAssertEqual(result.stopOutcome, .confirmed)
                if let command = f.commands.first {
                    XCTAssertGreaterThan(abs(command.left - command.right), 0.084)
                    XCTAssertEqual(command.right > command.left, direction > 0)
                    XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                }
            }
        }
    }

    func testRetainedAvoidanceFitsRecordedDepthBudgetWithoutRecoveryStops() async {
        // Recorded capture age/evaluation cost left enough time for one candidate
        // and native admission, but not pursuit + a second avoidance probe.
        for side in [DepthObstacleSide.left, .right] {
            for age in [0.198243, 0.203627] {
                let f = Fixture()
                f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
                f.targetSample = {
                    .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                        position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
                }
                var evaluations = 0
                f.depthProvider = { command in
                    evaluations += 1
                    let sampleAge = f.commands.isEmpty ? 0.01 : age
                    f.time += 0.014871
                    return .init(state: .clear, clearance: 1, supportCount: 0, sampleAge: sampleAge,
                        requiredStoppingDistance: 0.23, motionClass: .classify(command),
                        speedLimit: nil, obstacleSide: side)
                }
                f.onSend = { command in
                    XCTAssertEqual(command.left > command.right, side == .left)
                    XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                    if f.commands.count > 1 {
                        XCTAssertEqual(evaluations, 1, "Retained arc needs one fresh assessment")
                        XCTAssertGreaterThan(f.expiries.last! - f.time, 0.024871)
                    }
                    evaluations = 0
                    if f.commands.count == 8 { f.position = Vec2(3, 0) }
                }
                let result = await f.driver.run()
                XCTAssertEqual(result, .arrived, "side \(side), age \(age)")
                XCTAssertEqual(f.commands.count, 8)
                XCTAssertEqual(f.stops, 0)
            }
        }
    }

    func testRetainedAvoidanceRechecksChangedDepthBeforeSending() async {
        for initialSide in [DepthObstacleSide.left, .right] {
            for fault in 0...5 {
                let f = Fixture()
                f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
                f.targetSample = {
                    .init(frameID: .init(generation: f.generation, sequence: f.sequence), timestamp: f.time,
                        position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
                }
                f.depthProvider = { command in
                    let changed = !f.commands.isEmpty
                    if changed, fault == 3 {
                        return .unavailable(.missingRawDepth, sampleAge: .infinity, motionClass: .classify(command))
                    }
                    let side: DepthObstacleSide? = !changed ? initialSide :
                        fault == 0 ? .front : fault == 1 ? .both : fault == 2 ? nil :
                        (initialSide == .left ? .right : .left)
                    return .init(state: changed && fault < 3 ? .stop : .clear,
                        clearance: changed && fault < 3 ? 0.1 : 1, supportCount: 5,
                        sampleAge: changed && fault == 4 ? 0.3 : 0.01,
                        requiredStoppingDistance: 0.23, motionClass: .classify(command),
                        speedLimit: nil, obstacleSide: side)
                }
                f.onSend = { command in
                    if f.commands.count == 2 {
                        XCTAssertEqual(fault, 5, "Blocked/stale/missing depth cannot authorize motion")
                        XCTAssertEqual(command.left > command.right, initialSide == .right)
                        f.position = Vec2(3, 0)
                    }
                }
                let result = await f.driver.run()
                if fault < 3 { XCTAssertEqual(result, .failed(.obstacle)) }
                else if fault == 5 { XCTAssertEqual(result, .arrived) }
                else {
                    XCTAssertEqual(result, .failed(.depthUnavailable(fault == 3 ? .missingRawDepth : .staleRawDepth)))
                    XCTAssertEqual(f.stops, 1)
                }
                XCTAssertEqual(f.commands.count, fault == 5 ? 2 : 1)
            }
        }
    }

    func testRetainedAvoidanceStillRequiresNativeExactCommandApproval() async {
        for fault in 0...2 {
            let f = ControllerFixture()
            f.usePlanningPolicy = true
            var atSecondEntry = false
            f.depthOverride = { command in
                .init(state: atSecondEntry && fault != 2 ? (fault == 0 ? .stop : .caution) : .clear,
                    clearance: atSecondEntry ? 0.1 : 1, supportCount: 5, sampleAge: 0.01,
                    requiredStoppingDistance: 0.23, motionClass: .classify(command), speedLimit: nil,
                    obstacleSide: atSecondEntry ? (fault == 2 ? .right : .front) : .left)
            }
            f.beforeAttempt = { atSecondEntry = !f.commands.isEmpty }
            let feed = FollowTargetSpeedFeed()
            feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            f.afterSend = { f.position = .zero }
            let result = await FollowTargetSpeedScope.$feed.withValue(feed) {
                await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
            }
            XCTAssertTrue(atSecondEntry)
            XCTAssertEqual(result.result, .failed(.obstacle))
            XCTAssertEqual(f.commands.count, 1, "Retained arc must be rejected before the second native send")
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testContinuousHoldingCorrectsInsideFormerFiveDegreeBand() async {
        for degrees in [-4.0, 4.0] {
            let f = Fixture()
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                let bearing = f.commands.isEmpty ? degrees * .pi / 180 : 0
                return .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: Vec2(1.5 * cos(bearing), 1.5 * sin(bearing)), velocity: .zero,
                    standOff: 1.5, minimumDistance: 1.25, holdsTranslation: true, centersContinuously: true)
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(f.commands.count, 1)
            if let command = f.commands.first {
                XCTAssertEqual(command.left, -command.right)
                XCTAssertEqual(command.right > 0, degrees > 0)
            }
        }
    }
    func testRealDepthCellsDriveMirroredSideAvoidanceAndFrontalStop() async {
        for lateral in [-6, 5, 0] {
            let f = Fixture()
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            }
            let cells = Set((2...4).map { DepthSafetyCell(lateralIndex: lateral, heightIndex: $0, forwardIndex: 10) })
            f.depthProvider = { command in
                let snapshot = DepthSafetySnapshot(timestamp: f.time - 0.01, cells: cells, cellResolution: 0.04,
                    validSampleRatio: 1, geometry: .init(length: 0.30, width: 0.25,
                        minimumCollisionHeight: 0.04, maximumCollisionHeight: 0.50),
                    projection: nil, failureReason: nil)
                return DepthSafetyEvaluator.evaluate(snapshot, command: command, now: f.time,
                    visibilityPolicy: .astralCompatible, includeDiagnosticCoverage: false, minimumObstacleClearance: 0.15)
            }
            f.onSend = { command in
                XCTAssertEqual(command.left > command.right, lateral < 0)
                f.position = Vec2(3, 0)
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, lateral == 0 ? .failed(.obstacle) : .arrived)
            XCTAssertEqual(f.commands.count, lateral == 0 ? 0 : 1)
        }
    }

    func testUnclassifiedReactiveObstacleStopsInsteadOfGuessingASide() async {
        for state in [DepthSafetyState.caution, .stop] {
            let f = Fixture()
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.depthProvider = { command in
                .init(state: state, clearance: 0.35, supportCount: 5, sampleAge: 0.01,
                    requiredStoppingDistance: 0.15, motionClass: .classify(command), speedLimit: 0.12)
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .failed(.obstacle))
            XCTAssertTrue(f.commands.isEmpty)
        }
    }

    func testReactiveSideAvoidanceChecksOnlyOppositeGentleArc() async {
        for side in [DepthObstacleSide.left, .right] {
            for escapeClear in [false, true] {
                let f = Fixture()
                f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
                f.targetSample = {
                    .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                        position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
                }
                var probes: [WheelCommand] = []
                f.depthProvider = { command in
                    probes.append(command)
                    let escape = probes.count > 1
                    if escape {
                        XCTAssertEqual(command.left > command.right, side == .left)
                        XCTAssertEqual(min(command.left, command.right) / max(command.left, command.right), 0.65, accuracy: 1e-9)
                    }
                    return .init(state: escape && escapeClear ? .clear : .caution,
                        clearance: escape && escapeClear ? 1 : 0.35, supportCount: 5, sampleAge: 0.01,
                        requiredStoppingDistance: 0.15, motionClass: .classify(command), speedLimit: 0.12,
                        obstacleSide: side)
                }
                f.onSend = { _ in f.position = Vec2(3, 0) }
                let result = await f.driver.run()
                XCTAssertEqual(result, escapeClear ? .arrived : .failed(.obstacle))
                XCTAssertEqual(f.commands.count, escapeClear ? 1 : 0)
                XCTAssertEqual(probes.count, 2, "Do not try steering toward the obstacle")
            }
        }
    }

    func testRecordedFlankProbeBudgetStopsAndRecoversInsteadOfReportingObstacle() async {
        for side in [DepthObstacleSide.left, .right] {
            for fault in 0...7 {
                let f = Fixture()
                f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
                f.targetSample = {
                    .init(frameID: .init(generation: f.generation, sequence: f.sequence), timestamp: f.time,
                        position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
                }
                var stoppedAt: Double?
                f.onStop = {
                    stoppedAt = f.time
                    if fault == 2 { f.driver.inhibit() }
                    if fault == 3 { f.generation += 1 }
                }
                f.stopError = fault == 4
                f.depthProvider = { command in
                    guard !f.commands.isEmpty else { return f.observation(age: 0.01) }
                    let escape = abs(min(command.left, command.right) / max(command.left, command.right) - 0.65) < 1e-9
                    if f.stops == 0, escape, fault == 7 {
                        return .unavailable(.missingRawDepth, sampleAge: .infinity, motionClass: .classify(command))
                    }
                    if f.stops == 0, escape, fault == 6 { return f.observation(age: 0.3) }
                    let budgetMiss = (f.stops == 0 && fault < 6) || fault == 1
                    // Permanent starvation must fail even after recovery avoids
                    // the redundant cruise probe; a single sweep cannot fit.
                    let age = fault == 1 ? 0.23 : budgetMiss ? 0.198243 : 0.01
                    f.time += 0.018922
                    return .init(state: fault == 5 && f.stops > 0 ? .stop : .clear,
                        clearance: 1, supportCount: 0, sampleAge: age,
                        requiredStoppingDistance: 0.23345, motionClass: .classify(command),
                        speedLimit: nil, obstacleSide: side)
                }
                f.onSend = { command in
                    if f.commands.count == 2 {
                        XCTAssertNotNil(stoppedAt)
                        XCTAssertGreaterThan(f.time - stoppedAt!, 0.06)
                        XCTAssertEqual(command.left > command.right, side == .left)
                        f.position = Vec2(3, 0)
                    }
                }
                let result = await f.driver.run()
                let expected: NavigationResult
                switch fault {
                case 0, 6, 7: expected = .arrived
                case 1: expected = .failed(.depthUnavailable(.staleRawDepth))
                case 2: expected = .cancelled
                case 3: expected = .failed(.trackingLost)
                case 4: expected = .failed(.commandFailed)
                default: expected = .failed(.obstacle)
                }
                XCTAssertEqual(result, expected, "side \(side), fault \(fault)")
                XCTAssertEqual(f.commands.count, fault == 0 || fault >= 6 ? 2 : 1)
                XCTAssertEqual(f.stops, 1)
                if fault == 1 {
                    XCTAssertLessThanOrEqual(f.time - stoppedAt!, 0.65)
                    XCTAssertEqual(f.driver.failureCause, .pursuitDepthRefreshTimeout)
                }
            }
        }
    }

    func testReactiveFrontAndBothSidesStopWithoutTryingAnyArc() async {
        for side in [DepthObstacleSide.front, .both] {
            for state in [DepthSafetyState.stop] {
                let f = Fixture()
                f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
                f.targetSample = {
                    .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                        position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
                }
                var probes = 0
                f.depthProvider = { command in
                    probes += 1
                    return .init(state: state, clearance: 0.35, supportCount: 5, sampleAge: 0.01,
                        requiredStoppingDistance: 0.15, motionClass: .classify(command), speedLimit: 0.12,
                        obstacleSide: side)
                }
                let result = await f.driver.run()
                XCTAssertEqual(result, .failed(.obstacle))
                XCTAssertTrue(f.commands.isEmpty)
                XCTAssertEqual(probes, 1)
            }
        }
    }

    func testEarlyFrontalWarningTriesFreshSlowArcsAndRetainsAvoidanceBriefly() async {
        for side in [DepthObstacleSide.front, .both] {
            for escape in [-1, 0, 1] {
                let f = Fixture()
                f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
                f.targetSample = {
                    .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                        position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
                }
                f.depthProvider = { command in
                    let direction = command.right > command.left ? 1 : -1
                    let slow = max(command.left, command.right) <= 0.12 && command.left != command.right
                    let approved = slow && direction == escape
                    return .init(state: approved || !slow ? .clear : .stop,
                        clearance: approved || !slow ? .infinity : 0.1, supportCount: approved ? 0 : 5,
                        sampleAge: 0.01, requiredStoppingDistance: 0.20687,
                        motionClass: .classify(command), speedLimit: nil,
                        obstacleSide: f.commands.isEmpty ? side : nil)
                }
                f.onSend = { command in
                    XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                    XCTAssertEqual(command.right > command.left, escape == 1)
                    if f.commands.count == 2 { f.position = Vec2(3, 0) }
                }
                let result = await f.driver.run()
                XCTAssertEqual(result, escape == 0 ? .failed(.obstacle) : .arrived)
                XCTAssertEqual(f.commands.count, escape == 0 ? 0 : 2)
            }
        }
    }

    func testAvoidancePreferenceExpiresAndCannotAuthorizeUnknownBlockedPath() async {
        for blocked in [false, true] {
            let f = Fixture()
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            }
            f.depthProvider = { command in
                .init(state: blocked && !f.commands.isEmpty ? .stop : .clear,
                    clearance: blocked ? 0.1 : .infinity, supportCount: blocked ? 5 : 0,
                    sampleAge: 0.01, requiredStoppingDistance: 0.15, motionClass: .classify(command),
                    speedLimit: nil, obstacleSide: f.commands.isEmpty ? .right : nil)
            }
            f.onSend = { command in
                if f.commands.count < 3 {
                    XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                } else {
                    XCTAssertGreaterThan(max(command.left, command.right), 0.12)
                    f.position = Vec2(3, 0)
                }
                if f.commands.count == 2 { f.time += 0.51 }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, blocked ? .failed(.obstacle) : .arrived)
            XCTAssertEqual(f.commands.count, blocked ? 1 : 3)
        }
    }

    func testNewFrontWarningAtNativeEntryRejectsUnprobedFastCommand() async {
        let f = ControllerFixture()
        f.usePlanningPolicy = true
        var atEntry = false
        f.depthOverride = { command in
            .init(state: .clear, clearance: .infinity, supportCount: 0, sampleAge: 0.01,
                requiredStoppingDistance: 0.15, motionClass: .classify(command), speedLimit: nil,
                obstacleSide: atEntry ? .front : nil)
        }
        f.beforeAttempt = { atEntry = true }
        let result = await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
        XCTAssertEqual(result.result, .failed(.obstacle))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testHoldingTurnRequiresNativeTransportDepthApproval() async {
        for blockAtEntry in [false, true] {
            let f = ControllerFixture()
            f.usePlanningPolicy = true
            f.arrival = .zero
            var blocked = false
            let feed = FollowTargetSpeedFeed()
            feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                position: Vec2(1.3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                holdsTranslation: true)
            f.depthOverride = { command in
                XCTAssertEqual(command.left + command.right, 0, accuracy: 1e-9)
                return .init(state: blocked ? .stop : .clear, clearance: blocked ? 0.05 : 1,
                    supportCount: 5, sampleAge: 0.01, requiredStoppingDistance: 0.15,
                    motionClass: .classify(command), speedLimit: nil)
            }
            f.beforeAttempt = { if blockAtEntry { blocked = true } }
            f.afterSend = {
                feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                    position: Vec2(1.4, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                    holdsTranslation: true)
            }
            let result = await FollowTargetSpeedScope.$feed.withValue(feed) {
                await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
            }
            XCTAssertEqual(result.result, blockAtEntry ? .failed(.obstacle) : .arrived)
            XCTAssertEqual(f.commands.count, blockAtEntry ? 0 : 1)
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testHoldingTurnWithoutMeasuredYawProgressStopsAsStalled() async {
        let f = Fixture()
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: Vec2(1.3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25,
                holdsTranslation: true)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertFalse(f.commands.isEmpty)
        XCTAssertTrue(f.commands.allSatisfy { abs($0.left + $0.right) < 1e-9 })
        XCTAssertLessThan(f.time, 13)
    }

    func testCautionSelectsEarlyArcWithoutDuplicateDepthEvaluation() async {
        for evaluationCost in [0.020, 0.025, 0.028] {
            await assertCautionArc(evaluationCost: evaluationCost)
        }
    }

    private func assertCautionArc(evaluationCost: Double) async {
        let f = Fixture()
        var reads = 0
        var capture: Double?
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
        }
        f.depthProvider = { command in
            reads += 1
            if capture == nil { capture = f.time - 0.15 }
            let age = f.time - capture!
            f.time += evaluationCost
            let arc = command.right > command.left
            return .init(state: arc ? .clear : .caution, clearance: arc ? 1 : 0.5,
                supportCount: 5, sampleAge: age, requiredStoppingDistance: 0.15,
                motionClass: .classify(command), speedLimit: arc ? nil : 0.12, obstacleSide: .right)
        }
        f.onSend = { command in
            XCTAssertGreaterThan(command.right, command.left, "Steer before the desired path reaches hard stop")
            XCTAssertEqual(reads, 2, "Evaluate the intended command and one exact arc; transport owns its final recheck")
            XCTAssertEqual(f.stops, 0)
            XCTAssertGreaterThanOrEqual(capture! + 0.25 - f.time, evaluationCost + 0.01,
                "Reserve a full exact-command admission and scheduling margin")
            f.position = Vec2(3, 0)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
    }

    func testHoldingTurnUsesYawProgressWithoutTranslationOrIntermediateStops() async {
        for direction in [-1.0, 1.0] {
            let f = Fixture()
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: Vec2(1.4 * cos(0.6), direction * 1.4 * sin(0.6)),
                    velocity: .zero, standOff: 1.5, minimumDistance: 1.25, holdsTranslation: true)
            }
            f.onSend = { command in
                XCTAssertEqual(command.left + command.right, 0, accuracy: 1e-9)
                XCTAssertGreaterThan((command.right - command.left) * direction, 0)
                XCTAssertLessThanOrEqual(max(abs(command.left), abs(command.right)), 0.25)
                XCTAssertEqual(f.stops, 0)
                f.yaw += direction * 0.015
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived)
            XCTAssertGreaterThanOrEqual(f.commands.count, 5)
            XCTAssertGreaterThan(f.time, 13, "Holding yaw progress must survive the translational stall interval")
            XCTAssertEqual(f.position, .zero)
            XCTAssertEqual(f.stops, 0, "Terminal stop belongs to native controller, not each yaw update")
        }
    }

    func testHoldingTurnCanBecomeForwardDriveUnderSameOwner() async {
        let f = Fixture()
        var phase = 0
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: phase == 1 ? Vec2(3, 1) : (phase == 0 ? Vec2(1.3, 0.5) : Vec2(1.3, 0)),
                velocity: .zero, standOff: 1.5, minimumDistance: 1.25, holdsTranslation: phase != 1)
        }
        f.onSend = { command in
            if phase == 0 { XCTAssertEqual(command.left + command.right, 0, accuracy: 1e-9) }
            else { XCTAssertGreaterThan(command.left, 0); XCTAssertGreaterThan(command.right, 0) }
            XCTAssertEqual(f.stops, 0)
            phase += 1
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertEqual(f.stops, 0)
    }

    func testReactiveObstacleArcIsRecheckedAtNativeTransportEntry() async {
        for obstructAtEntry in [false, true] {
            let f = ControllerFixture()
            f.usePlanningPolicy = true
            f.arrival = Vec2(3, 0)
            var obstructed = false
            f.depthOverride = { command in
                let allowed = command.right > command.left && command.left > 0
                return .init(state: allowed && !obstructed ? .clear : .stop, clearance: allowed && !obstructed ? 1 : 0.1,
                    supportCount: 5, sampleAge: 0.01, requiredStoppingDistance: 0.15,
                    motionClass: .classify(command), speedLimit: nil, obstacleSide: obstructed ? .front : .right)
            }
            f.beforeAttempt = { if obstructAtEntry { obstructed = true } }
            let feed = FollowTargetSpeedFeed()
            feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            let result = await FollowTargetSpeedScope.$feed.withValue(feed) {
                await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
            }
            XCTAssertEqual(result.result, obstructAtEntry ? .failed(.obstacle) : .arrived)
            XCTAssertEqual(f.commands.count, obstructAtEntry ? 0 : 1)
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testEarlyFrontalArcRequiresClearExactCommandAtNativeEntry() async {
        for entryState in [DepthSafetyState.clear, .caution, .stop] {
            let f = ControllerFixture()
            f.usePlanningPolicy = true
            f.arrival = Vec2(3, 0)
            var atEntry = false
            f.depthOverride = { command in
                .init(state: atEntry ? entryState : .clear, clearance: atEntry && entryState != .clear ? 0.1 : .infinity,
                    supportCount: 0, sampleAge: 0.01, requiredStoppingDistance: 0.20687,
                    motionClass: .classify(command), speedLimit: nil, obstacleSide: .front)
            }
            f.beforeAttempt = { atEntry = true }
            let feed = FollowTargetSpeedFeed()
            feed.sample = .init(frameID: f.sample.frameID!, timestamp: f.clock.value,
                position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            let result = await FollowTargetSpeedScope.$feed.withValue(feed) {
                await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
            }
            XCTAssertEqual(result.result, entryState == .clear ? .arrived : .failed(.obstacle))
            XCTAssertEqual(f.commands.count, entryState == .clear ? 1 : 0)
            XCTAssertEqual(result.stopOutcome, .confirmed)
            if let command = f.commands.first {
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                XCTAssertEqual(min(command.left, command.right) / max(command.left, command.right), 0.65, accuracy: 1e-9)
            }
        }
    }

    func testReactiveObstacleUsesOnlyFreshDepthApprovedForwardArcWithoutStopping() async {
        for scenario in 0..<4 {
            let f = Fixture()
            let direction = scenario == 0 ? 1.0 : -1.0
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: Vec2(3, 0), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            }
            f.depthProvider = { command in
                if f.commands.isEmpty { return f.observation(age: 0.01) }
                let correctArc = (command.right - command.left) * direction > 0 &&
                    command.left > 0 && command.right > 0
                if correctArc && scenario != 2 {
                    return f.observation(age: scenario == 3 ? 0.3 : 0.01)
                }
                return .init(state: .stop, clearance: 0.2, supportCount: 5, sampleAge: 0.01,
                    requiredStoppingDistance: 0.25, motionClass: .classify(command), speedLimit: nil,
                    obstacleSide: direction > 0 ? .right : .left)
            }
            f.onSend = { command in
                if f.commands.count == 2 {
                    XCTAssertGreaterThan((command.right - command.left) * direction, 0)
                    XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                    XCTAssertEqual(f.stops, 0)
                    f.position = Vec2(3, 0)
                }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, scenario < 2 ? .arrived :
                scenario == 3 ? .failed(.depthUnavailable(.staleRawDepth)) : .failed(.obstacle), "scenario \(scenario)")
            if scenario == 3 {
                XCTAssertEqual(f.stops, 1)
                XCTAssertEqual(f.driver.failureCause, .pursuitDepthRefreshTimeout)
            }
            XCTAssertEqual(f.commands.count, scenario < 2 ? 2 : 1)
        }
    }

    func testReactiveTargetOwnedStartupCannotDriveWithoutFirstMeasurement() async {
        let f = Fixture()
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        let result = await FollowTargetSpeedScope.$feed.withValue(FollowTargetSpeedFeed()) { await f.driver.run() }
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertEqual(f.stops, 1)
        XCTAssertEqual(f.driver.failureCause, .pursuitTargetRefreshTimeout)
    }

    func testTargetRecoveryAllowsFullPostStopWindowDespiteDelayedAcknowledgement() async {
        for stopDelay in [0.15, 0.353, 0.6] {
            let f = Fixture()
            var stoppedAt: Double?
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                let recovered = stoppedAt.map { f.time >= $0 + 0.3 } ?? false
                return .init(frameID: .init(generation: 1, sequence: f.sequence),
                    timestamp: f.commands.isEmpty || recovered ? f.time : f.time - 0.6,
                    position: Vec2(3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            }
            f.onStop = {
                f.time += stopDelay
                stoppedAt = f.time
            }
            f.onSend = { _ in
                if f.commands.count == 2 {
                    XCTAssertGreaterThanOrEqual(f.time - stoppedAt!, 0.3)
                    XCTAssertLessThan(f.time - stoppedAt!, 0.5)
                    f.position = Vec2(3, 0.5)
                }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived, "stop delay \(stopDelay)")
            XCTAssertEqual(f.commands.count, 2)
            XCTAssertEqual(f.stops, 1)
            XCTAssertNil(f.driver.failureCause)
        }
    }

    func testNativeTargetTimeoutCarriesSpecificCauseAndConfirmedStop() async {
        let f = ControllerFixture()
        f.usePlanningPolicy = true
        let result = await FollowTargetSpeedScope.$feed.withValue(FollowTargetSpeedFeed()) {
            await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
        }
        XCTAssertEqual(result.result, .failed(.trackingLost))
        XCTAssertEqual(result.failure?.context.failureCause, .pursuitTargetRefreshTimeout)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertTrue(f.commands.isEmpty)
        if let failure = result.failure {
            XCTAssertFalse(FollowMotionFailureResolution(failure).message.contains("AR tracking"))
        }
    }

    func testRepeatedTargetLossCannotCreateUnlimitedRecoveryWindows() async {
        let f = Fixture()
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence),
                timestamp: f.commands.count <= f.stops ? f.time : f.time - 0.6,
                position: Vec2(3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertEqual(f.driver.failureCause, .pursuitTargetRecoveryExhausted)
        XCTAssertEqual(f.commands.count, 3)
        XCTAssertEqual(f.stops, 2, "Native terminal cleanup owns the final stop")
    }

    func testReactiveTargetRecoveryCannotResumeCancelledOwner() async {
        let f = Fixture()
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence),
                timestamp: f.time - (f.commands.isEmpty ? 0 : 0.6),
                position: Vec2(3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
        }
        f.onStop = { f.driver.inhibit() }
        let result = await f.driver.run()
        XCTAssertEqual(result, .cancelled)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.stops, 1)
    }

    func testReactiveTargetRecoveryRejectsMissingExpiredWrongGenerationAndPreStopSamples() async {
        for fault in 0..<4 {
            let f = Fixture()
            var lost = false
            var stopCapture = 0.0
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                if lost && fault == 0 { return nil }
                let timestamp = !lost ? f.time : (fault == 3 && f.stops > 0 ? stopCapture : f.time - 0.6)
                return .init(frameID: .init(generation: lost && fault == 2 ? 2 : 1, sequence: f.sequence),
                    timestamp: fault == 2 ? f.time : timestamp,
                    position: Vec2(3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            }
            f.onSend = { _ in
                XCTAssertFalse(lost, "Fault \(fault) must not send a fallback waypoint command")
                lost = true
            }
            f.onStop = { f.time += 0.353; stopCapture = f.time }
            let result = await f.driver.run()
            XCTAssertEqual(result, .failed(.trackingLost), "fault \(fault)")
            XCTAssertEqual(f.driver.failureCause, .pursuitTargetRefreshTimeout)
            XCTAssertEqual(f.commands.count, 1)
            XCTAssertEqual(f.stops, 1)
            XCTAssertLessThanOrEqual(f.time - stopCapture, 0.55)
        }
    }

    func testReactiveTargetRecoveryCannotResumeAfterFailedStop() async {
        let f = Fixture()
        f.stopError = true
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence),
                timestamp: f.time - (f.commands.isEmpty ? 0 : 0.6),
                position: Vec2(3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.commandFailed))
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.stops, 1)
    }

    func testReactiveExpiredTargetStopsInsteadOfFallingBackToWaypointThenResumesFreshTarget() async {
        let f = Fixture()
        var missing = false
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time - (missing ? 0.6 : 0),
                position: Vec2(3, 0.5), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
        }
        f.onStop = { missing = false }
        f.onSend = { _ in
            if f.commands.count == 1 { missing = true }
            else {
                XCTAssertEqual(f.stops, 1, "Expired target must confirm a stop, never drive the old waypoint")
                f.position = Vec2(3, 0.5)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertEqual(f.stops, 1)
    }

    func testReactiveCenteringTracksChangingBearingWithoutStoppingOnClearDepth() async {
        let f = Fixture()
        let bearings = [20.0, 5, 1, 0, -1, -5, -20, 10]
        var index = 0
        func target() -> Vec2 {
            let angle = bearings[index] * .pi / 180
            return Vec2(3 * cos(angle), 3 * sin(angle))
        }
        f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: target(), velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
        }
        f.depthProvider = { command in
            .init(state: .clear, clearance: 1, supportCount: 5, sampleAge: 0.01,
                requiredStoppingDistance: 0.15, motionClass: .classify(command), speedLimit: nil)
        }
        f.onSend = { command in
            XCTAssertEqual(f.stops, 0)
            XCTAssertGreaterThan(command.left, 0)
            XCTAssertGreaterThan(command.right, 0)
            let difference = command.right - command.left
            if bearings[index] == 0 { XCTAssertEqual(difference, 0, accuracy: 1e-9) }
            else { XCTAssertGreaterThan(difference * bearings[index], 0) }
            if abs(bearings[index]) == 20 {
                XCTAssertGreaterThan(abs(difference) / (command.left + command.right), 0.20,
                    "Heading correction must retain priority over translation")
            }
            if index == bearings.count - 1 { f.position = target() }
            else { index += 1 }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, bearings.count)
        XCTAssertEqual(f.stops, 0)
    }

    func testReactiveCenteringDoesNotWeakenWithTargetRangeOrIgnoreSmallOffsets() async {
        for degrees in [-85.0, -60, -44, -20, -5, -1, 0, 1, 5, 20, 44, 60, 85] {
            var ratios: [Double] = []
            for range in [2.0, 3.0, 4.0] {
                let f = Fixture()
                let angle = degrees * .pi / 180
                let target = Vec2(range * cos(angle), range * sin(angle))
                f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
                f.targetSample = {
                    .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                        position: target, velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
                }
                f.onSend = { command in
                    XCTAssertGreaterThan(command.left, 0)
                    XCTAssertGreaterThan(command.right, 0)
                    let ratio = (command.right - command.left) / (command.right + command.left)
                    ratios.append(ratio)
                    if degrees == 0 { XCTAssertEqual(ratio, 0, accuracy: 1e-9) }
                    else { XCTAssertGreaterThan(ratio * degrees, 0, "Even one degree must produce correction") }
                    if abs(degrees) == 20 {
                        XCTAssertGreaterThan(abs(ratio), 0.05, "A persistent 20-degree offset needs meaningful steering")
                    }
                    f.position = target
                }
                let result = await f.driver.run()
                XCTAssertEqual(result, .arrived)
            }
            XCTAssertEqual(ratios.count, 3)
            if ratios.count == 3 {
                XCTAssertEqual(ratios[0], ratios[1], accuracy: 1e-6, "Same bearing needs the same correction regardless of range")
                XCTAssertEqual(ratios[1], ratios[2], accuracy: 1e-6)
            }
        }
    }

    func testReactiveClearanceScopeReachesNativeDepthAndTransportAdmission() async {
        let f = ControllerFixture()
        f.usePlanningPolicy = true
        let result = await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
        XCTAssertEqual(result.result, .arrived)
        XCTAssertFalse(f.commands.isEmpty)
        XCTAssertGreaterThan(f.clearanceScopes.count, 1)
        XCTAssertTrue(f.clearanceScopes.allSatisfy { $0 == 0.15 })
        XCTAssertNil(FollowPlanningScope.minimumObstacleClearance)
    }

    func testReactiveSteeringCentersMeasuredTargetInsteadOfLedWaypoint() async {
        for direction in [-1.0, 1.0] {
            let f = Fixture()
            let target = Vec2(2 * cos(0.35), direction * 2 * sin(0.35))
            f.planProvider = { _, goal in .init(path: [goal], planningMode: .reactiveDepth) }
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: target, velocity: .zero, standOff: 1.5, minimumDistance: 1.25)
            }
            f.onSend = { command in
                XCTAssertGreaterThan(command.left, 0)
                XCTAssertGreaterThan(command.right, 0)
                XCTAssertGreaterThan((command.right - command.left) * direction, 0)
                f.position = target
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(f.commands.count, 1)
        }
    }

    func testNearbyVerifiedTargetUsesRequestedSpeedAfterRecoveryWithoutCatchupThreshold() async {
        let f = Fixture()
        var stale = false
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: Vec2(1.86, 0), velocity: Vec2(0.19, 0), standOff: 1.5, minimumDistance: 1.25)
        }
        f.depthProvider = { _ in stale ? .unavailable(.staleRawDepth, sampleAge: 0.3, motionClass: .forward) : f.observation(age: 0.01) }
        f.onStop = { stale = false }
        f.onSend = { command in
            XCTAssertEqual(command.left, 0.35, accuracy: 1e-9, "No extra crawl ramp for a fresh measured target")
            if f.commands.count == 1 { stale = true } else { f.position = Vec2(5, 0) }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertEqual(f.stops, 1)
    }

    func testPerceptionPauseDuringTerminalCleanupDoesNotInventFailedStop() async {
        let f = ControllerFixture()
        let stopping = expectation(description: "terminal stop held")
        let gate = FollowDiagnosticSuspension()
        var held = false
        f.stopHandler = {
            if !f.commands.isEmpty, !held {
                held = true
                stopping.fulfill()
                await gate.suspend()
            }
        }
        let running = Task { await f.run() }
        await fulfillment(of: [stopping], timeout: 2)
        let pause = await f.controller.pauseFollowForPerception()
        XCTAssertNil(pause, "The driver finished; its pending cleanup is not a failed stop")
        gate.release()
        let result = await running.value
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testReactiveModeBypassesMeshButStillRequiresLiveDepth() async {
        for scenario in 0..<4 {
            let f = ControllerFixture()
            f.usePlanningPolicy = true
            f.planningFailure = .startBlocked
            f.obstacle = scenario == 1
            if scenario == 2 { f.freshAt = 100 }
            if scenario == 3 { f.beforeAttempt = { f.obstacle = true } }
            let result = await FollowPlanningScope.$mode.withValue(.reactiveDepth) { await f.run() }
            XCTAssertEqual(f.meshPlanningCalls, 0)
            if scenario == 0 {
                XCTAssertEqual(result.result, .arrived)
                XCTAssertFalse(f.commands.isEmpty)
            } else {
                XCTAssertEqual(result.result, scenario == 2 ? .failed(.depthUnavailable(.staleRawDepth)) : .failed(.obstacle))
                XCTAssertTrue(f.commands.isEmpty)
            }
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
        let mesh = ControllerFixture()
        mesh.usePlanningPolicy = true
        mesh.planningFailure = .startBlocked
        let result = await mesh.run()
        XCTAssertEqual(result.result, .failed(.noPath))
        XCTAssertEqual(mesh.meshPlanningCalls, 1, "Reactive mode must not leak into later requests")
        XCTAssertTrue(mesh.commands.isEmpty)
    }

    func testDirectDepthPlanUsesCurrentPoseAndLatestTargetWithoutMeshJoin() async {
        let f = Fixture()
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 1 {
                f.position = Vec2(0.1, 0)
                f.sequence += 1
                _ = f.driver.updateGoal(Vec2(6, 0))
            }
            return .init(path: [goal], planningMode: .reactiveDepth)
        }
        f.onSend = { _ in
            XCTAssertEqual(f.paths.last?.last, Vec2(6, 0))
            f.position = Vec2(6, 0)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.plannedGoals.count, 1)
        XCTAssertEqual(f.commands.count, 1)
    }
    func testCatchupStartsAtRequestedSpeedAndDoesNotRerampAfterDepthRecovery() async {
        let f = Fixture()
        var stale = false
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: Vec2(6.5, 0), velocity: Vec2(0.2, 0), standOff: 1.5, minimumDistance: 1.25)
        }
        f.depthProvider = { _ in stale ? .unavailable(.staleRawDepth, sampleAge: 0.3, motionClass: .forward) : f.observation(age: 0.01) }
        f.onStop = { stale = false }
        f.onSend = { command in
            XCTAssertEqual(command.left, 0.35, accuracy: 1e-9)
            XCTAssertEqual(command.right, 0.35, accuracy: 1e-9)
            if f.commands.count == 1 { stale = true } else { f.position = Vec2(5, 0) }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertEqual(f.stops, 1)
        XCTAssertTrue(f.events.contains("depth_wait_recovered"))
    }

    func testCatchupRampBypassStillHonorsCautionAndStaleDepth() async {
        for stale in [false, true] {
            let f = Fixture()
            f.targetSample = {
                .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                    position: Vec2(6.5, 0), velocity: Vec2(0.2, 0), standOff: 1.5, minimumDistance: 1.25)
            }
            f.depthProvider = { command in
                stale ? .unavailable(.staleRawDepth, sampleAge: 0.3, motionClass: .classify(command)) :
                    .init(state: .caution, clearance: 0.49, supportCount: 10, sampleAge: 0.01,
                        requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: 0.12)
            }
            f.onSend = { command in
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.12)
                f.position = Vec2(5, 0)
            }
            let result = await f.driver.run()
            if stale {
                XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
                XCTAssertTrue(f.commands.isEmpty)
            } else {
                XCTAssertEqual(result, .arrived)
                XCTAssertEqual(f.commands.count, 1)
            }
        }
    }

    func testReplanCompletedDuringPauseIsRebuiltInsteadOfTimingOutOnResume() async {
        let f = Fixture()
        let sent = expectation(description: "first command")
        let planning = expectation(description: "periodic replan entered")
        let gate = FollowDiagnosticSuspension()
        f.waitHandler = { _ in try? await Task.sleep(for: .milliseconds(1)) }
        f.planProvider = { start, goal in
            if f.plannedGoals.count == 2 { planning.fulfill(); await gate.suspend() }
            return .init(path: [start, goal], map: Costmap(width: 100, height: 100, resolution: 0.1, origin: Vec2(-1, -1)))
        }
        f.onSend = { _ in if f.commands.count == 1 { sent.fulfill() } }
        let running = Task { await f.driver.run() }
        await fulfillment(of: [sent], timeout: 2)
        f.time = 11.1
        await fulfillment(of: [planning], timeout: 2)
        let paused = await f.driver.pauseForPerception()
        XCTAssertTrue(paused)
        gate.release()
        for _ in 0..<30 { await Task.yield() }
        f.time = 12.4; f.sequence += 1
        f.waitHandler = { _ in f.time += 0.02; f.sequence += 1; await Task.yield() }
        f.onSend = { _ in
            XCTAssertGreaterThanOrEqual(f.plannedGoals.count, 3)
            f.position = f.driver.goal
        }
        XCTAssertTrue(f.driver.resumePerception(timestamp: 12.39))
        let result = await running.value
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.stops, 1)
        XCTAssertTrue(f.records.contains { $0.0 == "planning_superseded" &&
            $0.1["acceptance_reason"] == .string("expired_plan_after_perception_pause") })
    }
    func testConfirmedPerceptionPauseDoesNotConsumeCommandedProgressTime() async {
        let f = Fixture()
        f.planProvider = { start, goal in
            .init(path: [start, goal], map: Costmap(width: 100, height: 100, resolution: 0.1, origin: Vec2(-1, -1)))
        }
        let sent = expectation(description: "first command")
        f.waitHandler = { _ in try? await Task.sleep(for: .milliseconds(1)) }
        f.onSend = { _ in if f.commands.count == 1 { sent.fulfill() } }
        let running = Task { await f.driver.run() }
        await fulfillment(of: [sent], timeout: 2)
        f.time += 2.0
        let paused = await f.driver.pauseForPerception()
        XCTAssertTrue(paused)
        f.time += 1.8; f.sequence += 1
        f.onSend = { _ in f.position = f.driver.goal }
        XCTAssertTrue(f.driver.resumePerception(timestamp: f.time - 0.01))
        let result = await running.value
        XCTAssertEqual(result, .arrived, "Confirmed stationary time must not produce a false movement stall")
    }

    func testTargetSpeedExpiryBoundsAdmissionWithoutSpendingDepthRetry() async {
        let f = Fixture()
        f.time = 10.49
        f.waitHandler = { _ in await Task.yield() }
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: 1), timestamp: 10,
                position: Vec2(6.5, 0), velocity: Vec2(0.2, 0), standOff: 1.5, minimumDistance: 1.25)
        }
        f.onSend = { _ in
            if f.commands.count == 1 {
                f.time = 10.51
                throw FollowPursuitSendFailure(reason: .depthUnavailable(.staleRawDepth), definitelyUnsent: true, authorizationExpired: true)
            }
            f.position = Vec2(5, 0)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.expiries.first ?? 0, 10.5, accuracy: 1e-9)
        XCTAssertEqual(f.stops, 0)
        XCTAssertFalse(f.events.contains("authorization_expired"))
        XCTAssertLessThanOrEqual(f.commands.last?.left ?? 1, 0.1)
    }

    func testNativeOwnerIsPreservedWhileTerminalStopCancelsInnerLoop() async {
        let f = ControllerFixture()
        let stopping = expectation(description: "terminal stop held")
        let gate = FollowDiagnosticSuspension()
        var held = false
        f.stopHandler = {
            if !f.commands.isEmpty, !held {
                held = true
                stopping.fulfill()
                await gate.suspend()
            }
        }
        let running = Task { await f.run() }
        await fulfillment(of: [stopping], timeout: 2)
        XCTAssertTrue(f.controller.preservesFollowGoalOwner)
        XCTAssertFalse(f.controller.updateFollowGoal(Vec2(6, 0)))
        XCTAssertEqual(f.plannedGoals.count, 1)
        gate.release()
        let result = await running.value
        XCTAssertEqual(result.result, .arrived)
        XCTAssertFalse(f.controller.preservesFollowGoalOwner)
    }
    func testFailedPerceptionPauseStopsOwnerAndCannotResume() async {
        let f = Fixture()
        let sent = expectation(description: "first command")
        f.waitHandler = { _ in try? await Task.sleep(for: .milliseconds(1)) }
        f.onSend = { _ in if f.commands.count == 1 { sent.fulfill() } }
        let running = Task { await f.driver.run() }
        await fulfillment(of: [sent], timeout: 2)
        f.stopError = true
        let paused = await f.driver.pauseForPerception()
        XCTAssertFalse(paused)
        let result = await running.value
        XCTAssertEqual(result, .failed(.commandFailed))
        XCTAssertFalse(f.driver.resumePerception(timestamp: f.time))
        XCTAssertFalse(f.driver.updateGoal(Vec2(6, 0)))
    }
    func testPerceptionPauseKeepsOneOwnerAndNewestGoalWithoutRepeatedStops() async {
        let f = Fixture()
        let sent = expectation(description: "first command")
        f.autoAdvance = false
        f.waitHandler = { _ in try? await Task.sleep(for: .milliseconds(1)) }
        f.onSend = { _ in if f.commands.count == 1 { sent.fulfill() } }
        let running = Task { await f.driver.run() }
        await fulfillment(of: [sent], timeout: 2)
        let paused = await f.driver.pauseForPerception()
        XCTAssertTrue(paused)
        let sends = f.commands.count
        XCTAssertEqual(f.stops, 1)
        XCTAssertTrue(f.driver.updateGoal(Vec2(6, 0)))
        XCTAssertTrue(f.driver.updateGoal(Vec2(7, 0)))
        let repeated = await f.driver.pauseForPerception()
        XCTAssertTrue(repeated)
        XCTAssertEqual(f.stops, 1)
        XCTAssertEqual(f.commands.count, sends)
        XCTAssertFalse(f.driver.resumePerception(timestamp: f.time - 0.1))
        f.time += 0.1; f.sequence += 1
        f.waitHandler = { _ in f.time += 0.02; f.sequence += 1; await Task.yield() }
        f.onSend = { _ in
            XCTAssertEqual(f.paths.last?.last, f.driver.goal, "Resume must wait for the newest paused goal's route")
            f.position = f.driver.goal
        }
        XCTAssertTrue(f.driver.resumePerception(timestamp: f.time - 0.01))
        let result = await running.value
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.driver.goal, Vec2(7, 0))
        XCTAssertEqual(f.stops, 1)
        XCTAssertGreaterThan(f.commands.count, sends)
    }

    func testFreshMovingTargetAvoidsStaticWaypointApproachBraking() async {
        let f = Fixture()
        f.position = Vec2(4.6, 0)
        f.targetSample = {
            .init(frameID: .init(generation: 1, sequence: f.sequence), timestamp: f.time,
                position: Vec2(6.5, 0), velocity: Vec2(0.2, 0), standOff: 1.5, minimumDistance: 1.25)
        }
        f.onSend = { _ in if f.commands.count == 5 { f.position = Vec2(5, 0) } }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertGreaterThan(f.commands.map { max($0.left, $0.right) }.max() ?? 0, 0.15)
        XCTAssertTrue(f.commands.allSatisfy { max($0.left, $0.right) <= 0.35 })
    }
    func testFreshDepthDoesNotWaitBehindAcknowledgementActorBeforeEverySend() async {
        let f = Fixture()
        var capture = f.time - 0.15
        var queuedReads = 0
        f.acknowledgementSnapshot = { Date(timeIntervalSince1970: f.time) }
        f.acknowledgement = {
            queuedReads += 1
            f.time += 0.06 // Recorded 32–55 ms acknowledgement scheduling delay.
            return Date(timeIntervalSince1970: f.time)
        }
        f.depthProvider = { _ in
            let age = f.time - capture
            f.time += 0.002
            return f.observation(age: age)
        }
        f.waitHandler = { duration in
            f.time += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
            f.sequence += 1
            capture = f.time - 0.15
            await Task.yield()
        }
        f.onSend = { _ in
            f.time += 0.037146375
            f.sendAdmissionDuration = 0.037146375
            XCTAssertLessThan(f.time - capture, 0.25)
            if f.commands.count == 5 { f.position = Vec2(5, 0) }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 5)
        XCTAssertEqual(f.stops, 0)
        XCTAssertEqual(queuedReads, 0)
    }

    func testSynchronousAcknowledgementCannotInventOrRenewFreshness() async {
        for offset: Double? in [nil, -2.1, 0.1] {
            let f = Fixture()
            f.acknowledgementSnapshot = { offset.map { Date(timeIntervalSince1970: f.time + $0) } }
            f.acknowledgement = { XCTFail("Do not substitute another acknowledgement for invalid snapshot evidence"); return Date() }
            let result = await f.driver.run()
            XCTAssertEqual(result, .failed(.commsLost))
            XCTAssertTrue(f.commands.isEmpty)
        }
    }

    func testCautionRampRoundingCannotBecomeTransportObstacleFailure() async {
        let f = ControllerFixture()
        f.caution = true
        f.arrival = .zero
        f.sleepHandler = { _ in
            if f.commands.count == 1, let entry = f.firstSendTime {
                // Pick a real ramp timestamp whose multiply/divide limiting
                // rounds one ULP above 0.12 before the fix.
                let next = (801...1900).map { entry + Double($0) / 10000 }.first { time in
                    let peak = 0.1 + 0.25 * (time - entry)
                    return peak * (0.12 / peak) > 0.12
                }
                XCTAssertNotNil(next)
                f.clock.value = next ?? (entry + 0.15)
            } else { f.clock.value += 0.02 }
            f.sequence += 1
            await Task.yield()
        }
        f.afterSend = { if f.commands.count == 2 { f.position = Vec2(5, 0) } }
        let result = await f.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertTrue(f.commands.allSatisfy { max(abs($0.left), abs($0.right)) <= 0.12 })
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testTransportStillRejectsPreparedCommandWhenCautionLimitTightens() async {
        for limit in [0.08, 0.12.nextDown] {
            let f = ControllerFixture()
            f.caution = true
            f.arrival = .zero
            f.beforeAttempt = { if !f.commands.isEmpty { f.cautionLimit = limit } }
            let result = await f.run()
            XCTAssertEqual(result.result, .failed(.obstacle))
            XCTAssertEqual(f.commands.count, 1, "The request prepared for the older cap must not enter transport")
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testUnresolvedStartingGeometryHasDistinctFailureWithoutMotion() async {
        let f = Fixture()
        f.planProvider = { _, _ in .init(path: [], rejection: .startBlocked,
            diagnostics: ["start_geometry_conflict": "true"]) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertEqual(f.driver.failureCause, .pursuitMapGeometryConflict)
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testApproachingNormalStopUsesLowSpeedBeforeArrival() async {
        let f = Fixture()
        f.position = Vec2(4.7, 0) // 30 cm from the stand-off goal, just outside arrival.
        f.onSend = { command in
            XCTAssertGreaterThan(command.left, 0)
            XCTAssertLessThanOrEqual(max(command.left, command.right), 0.04 + 1e-9)
            f.position = Vec2(4.81, 0)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
    }

    func testMovingGoalDuringEveryReplanCanReplaceActiveRouteWithoutStopping() async {
        let f = Fixture()
        let map = Costmap(width: 100, height: 100, resolution: 0.1, origin: Vec2(-1, -1))
        f.planProvider = { start, goal in
            f.time += 0.3
            _ = f.driver.updateGoal(goal + Vec2(0.12, 0))
            return .init(path: [start, goal], map: map)
        }
        f.onSend = { _ in
            if f.commands.count == 1 { _ = f.driver.updateGoal(Vec2(5.5, 0)) }
            if f.paths.count >= 2 { f.position = f.driver.goal }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertGreaterThanOrEqual(f.paths.count, 2)
        XCTAssertEqual(f.paths.last?.last, f.driver.goal)
        XCTAssertEqual(f.stops, 0)
    }

    func testForwardPursuitStartsGentlyAndDoesNotJumpToCruiseSpeed() async {
        let f = Fixture()
        f.onSend = { _ in if f.commands.count >= 6 { f.position = Vec2(5, 0) } }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 6)
        XCTAssertLessThanOrEqual(f.commands[0].left, 0.1)
        XCTAssertGreaterThan(f.commands[5].left, f.commands[0].left)
        for (previous, next) in zip(f.commands, f.commands.dropFirst()) {
            XCTAssertLessThanOrEqual(next.left - previous.left, 0.05 + 1e-9)
        }
    }

    func testMovingRetargetCannotAcceptBlockedNewTail() async {
        let f = Fixture()
        let clear = Costmap(width: 100, height: 100, resolution: 0.1, origin: Vec2(-1, -1))
        f.planProvider = { start, goal in
            var map = clear
            if !f.commands.isEmpty {
                let replacement = goal + Vec2(0.12, 0)
                _ = f.driver.updateGoal(replacement)
                let cell = map.worldToCell(replacement)
                map.setCost(cell.cx, cell.cy, 1)
            }
            return .init(path: [start, goal], map: map)
        }
        f.onSend = { _ in if f.commands.count == 1 { _ = f.driver.updateGoal(Vec2(5.5, 0)) } }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertEqual(f.paths.count, 1, "An unsafe new tail cannot replace the validated route")
        XCTAssertEqual(f.paths.first?.last, Vec2(5, 0))
    }

    func testDepthRecoveryRestartsForwardRampFromLowSpeed() async {
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: f.commands.count >= 5 && f.stops == 0 ? 0.26 : 0.01) }
        f.onSend = { command in
            if f.stops > 0 {
                XCTAssertLessThanOrEqual(max(command.left, command.right), 0.1)
                f.position = Vec2(5, 0)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.stops, 1)
        XCTAssertEqual(f.commands.count, 6)
        XCTAssertGreaterThan(f.commands[4].left, f.commands[0].left)
    }

    func testRecordedStartupJoinAcceptsPlannerSoftCostsButRejectsHardCells() async {
        // Nov 10 14:57 PST: retain the captured pose, route and the single
        // start-cell cost sufficient to reproduce path_join_blocked.
        let path = [Vec2(-0.0162500038743012, 0.019763301312924142),
            Vec2(0.08374999612569844, 0.11976330131292379),
            Vec2(0.1837499961256981, 0.21976330131292343),
            Vec2(0.2837499961256986, 0.31976330131292396),
            Vec2(0.2837499961256986, 0.4197633013129236),
            Vec2(0.2837499961256986, 0.5197633013129241),
            Vec2(0.3182586567461686, 0.7896189417008743)]
        for cost: UInt8 in [14, 252, 253, 255] {
            let f = Fixture()
            f.position = Vec2(-0.06614302098751068, -0.03034256584942341)
            var map = Costmap(width: 120, height: 120, resolution: 0.1,
                origin: Vec2(-6.066250003874302, -6.030236698687077))
            map.setCost(60, 59, cost)
            _ = f.driver.updateGoal(path.last!)
            f.planProvider = { [map] _, _ in .init(path: path, map: map) }
            f.onSend = { _ in f.position = path.last! }
            let result = await f.driver.run()
            let blocked = cost >= Costmap.lethalThreshold
            XCTAssertEqual(result, blocked ? .failed(.noPath) : .arrived, "cost \(cost)")
            XCTAssertEqual(f.commands.count, blocked ? 0 : 1, "cost \(cost)")
        }
    }

    func testPlannerCostJoinRetainsHardClearanceAndConservativeDefault() {
        var map = Costmap(width: 10, height: 10, resolution: 0.1)
        let start = Vec2(0.25, 0.299)
        let end = Vec2(0.55, 0.299)
        map.setCost(3, 3, 253)
        XCTAssertNil(FollowPursuitPlan(path: [end], map: map).joined(from: start),
            "The two-centimetre margin still intersects a hard obstacle beside the segment")
        map.setCost(3, 3, 14)
        XCTAssertNotNil(FollowPursuitPlan(path: [end], map: map).joined(from: start))
        XCTAssertFalse(map.isSegmentClear(from: start, to: end, margin: 0.02),
            "Other clearance callers retain the strict soft-inflation policy")
        XCTAssertNil(FollowPursuitPlan(path: [end], map: map).joined(from: Vec2(0.01, 0.299)),
            "The margin cannot extend outside the captured map")
        map.setCost(3, 2, 255)
        XCTAssertNil(FollowPursuitPlan(path: [Vec2(0.45, 0.45)], map: map).joined(from: Vec2(0.25, 0.25)),
            "A diagonal connector cannot cut a hard obstacle's corner")
    }

    func testMovingGoalDuringEveryPlanCanStartUsingVerifiedLatestGoalTail() async {
        let f = Fixture()
        let map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
        f.planProvider = { start, goal in
            f.time += 0.4 // Device worker latency overlaps a new target update.
            _ = f.driver.updateGoal(goal + Vec2(0.12, 0))
            return .init(path: [start, goal], map: map)
        }
        f.onSend = { _ in
            XCTAssertEqual(f.paths.last?.last, f.driver.goal, "Never send toward the superseded goal")
            XCTAssertLessThan(f.time, 12, "Keep the original startup deadline")
            f.position = f.driver.goal
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.plannedGoals.count, 1)
    }

    func testStartupRetargetCannotCrossObstacleOrExtendAdmissionDeadline() async {
        let f = Fixture()
        f.planProvider = { start, goal in
            var map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
            let updatedGoal = goal + Vec2(0.12, 0)
            let blocked = map.worldToCell(updatedGoal)
            map.setCost(blocked.cx, blocked.cy, Costmap.lethal)
            f.time += 0.4
            _ = f.driver.updateGoal(updatedGoal)
            return .init(path: [start, goal], map: map)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertTrue(f.paths.isEmpty)
        XCTAssertLessThan(f.time, 12.5)
        XCTAssertFalse(f.events.contains("planning_retargeted"))
    }

    func testRetargetPreservesDetourAndRejectsUnverifiedNewTail() {
        var map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
        let original = [Vec2.zero, Vec2(0, 1), Vec2(5, 1), Vec2(5, 0)]
        let goal = Vec2(4.8, 0)
        let plan = FollowPursuitPlan(path: original, map: map)
        XCTAssertEqual(plan.retargeted(to: goal, from: .zero)?.path, Array(original.dropLast()) + [goal])
        let obstacle = map.worldToCell(Vec2(4.9, 0.5))
        map.setCost(obstacle.cx, obstacle.cy, 1)
        XCTAssertNil(FollowPursuitPlan(path: original, map: map).retargeted(to: goal, from: .zero),
            "Soft inflation still blocks a newly verified segment")
        XCTAssertNil(FollowPursuitPlan(path: original).retargeted(to: goal, from: .zero))
        XCTAssertNil(plan.retargeted(to: Vec2(100, 0), from: .zero))
        XCTAssertNil(plan.retargeted(to: Vec2(.nan, 0), from: .zero))
    }

    func testNewGoalDuringAdmissionCannotSendPreviouslyRetargetedRoute() async {
        let f = Fixture()
        let map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
        f.planProvider = { start, goal in
            _ = f.driver.updateGoal(goal + Vec2(0.12, 0))
            return .init(path: [start, goal], map: map)
        }
        var changedDuringAdmission = false
        f.acknowledgement = {
            if !f.paths.isEmpty, !changedDuringAdmission {
                changedDuringAdmission = true
                _ = f.driver.updateGoal(Vec2(4, 0))
            }
            return Date(timeIntervalSince1970: f.time)
        }
        f.onSend = { _ in
            XCTAssertTrue(changedDuringAdmission)
            XCTAssertEqual(f.paths.last?.last, f.driver.goal)
            XCTAssertEqual(f.driver.goal, Vec2(4.12, 0))
            f.position = f.driver.goal
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
    }

    func testAcceptanceDiagnosticsIdentifyActualRejectingGuardWithoutSending() async throws {
        for reason in ["planner_returned_nil", "empty_path", "nonfinite_waypoint", "endpoint_mismatch", "path_join_blocked", "unmapped_pose_shift"] {
            let f = Fixture()
            f.planProvider = { _, goal in
                switch reason {
                case "planner_returned_nil": return nil
                case "empty_path": return .init(path: [])
                case "nonfinite_waypoint": return .init(path: [Vec2(.nan, 0), goal])
                case "endpoint_mismatch": return .init(path: [goal + Vec2(0.2, 0)])
                case "path_join_blocked":
                    var map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
                    map.setCost(15, 10, Costmap.lethalThreshold) // A hard cell blocks the route connector.
                    return .init(path: [goal], map: map)
                default:
                    f.position = Vec2(0.03, 0)
                    return .init(path: [goal])
                }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, reason == "unmapped_pose_shift" ? .failed(.trackingLost) : .failed(.noPath))
            XCTAssertTrue(f.commands.isEmpty)
            let records = f.records.filter { $0.0 == "plan_acceptance_rejected" }
            XCTAssertEqual(records.count, 1)
            let fields = try XCTUnwrap(records.first?.1)
            XCTAssertEqual(fields["acceptance_reason"], .string(reason))
            XCTAssertEqual(fields["captured_goal_xz"], .object(["x": .number(5), "z": .number(0)]))
            XCTAssertEqual(fields["has_sent_motion"], .bool(false))
            XCTAssertNotNil(fields["captured_source"])
            XCTAssertNotNil(fields["current_source"])
        }
    }

    func testSupersededPlanDiagnosticCapturesBothGoalsWithoutFullGeometry() async throws {
        let f = Fixture()
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 1 { _ = f.driver.updateGoal(Vec2(4, 0)) }
            return .init(path: [goal])
        }
        f.onSend = { _ in f.position = Vec2(4, 0) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        let fields = try XCTUnwrap(f.records.first { $0.0 == "planning_superseded" }?.1)
        XCTAssertEqual(fields["acceptance_reason"], .string("goal_revision_superseded"))
        XCTAssertEqual(fields["captured_revision"], .string("0"))
        XCTAssertEqual(fields["latest_revision"], .string("1"))
        XCTAssertEqual(fields["captured_goal_xz"], .object(["x": .number(5), "z": .number(0)]))
        XCTAssertEqual(fields["latest_goal_xz"], .object(["x": .number(4), "z": .number(0)]))
        XCTAssertEqual(fields["waypoints_capture_status"], .string("summary_only"))
        XCTAssertEqual(fields["waypoints_xz"], .null)
        XCTAssertFalse(f.events.contains("plan_acceptance_rejected"))
    }

    func testStartupTimeoutDiagnosticDoesNotInventAPlannerRejection() async throws {
        let f = Fixture()
        f.planProvider = { _, goal in
            _ = f.driver.updateGoal(Vec2(goal.x + 0.1, 0))
            return .init(path: [goal])
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.noPath))
        let fields = try XCTUnwrap(f.records.first { $0.0 == "plan_acceptance_rejected" }?.1)
        XCTAssertEqual(fields["acceptance_reason"], .string("startup_timeout"))
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testFinalSourceRejectionIsCapturedBeforeAnySend() async throws {
        let f = Fixture()
        f.depthProvider = { _ in
            f.generation += 1
            return f.observation(age: 0.01)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertTrue(f.commands.isEmpty)
        let fields = try XCTUnwrap(f.records.first { $0.0 == "plan_acceptance_rejected" }?.1)
        XCTAssertEqual(fields["acceptance_reason"], .string("source_source_generation_changed"))
        XCTAssertEqual(fields["expected_ar_generation"], .string("1"))
        guard case .object(let source)? = fields["current_source"] else { return XCTFail("Missing source") }
        XCTAssertEqual(source["ar_generation"], .string("2"))
    }

    func testMovingTargetCannotDiscardBlockedStartAtUnchangedRoverCell() async {
        let f = Fixture()
        var map = Costmap(width: 40, height: 40, resolution: 0.1, origin: Vec2(-1, -1))
        let start = map.worldToCell(.zero)
        map.setCost(start.cx, start.cy, Costmap.lethal)
        let blockedMap = map
        f.planProvider = { _, goal in
            // Recorded run: every 120–133 ms result lost its goal revision,
            // although the rover remained in the same blocked start cell.
            f.time += 0.13
            _ = f.driver.updateGoal(Vec2(goal.x + 0.1, goal.y))
            return .init(path: [], map: blockedMap, rejection: .startBlocked,
                diagnostics: ["raw_start_cost": "255", "local_floor_rejection": "inconsistent_surface"])
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertEqual(f.driver.failureCause, .pursuitStartBlocked)
        XCTAssertEqual(f.plannedGoals.count, 1, "A goal update cannot change the unchanged starting cell")
        XCTAssertTrue(f.events.contains("planning_rejected"))
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testSupersededGoalBlockCannotRejectNewGoal() async {
        let f = Fixture()
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 1 {
                _ = f.driver.updateGoal(Vec2(4, 0))
                return .init(path: [], rejection: .goalBlocked)
            }
            return .init(path: [goal])
        }
        f.onSend = { _ in f.position = Vec2(4, 0) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertNil(f.driver.failureCause)
        XCTAssertEqual(f.commands.count, 1)
    }

    func testSupersededStartBlockCannotRejectRoverInDifferentCell() async {
        let f = Fixture()
        var map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
        let start = map.worldToCell(.zero)
        map.setCost(start.cx, start.cy, Costmap.lethal)
        let blockedMap = map
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 1 {
                f.position = Vec2(0.3, 0)
                _ = f.driver.updateGoal(Vec2(4, 0))
                return .init(path: [], map: blockedMap, rejection: .startBlocked)
            }
            return .init(path: [goal])
        }
        f.onSend = { _ in f.position = Vec2(4, 0) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertNil(f.driver.failureCause)
    }

    func testBlockedPlanningFailureReachesControllerWithoutSendingMotion() async {
        let f = ControllerFixture()
        f.planningFailure = .startBlocked
        let result = await f.run()
        XCTAssertEqual(result.result, .failed(.noPath))
        XCTAssertEqual(result.context.failureCause, .pursuitStartBlocked)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testCopiedLocalMeshTransformsOffActorAndPreservesStride() async {
        var values: [Float] = [1, 2, 3, 999, 4, 5, 6, 999]
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4(10, 20, 30, 1)
        let captured = CostmapBuilder.LocalMeshSnapshot(bytes: values.withUnsafeBytes { Data($0) },
            stride: 16, count: 2, transform: transform)
        values[0] = 100
        let points = await Task.detached {
            XCTAssertFalse(Thread.isMainThread)
            return captured.worldVertices()
        }.value
        XCTAssertEqual(points, [SIMD3<Float>(11, 22, 33), SIMD3<Float>(14, 25, 36)])
        let truncated = CostmapBuilder.LocalMeshSnapshot(bytes: Data(), stride: 16, count: 2, transform: transform)
        XCTAssertTrue(truncated.worldVertices().isEmpty)
    }

    func testDepthPublicationAndCancellationWakePendingControlWait() async {
        for cancel in [false, true] {
            let f = Fixture()
            f.notifications = true
            let gate = FollowDiagnosticSuspension()
            f.waitHandler = { _ in
                if f.commands.count == 1 { await gate.suspend() }
                else { f.time += 0.02; await Task.yield() }
            }
            f.onSend = { _ in if f.commands.count == 2 { f.allowed = false } }
            let task = Task { await f.driver.run() }
            await wait { gate.entered }
            XCTAssertEqual(f.commands.count, 1)
            if cancel { task.cancel() }
            else { f.time += 0.033; f.sequence += 1; f.driver.depthPublished() }
            let result = await task.value
            gate.release()
            XCTAssertEqual(result, .cancelled)
            XCTAssertEqual(f.commands.count, cancel ? 1 : 2)
        }
    }

    func testDepthPublishedDuringSendIsNotLostBeforeWaitRegistration() async {
        let f = Fixture()
        f.notifications = true
        f.waitHandler = { _ in
            XCTAssertTrue(f.commands.isEmpty, "An already-published update must bypass the post-send wait")
            f.time += 0.02
            await Task.yield()
        }
        f.onSend = { _ in
            if f.commands.count == 1 { f.driver.depthPublished() }
            else { f.allowed = false }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .cancelled)
        XCTAssertEqual(f.commands.count, 2)
    }

    func testNoPublicationStillWakesAtDepthDeadlineAndChecksSafety() async {
        let f = Fixture()
        f.notifications = true
        f.depthProvider = { _ in f.observation(age: 0.19) }
        let gate = FollowDiagnosticSuspension()
        var waitSeconds = 0.0
        f.waitHandler = { duration in
            if f.commands.count == 1 {
                waitSeconds = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                await gate.suspend()
            } else { f.time += 0.02; await Task.yield() }
        }
        let task = Task { await f.driver.run() }
        await wait { gate.entered }
        XCTAssertGreaterThan(waitSeconds, 0)
        XCTAssertLessThanOrEqual(waitSeconds, 0.061)
        f.time += waitSeconds
        f.clearance = 0.1
        gate.release()
        let result = await task.value
        XCTAssertEqual(result, .failed(.obstacle))
        XCTAssertEqual(f.commands.count, 1)
    }

    func testRecoveryRequiresSeveralPostStopDepthCapturesBeforeResuming() async {
        let f = Fixture()
        var recovering = false
        var captureTimes: Set<Double> = []
        f.depthProvider = { _ in
            if f.commands.count == 1, !recovering {
                recovering = true
                return f.observation(age: 0.3)
            }
            if recovering { captureTimes.insert(f.time - 0.01) }
            return f.observation(age: 0.01)
        }
        f.onSend = { _ in
            if f.commands.count == 2 {
                XCTAssertGreaterThanOrEqual(captureTimes.count, 3)
                XCTAssertGreaterThanOrEqual(captureTimes.max()! - captureTimes.min()!, 0.06)
                XCTAssertEqual(f.stops, 1)
                f.position = Vec2(5, 0)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertTrue(f.events.contains("depth_stability_wait"))
    }

    func testRecordedStopLatencyDoesNotConsumePostStopDepthRecoveryWindow() async {
        let f = Fixture()
        var recoveryStarted = 0.0
        var delivered = 0
        // Nov 6 trace: 135 ms stop, pre-stop depth at 328 ms, then
        // qualifying captures at 446 and 531 ms. A third at 614 ms must
        // still be eligible: only 479 ms have elapsed since stopping.
        let deliveries: [(at: Double, age: Double)] = [
            (0.328, 0.211), (0.446, 0.162), (0.531, 0.163), (0.614, 0.163)
        ]
        f.depthProvider = { _ in
            if f.commands.isEmpty { return f.observation(age: 0.01) }
            return f.observation(age: delivered == 0 ? 0.219 : deliveries[delivered - 1].age)
        }
        f.onStop = { recoveryStarted = f.time; f.time += 0.135 }
        f.waitHandler = { duration in
            if f.stops == 1, delivered < deliveries.count {
                f.time = recoveryStarted + deliveries[delivered].at
                delivered += 1
            } else {
                f.time += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
            }
            f.sequence += 1
            await Task.yield()
        }
        f.onSend = { _ in
            f.sendAdmissionDuration = 0.039
            if f.commands.count == 2 {
                XCTAssertEqual(delivered, 4, "Require all three qualifying post-stop captures")
                XCTAssertEqual(f.stops, 1)
                f.position = Vec2(5, 0)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertNil(f.driver.failureCause)
    }

    func testPersistentDepthOutageIsBoundedFromConfirmedStop() async {
        let f = Fixture()
        var stoppedAt = 0.0
        f.depthProvider = { _ in f.observation(age: f.commands.isEmpty ? 0.01 : 0.3) }
        f.onStop = { f.time += 0.135; stoppedAt = f.time }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(f.driver.failureCause, .pursuitDepthRefreshTimeout)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.stops, 1)
        XCTAssertGreaterThanOrEqual(f.time - stoppedAt, 0.6 - 1e-6)
        XCTAssertLessThan(f.time - stoppedAt, 0.63)
    }

    func testUnsentExpiryDuringRecoveryCannotRenewConfirmedStopDeadline() async {
        let f = Fixture()
        var firstStopAt = 0.0
        f.depthProvider = { _ in
            f.observation(age: (f.commands.count == 1 && f.stops == 0) || f.stops == 2 ? 0.3 : 0.01)
        }
        f.onStop = {
            f.time += 0.135
            if f.stops == 1 { firstStopAt = f.time }
        }
        f.onSend = { _ in
            if f.commands.count == 2 {
                f.time = firstStopAt + 0.59
                throw FollowPursuitSendFailure(reason: .depthUnavailable(.staleRawDepth),
                    definitelyUnsent: true, authorizationExpired: true)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(f.commands.count, 2, "One successful send and one rejected admission; no retry send")
        XCTAssertEqual(f.stops, 2)
        XCTAssertLessThan(f.time - firstStopAt, 0.75, "Second stop must not renew the original deadline")
    }

    func testPostStopRecoveryCannotExtendOriginalStartupAdmissionDeadline() async {
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: f.stops == 0 ? 0.239 : 0.01) }
        f.onStop = { f.time = 11.95 }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertEqual(f.stops, 1)
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertLessThan(f.time, 12.03)
    }

    func testCautionDoesNotRepeatDriverDepthEvaluationForScaledCommand() async {
        let f = Fixture()
        var reads = 0
        f.depthProvider = { _ in
            reads += 1
            return .init(state: .caution, clearance: 0.49, supportCount: 138, sampleAge: 0.147,
                requiredStoppingDistance: 0.3, motionClass: .forward, speedLimit: 0.12)
        }
        f.onSend = { command in
            XCTAssertLessThanOrEqual(max(abs(command.left), abs(command.right)), 0.12)
            f.position = Vec2(5, 0)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(reads, 1)
    }

    func testSpeedLimitedCommandStillChecksExactDepthAtTransport() async {
        for obstacle in [false, true] {
            let f = ControllerFixture()
            f.caution = true
            if obstacle { f.beforeAttempt = { f.obstacle = true } }
            let result = await f.run()
            XCTAssertEqual(result.result, obstacle ? .failed(.obstacle) : .arrived)
            XCTAssertEqual(f.commands.count, obstacle ? 0 : 1)
            XCTAssertEqual(f.depthReads, 2, "One driver check plus the exact-command transport check")
            if let command = f.commands.first { XCTAssertLessThanOrEqual(max(abs(command.left), abs(command.right)), 0.12) }
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testSuccessfulSendsAloneCannotReplenishRecoveryBudget() async {
        let f = Fixture()
        var outages = 0
        var recovering = false
        f.depthProvider = { _ in
            if !f.commands.isEmpty, !recovering {
                outages += 1
                recovering = true
                return f.observation(age: 0.297)
            }
            return f.observation(age: 0.01)
        }
        f.onSend = { _ in recovering = false; f.position.x += 0.02 }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(f.driver.failureCause, .pursuitRecoveryExhausted)
        XCTAssertEqual(outages, 3)
        XCTAssertFalse(f.events.contains("recovery_budget_replenished"))
    }

    func testSustainedFreshDrivingReplenishesRecoveryAllowance() async {
        let f = Fixture()
        let map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
        f.planProvider = { _, goal in .init(path: [goal], map: map) }
        var outages = 0
        var recovering = false
        f.depthProvider = { _ in
            if !f.commands.isEmpty, !recovering,
               outages < 2 || (outages == 2 && f.events.contains("recovery_budget_replenished")) {
                outages += 1
                recovering = true
                return f.observation(age: 0.297)
            }
            return f.observation(age: 0.01)
        }
        f.onSend = { _ in
            recovering = false
            f.position.x += 0.02
            if outages == 3 {
                f.position = Vec2(5, 0)
            } else if f.time > 14 { f.allowed = false }
        }
        let result = await f.driver.run()
        XCTAssertEqual(outages, 3)
        XCTAssertTrue(f.events.contains("recovery_budget_replenished"))
        XCTAssertEqual(result, .arrived)
    }

    func testCancelledMovingPredecessorCannotReuseAdmissionStopConfirmation() async {
        let f = ControllerFixture()
        f.autoAdvance = false
        f.arrival = .zero
        let tickGate = FollowDiagnosticSuspension()
        let stopGate = FollowDiagnosticSuspension()
        f.sleepHandler = { _ in
            if f.commands.count == 1 { await tickGate.suspend() }
            else { await Task.yield() }
        }
        f.stopHandler = { if f.stops == 2 { await stopGate.suspend() } }
        let task = Task { await f.run() }
        await wait { tickGate.entered }
        f.controller.cancel()
        tickGate.release()
        let result = await task.value
        await wait { stopGate.entered }
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertNotEqual(result.stopOutcome, .confirmed, "The initial admission stop predates the moving command")
        XCTAssertEqual(result.commandReceipt?.attempts, 1)
        XCTAssertEqual(result.commandReceipt?.acknowledged, true)
        stopGate.release()
        try? await f.controller.stopAndConfirm()
    }

    func testEpisodeExpiryStillConfirmsStopForTheCurrentOwner() async {
        let f = ControllerFixture()
        f.arrival = .zero
        let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
            deadline: 10.08, now: { f.clock.value }, canContinue: { true })
        let result = await FollowRecoveryScope.$authorization.withValue(authorization) { await f.run() }
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertGreaterThanOrEqual(f.stops, 2)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testFailedInLoopStopLatchesFailureAndCannotResumeReplacementPlan() async {
        let f = ControllerFixture()
        f.autoAdvance = false
        f.holdPlanNumber = 2
        f.failStopNumber = 2
        f.afterSend = {
            if f.commands.count == 1 {
                f.clock.value += 0.5
                XCTAssertTrue(f.controller.updateFollowGoal(Vec2(0, 5)))
            }
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .failed)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertFalse(f.controller.updateFollowGoal(Vec2(1, 1)))
        f.planGate.release()
    }

    func testRealControllerStopsAtOldGoalWhileReplacementPlanIsHeld() async {
        let f = ControllerFixture()
        f.autoAdvance = false
        f.holdPlanNumber = 2
        let stopped = expectation(description: "in-loop stop sent without joining the pursuit task")
        f.onStop = { if f.stops == 2 { stopped.fulfill() } }
        f.afterSend = {
            if f.commands.count == 1 {
                f.arrival = Vec2(0, 5)
                f.clock.value += 0.5
                XCTAssertTrue(f.controller.updateFollowGoal(f.arrival))
            }
        }
        let task = Task { await f.run() }
        await fulfillment(of: [stopped], timeout: 2)
        await wait { f.planGate.entered }
        XCTAssertEqual(f.commands.count, 1)
        f.planGate.release()
        let result = await task.value
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
    }

    func testRetargetCannotHideAnEnteredFirstSendFailure() async {
        let f = ControllerFixture()
        f.failAfterAttempt = true
        f.afterAttempt = {
            f.freshAt = .infinity
            XCTAssertTrue(f.controller.updateFollowGoal(Vec2(0, 5)))
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertLessThan(f.clock.value, 10.30, "No stopped-depth refresh is allowed after uncertain motor admission")
    }

    func testProvenZeroAttemptRetargetMayContinueWithoutInventingMotion() async {
        let f = ControllerFixture()
        f.beforeAttempt = {
            f.beforeAttempt = nil
            f.arrival = Vec2(0, 5)
            XCTAssertTrue(f.controller.updateFollowGoal(f.arrival))
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.stops, 2, "Only the initial/final stops are required for a proven unsent replacement")
    }

    func testMovingReplanJoinsForwardPathAfterRoundingCorner() async {
        let f = Fixture()
        let destination = Vec2(1, 1)
        XCTAssertTrue(f.driver.updateGoal(destination))
        var map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-2, -2))
        let cell = map.worldToCell(Vec2(0.5, 0.3))
        map.setCost(cell.cx, cell.cy, 255)
        let capturedMap = map
        let route = [Vec2.zero, Vec2(1, 0), destination]
        f.planProvider = { _, _ in
            if f.plannedGoals.count == 2 { f.position = Vec2(1, 0.6) }
            return .init(path: route, map: capturedMap)
        }
        f.onSend = { _ in
            if f.commands.count == 1 { f.time += 1.01 }
            if f.paths.count == 2 { f.position = destination }
        }
        XCTAssertFalse(map.isSegmentClear(from: Vec2(1, 0.6), to: .zero, margin: 0.02))
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.paths.count, 2)
        XCTAssertEqual(f.paths.last?.first, destination)
    }

    func testRealControllerUsesAsyncPlanningAndStoppedDepthRefresh() async {
        let f = ControllerFixture()
        f.freshAt = 10.15
        let result = await f.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertGreaterThanOrEqual(f.firstSendTime ?? 0, 10.15)
        XCTAssertGreaterThan(f.depthReads, 1)
        XCTAssertGreaterThanOrEqual(f.stops, 2)
    }

    func testMovingGoalUpdatesThrottlePlanningWithoutBlockingDriving() async {
        let f = Fixture()
        var planningTimes: [Double] = []
        f.planProvider = { _, goal in
            planningTimes.append(f.time)
            return .init(path: [goal])
        }
        f.onSend = { _ in
            _ = f.driver.updateGoal(Vec2(5 + Double(f.commands.count) * 0.1, 0))
            if f.time >= 11.2 { f.allowed = false }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .cancelled)
        XCTAssertGreaterThan(planningTimes.count, 1)
        for (previous, current) in zip(planningTimes, planningTimes.dropFirst()) {
            XCTAssertGreaterThanOrEqual(current - previous, 0.5)
        }
        XCTAssertGreaterThan(f.commands.count, planningTimes.count)
    }

    func testTransportExpiryStopsBeforeRebuildingFreshCommand() async {
        let f = ControllerFixture()
        var attempts = 0
        var stopsAtExpiry = 0
        f.beforeAttempt = {
            attempts += 1
            if attempts == 1 {
                stopsAtExpiry = f.stops
                f.clock.value += 0.30
            } else {
                XCTAssertGreaterThan(f.stops, stopsAtExpiry)
            }
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(attempts, 2)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testExpiryRecoveryCannotContinueAfterFailedStopOrNewObstacle() async {
        for stopFails in [true, false] {
            let f = ControllerFixture()
            f.beforeAttempt = {
                f.beforeAttempt = nil
                f.clock.value += 0.30
            }
            if stopFails { f.failStopNumber = 2 }
            else { f.onStop = { if f.stops == 2 { f.obstacle = true } } }
            let result = await f.run()
            XCTAssertEqual(result.result, .failed(stopFails ? .commandFailed : .obstacle))
            XCTAssertTrue(f.commands.isEmpty)
        }
    }

    func testExpiryRecoveryRequiresANewFrameWithinFixedWindow() async {
        let f = ControllerFixture()
        f.beforeAttempt = {
            f.beforeAttempt = nil
            f.clock.value += 0.30
            f.sleepHandler = { _ in f.clock.value += 0.02; await Task.yield() }
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertLessThan(f.clock.value, 11.0)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testExpiryRecoveryAfterMovingStopsBeforeNextNonzeroCommand() async {
        let f = ControllerFixture()
        f.arrival = .zero
        var expired = false
        var stopCount = 0
        f.beforeAttempt = {
            if f.commands.count == 1, !expired {
                expired = true
                stopCount = f.stops
                f.clock.value += 0.30
            } else if expired {
                XCTAssertGreaterThan(f.stops, stopCount)
                f.arrival = Vec2(5, 0)
            }
        }
        let result = await f.run()
        XCTAssertTrue(expired)
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testRealControllerRejectsDepthExpiryAndNewObstacleAtTransport() async {
        for expired in [false, true] {
            let f = ControllerFixture()
            f.beforeAttempt = {
                if expired { f.clock.value += 0.30 } else { f.obstacle = true }
            }
            let result = await f.run()
            XCTAssertEqual(result.result, expired ? .failed(.depthUnavailable(.staleRawDepth)) : .failed(.obstacle))
            XCTAssertTrue(f.commands.isEmpty)
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testRealControllerCoalescesStartupGoalAndFencesCancelledWorker() async {
        for cancel in [false, true] {
            let f = ControllerFixture()
            f.autoAdvance = false
            f.holdFirstPlan = true
            let task = Task { await f.run() }
            await wait { f.planGate.entered }
            XCTAssertTrue(f.commands.isEmpty)
            if cancel {
                f.controller.cancel()
            } else {
                f.arrival = Vec2(0, 5)
                XCTAssertTrue(f.controller.updateFollowGoal(f.arrival))
                f.clock.value += 0.5
            }
            f.planGate.release()
            let result = await task.value
            if cancel {
                XCTAssertEqual(result.result, .cancelled)
                XCTAssertTrue(f.commands.isEmpty)
            } else {
                XCTAssertEqual(result.result, .arrived)
                XCTAssertEqual(f.plannedGoals, [Vec2(5, 0), Vec2(0, 5)])
                XCTAssertEqual(f.commands.count, 1)
                XCTAssertGreaterThan(f.commands[0].right, f.commands[0].left)
            }
            XCTAssertFalse(f.controller.updateFollowGoal(Vec2(4, 4)))
        }
    }

    func testSnapshotWorkerLeavesControlActorResponsiveAndKeepsCapturedGeometry() async throws {
        let entered = expectation(description: "worker entered")
        let gate = DispatchSemaphore(value: 0)
        defer { gate.signal() }
        var vertices: [SIMD3<Float>] = [.zero, SIMD3(1, 0, 1)]
        let snapshot = CostmapBuilder.MeshSnapshot(worldVertices: vertices)
        let task = Task {
            await CostmapBuilder.planCaptured(snapshot, start: .zero, goal: Vec2(1, 0)) { mesh, _, goal in
                XCTAssertFalse(Thread.isMainThread)
                entered.fulfill()
                _ = gate.wait(timeout: .now() + 3)
                XCTAssertEqual(mesh.worldVertices.first, .zero)
                return .init(path: [goal])
            }
        }
        await fulfillment(of: [entered], timeout: 2)
        vertices[0] = SIMD3(99, 99, 99)
        XCTAssertEqual(vertices[0].x, 99, "Control work runs while the worker is held")
        gate.signal()
        let result = await task.value
        XCTAssertEqual(result?.path, [Vec2(1, 0)])
        let actual = await CostmapBuilder.planCaptured(snapshot, start: .zero, goal: Vec2(1, 0))
        XCTAssertNotNil(actual?.map)
        XCTAssertFalse(actual?.path.isEmpty ?? true)
    }

    func testRecordedStaleStartupDepthWaitsStoppedThenSendsFresh() async {
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: f.time < 10.15 ? 0.30918320833006874 : 0.01) }
        f.onSend = { _ in
            XCTAssertGreaterThanOrEqual(f.time, 10.15)
            f.position = f.driver.goal
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.events.filter { $0 == "depth_wait_started" }.count, 1)
        XCTAssertEqual(f.events.filter { $0 == "depth_wait_recovered" }.count, 1)
    }

    func testRepeatedStaleFramesCannotRenewFixedRefreshWindow() async {
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: 0.309) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertLessThanOrEqual(f.time, 10.70)
        XCTAssertEqual(f.events.filter { $0 == "depth_wait_started" }.count, 1)
    }

    func testFreshDepthAtWaitDeadlineDoesNotAuthorizeLateSend() async {
        let f = Fixture()
        var firstRead: Double?
        f.depthProvider = { _ in
            if firstRead == nil { firstRead = f.time }
            return f.observation(age: f.time >= firstRead! + 0.6 ? 0 : 0.309)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testControllerRecoversMovingStaleDepthWithoutTerminalFailureEvidence() async {
        let f = ControllerFixture()
        f.arrival = .zero
        f.afterSend = {
            if f.commands.count == 1 {
                f.freshAt = f.clock.value + 0.3
                f.arrival = Vec2(5, 0)
            }
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertNil(result.failure)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertGreaterThanOrEqual(f.stops, 3)
    }

    func testRawDepthRefreshCannotResumeAfterSourceResetCancellationOrObstacle() async {
        for fault in ["source", "cancel", "obstacle"] {
            let f = Fixture()
            f.depthProvider = { _ in f.observation(age: f.commands.isEmpty ? 0.01 : 0.254) }
            f.onStop = {
                switch fault {
                case "source": f.generation += 1
                case "cancel": f.allowed = false
                default: f.clearance = 0.1
                }
            }
            let result = await f.driver.run()
            XCTAssertEqual(f.commands.count, 1)
            XCTAssertEqual(result, fault == "source" ? .failed(.trackingLost) :
                (fault == "cancel" ? .cancelled : .failed(.obstacle)))
        }
    }

    func testMovingRawDepthStalenessStopsAndRecoversWithFreshMargin() async {
        let f = Fixture()
        var staleUntil = Double.infinity
        f.depthProvider = { _ in
            f.observation(age: f.commands.count == 1 && f.time < staleUntil ? 0.254045542 : 0.01)
        }
        f.onSend = { _ in
            if f.commands.count == 1 { staleUntil = f.time + 0.3 }
            else {
                XCTAssertEqual(f.stops, 1)
                XCTAssertGreaterThanOrEqual(f.time, staleUntil)
                f.position = Vec2(5, 0)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertTrue(f.events.contains("raw_depth_refresh_started"))
        XCTAssertTrue(f.events.contains("depth_wait_recovered"))
        XCTAssertNil(f.driver.failureCause)
    }

    func testPostStopDepthStabilitySurvivesIntermittentSendMarginMisses() async {
        for expensiveEvery in [2, 3] {
            let f = Fixture()
            var reads = 0
            var stoppedAt = 0.0
            f.onStop = { stoppedAt = f.time }
            f.depthProvider = { _ in
                if f.commands.isEmpty { return f.observation(age: 0.01) }
                if f.stops == 0 { return f.observation(age: 0.26) }
                reads += 1
                // Same 150 ms delivery age as the capture. Some local
                // evaluations leave too little time to safely admit a send.
                let observation = f.observation(age: 0.15)
                f.time += reads % expensiveEvery == 0 ? 0.075 : 0.005
                return observation
            }
            f.onSend = { _ in
                if f.commands.count == 2 {
                    XCTAssertGreaterThanOrEqual(f.time - stoppedAt, 0.21)
                    XCTAssertLessThan(f.time - stoppedAt, 0.6)
                    XCTAssertNotEqual(reads % expensiveEvery, 0, "Never send on a margin-rejected tick")
                    f.position = Vec2(5, 0)
                }
            }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(f.commands.count, 2)
            XCTAssertEqual(f.stops, 1)
            XCTAssertTrue(f.events.contains("freshness_margin_wait"))
            XCTAssertTrue(f.events.contains("depth_wait_recovered"))
        }
    }

    func testMeasuredAdmissionCostRaisesReserveBeforeAnotherSend() async {
        let f = Fixture()
        var freshAt = 0.0
        f.depthProvider = { _ in
            f.observation(age: f.commands.count == 1 && f.time < freshAt ? 0.19 : 0.01)
        }
        f.onSend = { _ in
            if f.commands.count == 1 {
                f.time += 0.06
                f.sendAdmissionDuration = 0.06
                freshAt = f.time + 0.3
            } else {
                XCTAssertGreaterThanOrEqual(f.time, freshAt)
                XCTAssertEqual(f.stops, 1)
                f.position = Vec2(5, 0)
            }
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertTrue(f.events.contains("freshness_margin_wait"))
    }

    func testStoppedRecoveryDoesNotAgeEveryDepthPublicationByRepeatedAckReads() async {
        let f = Fixture()
        var capture = f.time - 0.01
        f.depthProvider = { _ in f.observation(age: f.time - capture) }
        f.acknowledgement = {
            // A cross-actor acknowledgement read can resume one frame later.
            // With the recorded 149 ms publication age this consumes the margin
            // needed for the observed 47.6 ms transport admission.
            if f.stops > 0 { f.time += 0.06 }
            return Date(timeIntervalSince1970: f.time)
        }
        f.waitHandler = { duration in
            f.time += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
            f.sequence += 1
            capture = f.time - (f.stops > 0 ? 0.149 : (f.commands.isEmpty ? 0.01 : 0.21))
            await Task.yield()
        }
        f.onSend = { _ in
            f.time += 0.0476
            f.sendAdmissionDuration = 0.0476
            XCTAssertLessThan(f.time - capture, DepthSafetyEvaluator.maximumRawDepthAge)
            if f.commands.count == 2 { f.position = Vec2(5, 0) }
        }
        // Force the first moving refresh, then deliver continuously clear depth.
        f.onStop = { capture = f.time - 0.3 }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertEqual(f.stops, 1)
        XCTAssertTrue(f.events.contains("depth_wait_recovered"))
    }

    func testStoppedRecoveryCannotRenewCachedAcknowledgementOrHideCommsLoss() async {
        let f = Fixture()
        var stoppedAck: Date?
        var stoppedReads = 0
        f.depthProvider = { _ in f.observation(age: f.commands.isEmpty ? 0.01 : 0.239) }
        f.onStop = { stoppedAck = Date(timeIntervalSince1970: f.time - 1.9) }
        f.acknowledgement = {
            guard f.stops > 0 else { return Date(timeIntervalSince1970: f.time) }
            stoppedReads += 1
            return stoppedReads == 1 ? stoppedAck : nil
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.commsLost))
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.stops, 1)
        XCTAssertEqual(stoppedReads, 2)
        XCTAssertLessThan(f.time, 10.6)
        XCTAssertFalse(f.events.contains("depth_wait_recovered"))
    }

    func testStartupGoalChurnIsThrottledAndNeverSendsObsoletePlan() async {
        let f = Fixture()
        var starts: [Double] = []
        f.planProvider = { _, goal in
            starts.append(f.time)
            _ = f.driver.updateGoal(Vec2(goal.x + 0.1, 0))
            return .init(path: [goal])
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertGreaterThan(starts.count, 1)
        for (a, b) in zip(starts, starts.dropFirst()) { XCTAssertGreaterThanOrEqual(b - a, 0.5) }
        XCTAssertLessThanOrEqual(starts.count, 4)
    }

    func testRecordedPublishedDepthAgesCanStartPursuit() async {
        for age in [0.15713395868, 0.186346167, 0.190] {
            let f = Fixture()
            f.depthProvider = { _ in
                f.time += 0.004 // Recorded motion-depth evaluation cost.
                return f.observation(age: age)
            }
            f.onSend = { _ in f.position = Vec2(5, 0) }
            let result = await f.driver.run()
            XCTAssertEqual(result, .arrived, "Recorded age \(age) should not require a 100 ms reserve")
            XCTAssertEqual(f.commands.count, 1)
            XCTAssertEqual(f.stops, 0)
        }
    }

    func testInsufficientFreshnessMarginWaitsStoppedAndTimesOut() async {
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: 0.239) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertEqual(f.stops, 1)
        XCTAssertEqual(f.driver.failureCause, .pursuitDepthRefreshTimeout)
        XCTAssertLessThan(f.time, 10.7)
        XCTAssertFalse(f.events.contains("depth_wait_recovered"))
    }

    func testHardDepthFaultAndPersistentPostMovementStalenessFailSafely() async {
        for failure in [DepthSafetyUnavailableReason.invalidCalibration, .malformedDepth, .blindSweptVolume] {
            let f = Fixture()
            f.depthProvider = { _ in .unavailable(failure, sampleAge: 0.01, motionClass: .forward) }
            let result = await f.driver.run()
            XCTAssertEqual(result, .failed(.depthUnavailable(failure)))
            XCTAssertTrue(f.commands.isEmpty)
            XCTAssertFalse(f.events.contains("depth_wait_started"))
        }
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: f.commands.isEmpty ? 0.01 : 0.309) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.stops, 1)
        XCTAssertEqual(f.driver.failureCause, .pursuitDepthRefreshTimeout)
        XCTAssertFalse(f.events.contains("depth_wait_started"))
    }

    func testCancellationSourceResetAndPlanningTimeoutCannotPublishOrSend() async {
        for kind in ["cancel", "generation", "timeout", "wrong_goal"] {
            let f = Fixture()
            f.planProvider = { _, goal in
                switch kind {
                case "cancel": f.allowed = false
                case "generation": f.generation = 2
                case "timeout": f.time += 1
                default: return .init(path: [Vec2(99, 99)])
                }
                return .init(path: [goal])
            }
            let result = await f.driver.run()
            XCTAssertNotEqual(result, .arrived, kind)
            XCTAssertTrue(f.commands.isEmpty, kind)
            XCTAssertTrue(f.paths.isEmpty, kind)
        }
    }

    func testObsoleteStartupPlanCannotSendTowardReplacedTarget() async {
        let f = Fixture()
        let gate = FollowDiagnosticSuspension()
        f.autoAdvance = false
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 1 { await gate.suspend() }
            return .init(path: [goal])
        }
        f.onSend = { _ in f.position = f.driver.goal }
        let task = Task { await f.driver.run() }
        await wait { gate.entered }
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertTrue(f.driver.updateGoal(Vec2(0, 5)))
        f.time += 0.5
        gate.release()
        let result = await task.value
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.paths, [[Vec2(0, 5)]])
        XCTAssertEqual(f.plannedGoals, [Vec2(5, 0), Vec2(0, 5)])
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertGreaterThan(f.commands[0].right, f.commands[0].left)
    }

    func testPeriodicWorkerDoesNotBlockDepthSafetyAndLateCompletionCannotPublish() async {
        let f = Fixture()
        let gate = FollowDiagnosticSuspension()
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 2 { f.autoAdvance = false; await gate.suspend() }
            return .init(path: [goal])
        }
        let task = Task { await f.driver.run() }
        await wait { gate.entered }
        let previousCount = f.commands.count
        await wait { f.commands.count > previousCount }
        f.depthProvider = { _ in .unavailable(.invalidCalibration, sampleAge: 0.01, motionClass: .forward) }
        let result = await task.value
        XCTAssertEqual(result, .failed(.depthUnavailable(.invalidCalibration)))
        gate.release()
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(f.paths.count, 1)
        XCTAssertFalse(f.driver.updateGoal(Vec2(1, 1)))
    }

    func testLiveGoalUpdatesCoalesceAndDoNotResetProgressWatchdog() async {
        let f = Fixture()
        f.onSend = { _ in
            _ = f.driver.updateGoal(Vec2(5, f.commands.count.isMultiple(of: 2) ? 0.1 : -0.1))
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertLessThan(f.time, 13)
        XCTAssertGreaterThan(f.commands.count, 5)
        XCTAssertEqual(f.stops, 0)
    }

    func testSafetyIsRecheckedAfterAcknowledgementAndBeforeFirstSend() async {
        let f = Fixture()
        f.acknowledgement = { f.clearance = 0.2; return Date(timeIntervalSince1970: f.time) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.obstacle))
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testCautionRetainsSpeedLimitWithoutDuplicateDriverRead() async {
        let f = Fixture()
        var reads = 0
        f.depthProvider = { command in
            reads += 1
            return .init(state: .caution, clearance: 0.5, supportCount: 3, sampleAge: 0.01,
                requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: 0.12)
        }
        f.onSend = { command in
            XCTAssertLessThanOrEqual(max(abs(command.left), abs(command.right)), 0.12)
            f.position = f.driver.goal
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(reads, 1)
    }

    private func wait(_ condition: () -> Bool, file: StaticString = #filePath, line: UInt = #line) async {
        for _ in 0..<4000 where !condition() { await Task.yield() }
        XCTAssertTrue(condition(), file: file, line: line)
    }

    @MainActor
    private final class ControllerFixture {
        let clock = PursuitTransportClock()
        let planGate = FollowDiagnosticSuspension()
        var holdFirstPlan = false
        var holdPlanNumber: Int?
        var autoAdvance = true
        var plannedGoals: [Vec2] = []
        var commands: [WheelCommand] = []
        var stops = 0
        var failStopNumber: Int?
        var depthReads = 0
        var depthOverride: ((WheelCommand) -> DepthSafetyObservation)?
        var clearanceScopes: [Double?] = []
        var freshAt = 0.0
        var obstacle = false
        var caution = false
        var cautionLimit = 0.12
        var planningFailure: PathPlanningFailure?
        var usePlanningPolicy = false
        var meshPlanningCalls = 0
        var position = Vec2.zero
        var arrival = Vec2(5, 0)
        var sequence: UInt64 = 1
        var firstSendTime: Double?
        var beforeAttempt: (() -> Void)?
        var afterAttempt: (() -> Void)?
        var afterSend: (() -> Void)?
        var onStop: (() -> Void)?
        var failAfterAttempt = false
        var sleepHandler: (@MainActor (Duration) async -> Void)?
        var stopHandler: (@MainActor () async -> Void)?
        var sample: NavigationPoseSample {
            .init(pose: .init(position: position, yaw: 0), frameID: .init(generation: 1, sequence: sequence),
                sourceTimestamp: clock.value - 0.001, trackingQuality: .normal)
        }
        lazy var controller: NavigationController = makeController()
        private func makeController() -> NavigationController {
            let clock = self.clock
            return NavigationController(currentPose: { self.sample.pose }, forwardClearance: { 5 },
                depthSafetyObservation: { command in
                    self.depthReads += 1
                    self.clearanceScopes.append(FollowPlanningScope.minimumObstacleClearance)
                    if let override = self.depthOverride { return override(command) }
                    if self.obstacle { return .init(state: .stop, clearance: 0.1, supportCount: 5, sampleAge: 0.01,
                        requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil) }
                    if clock.value < self.freshAt {
                        return .unavailable(.staleRawDepth, sampleAge: 0.30918320833006874, motionClass: .classify(command))
                    }
                    return .init(state: self.caution ? .caution : .clear, clearance: self.caution ? 0.49 : 5,
                        supportCount: 0, sampleAge: 0.01,
                        requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: self.caution ? self.cautionLimit : nil)
                }, plan: { _, _ in XCTFail("Production-style pursuit must not call the synchronous planner"); return nil },
                lastAckAt: { Date(timeIntervalSince1970: clock.value) }, sendCommand: { _ in XCTFail("Receipt sender required") },
                stopRover: {
                    self.stops += 1; self.onStop?()
                    if self.stops == self.failStopNumber { throw URLError(.timedOut) }
                    await self.stopHandler?()
                }, sleep: { duration in
                    if let handler = self.sleepHandler { await handler(duration); return }
                    if self.autoAdvance { clock.value += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18 }
                    self.sequence += 1
                    await Task.yield()
                }, now: { Date(timeIntervalSince1970: clock.value) }, sendCommandReceipt: { command in
                    self.beforeAttempt?()
                    guard let authorization = FollowTurnBurstTransportScope.authorization,
                          await authorization.authorizeAttempt?() == true, authorization.isAuthorized() else {
                        let expired = FollowTurnBurstTransportScope.authorization.map { clock.value >= $0.deadline } ?? false
                        return .init(receipt: .init(httpStatus: nil, acknowledged: false, acknowledgementUTC: nil,
                            attempts: 0, outcome: expired ? "expired" : "fenced"),
                            failure: expired ? FollowTurnBurstTransportDenial.expired : .fenced)
                    }
                    authorization.didEnterAttempt?(.init(operationID: authorization.operationID, attempt: 1, entryUptime: clock.value))
                    self.commands.append(command)
                    self.firstSendTime = self.firstSendTime ?? clock.value
                    if self.failAfterAttempt {
                        self.afterAttempt?()
                        return .init(receipt: .init(httpStatus: nil, acknowledged: false, acknowledgementUTC: nil,
                            attempts: 1, outcome: "fenced"), failure: FollowTurnBurstTransportDenial.fenced)
                    }
                    self.position = self.arrival
                    self.sequence += 1
                    self.afterSend?()
                    return .init(receipt: .init(httpStatus: 200, acknowledged: true,
                        acknowledgementUTC: Date(timeIntervalSince1970: clock.value), attempts: 1, outcome: "acknowledged"), failure: nil)
                }, poseSample: { self.sample }, sourceNow: { clock.value }, sourceStopSnapshot: { self.sample },
                transportUptime: { clock.value }, pursuitPlan: { _, goal in
                    self.plannedGoals.append(goal)
                    if (self.holdFirstPlan && self.plannedGoals.count == 1) || self.plannedGoals.count == self.holdPlanNumber {
                        await self.planGate.suspend()
                    }
                    if self.usePlanningPolicy {
                        return await FollowPlanningPolicy.plan(mode: FollowPlanningScope.mode, start: self.position, goal: goal) {
                            self.meshPlanningCalls += 1
                            return .init(path: self.planningFailure == nil ? [goal] : [], rejection: self.planningFailure)
                        }
                    }
                    return .init(path: self.planningFailure == nil ? [goal] : [], rejection: self.planningFailure)
                })
        }
        func run() async -> FollowMotionResult {
            await controller.performFollowMotion(.following(Vec2(5, 0), 1.25), context:
                .init(sessionGeneration: 1, requestToken: 1, purpose: .followGoal, phase: "following"))
        }
    }

    @MainActor
    private final class Fixture {
        var time = 10.0
        var generation: UInt64 = 1
        var sequence: UInt64 = 1
        var position = Vec2.zero
        var yaw = 0.0
        var allowed = true
        var autoAdvance = true
        var clearance = 5.0
        var commands: [WheelCommand] = []
        var plannedGoals: [Vec2] = []
        var paths: [[Vec2]] = []
        var events: [String] = []
        var records: [(String, [String: FollowDiagnosticValue])] = []
        var expiries: [Double] = []
        var stops = 0
        var stopError = false
        var planProvider: (@MainActor (Vec2, Vec2) async -> FollowPursuitPlan?)?
        var depthProvider: ((WheelCommand) -> DepthSafetyObservation)?
        var onSend: ((WheelCommand) throws -> Void)?
        var onStop: (() -> Void)?
        var sendAdmissionDuration = 0.0
        var notifications = false
        var waitHandler: ((Duration) async -> Void)?
        var acknowledgement: (@MainActor () async -> Date?)?
        var acknowledgementSnapshot: (@MainActor () -> Date?)?
        var targetSample: (@MainActor () -> FollowTargetSpeedSample?)?
        func observation(age: Double) -> DepthSafetyObservation {
            .init(state: .clear, clearance: 5, supportCount: 0, sampleAge: age,
                requiredStoppingDistance: 0.3, motionClass: .forward, speedLimit: nil)
        }
        lazy var driver: FollowPursuitDriver = makeDriver()
        private func makeDriver() -> FollowPursuitDriver {
            FollowPursuitDriver(goal: Vec2(5, 0), environment: .init(plan: { start, goal in
            self.plannedGoals.append(goal)
            if let provider = self.planProvider { return await provider(start, goal) }
            return .init(path: [goal])
        }, pose: {
            .init(pose: .init(position: self.position, yaw: self.yaw), frameID: .init(generation: self.generation, sequence: self.sequence),
                sourceTimestamp: self.time - 0.001, trackingQuality: .normal)
        }, owned: { self.allowed }, uptime: { self.time }, date: { Date(timeIntervalSince1970: self.time) },
        acknowledgement: {
            if let read = self.acknowledgement { return await read() }
            return Date(timeIntervalSince1970: self.time)
        }, clearance: { self.clearance }, depth: { self.depthProvider?($0) ?? self.observation(age: 0.01) },
        send: { command, expiry in
            XCTAssertLessThan(self.time, expiry)
            self.commands.append(command)
            self.expiries.append(expiry)
            try self.onSend?(command)
        }, stop: {
            self.stops += 1
            if self.stopError { throw URLError(.timedOut) }
            self.onStop?()
        }, sleep: { duration in
            if let handler = self.waitHandler { await handler(duration); return }
            if self.autoAdvance {
                self.time += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
            }
            self.sequence += 1
            await Task.yield()
        }, publishPath: { self.paths.append($0) }, moving: {}, emit: { name, fields in
            self.events.append(name)
            self.records.append((name, fields))
        },
            admissionDuration: { self.sendAdmissionDuration }, depthNotificationsEnabled: notifications,
            acknowledgementSnapshot: acknowledgementSnapshot, targetSample: { self.targetSample?() }))
        }
    }
}

private final class PursuitTransportClock: @unchecked Sendable {
    private let lock = NSLock()
    private var time = 10.0
    var value: Double {
        get { lock.withLock { time } }
        set { lock.withLock { time = newValue } }
    }
}
