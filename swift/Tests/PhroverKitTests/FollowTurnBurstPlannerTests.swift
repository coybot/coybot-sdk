import XCTest
import PhroverKit

final class FollowTurnBurstPlannerTests: XCTestCase {
    func testAstralAndCoybotResidualsUseInclusiveFiveDegreeAlignmentAcceptance() {
        for sign in [-1.0, 1.0] {
            for residual in [0.052565507099431086, 0.08364032971500879, 5 * Double.pi / 180] {
                XCTAssertEqual(plan(target: sign * residual), .arrived)
            }
            guard case .burst(let direction, _) = plan(target: sign * 5.1 * .pi / 180) else {
                return XCTFail("Outside five degrees still requires correction")
            }
            XCTAssertEqual(direction, sign < 0 ? -1 : 1)
        }
        XCTAssertEqual(plan(target: 0.08364032971500879, authorized: false), .unavailable)
        XCTAssertEqual(FollowMeConfiguration().alignmentAngularTolerance, 5 * .pi / 180)
    }

    func testRecordedCoybotUndershootAcceptsSettledResidualDespiteStopOverhead() {
        // iPhone 15 Pro, 2026-10-28 03:35:22Z, alignment operation 14.
        // Immutable bracket endpoints suffice here: the measured rate remains
        // below the reference floor, preserving the exact logged inverse model.
        let seed = initial(2.051134330124257)
        let profile = Planner.Profile(purpose: .alignment)
        let first = sample(1.8138976608582102, 3240, 127361.15422266599, collected: 127361.25510495834)
        let last = sample(1.9642580417755016, 3279, 127361.804265333, collected: 127361.90646229168)
        let measured = Planner.recording(.init(operationID: 1, generation: 7, targetYaw: seed.targetYaw,
            clockDomain: "test_uptime", requestedBudget: 0.08,
            sendEntryUptime: 127361.40614795835, sendResponseUptime: 127361.47170787501,
            stopObligationUptime: 127361.55479579167, stopAcknowledgementUptime: 127361.67355866668,
            samples: [first, last],
            planningEvaluation: sample(first.yaw, 3240, first.sourceTimestamp, collected: 127361.40558191667),
            stoppedEvaluation: sample(last.yaw, 3279, last.sourceTimestamp, collected: 127362.05452745834)),
            in: seed, profile: profile)
        XCTAssertNil(measured.rejection)
        XCTAssertEqual(measured.calibration.responseOverhead, 0.18741070832940748, accuracy: 1e-10)
        for actual in [last.yaw, 1.9674940004092483] {
            XCTAssertEqual(Planner.plan(.init(actualYaw: actual, profile: profile,
                calibration: measured.calibration, sendEntryUptime: 127362.14889920835,
                provisionalProbeIssued: true)), .arrived)
        }
    }

    func testRecordedStopLatencySpikeAllowsOneShrunkenOppositeCorrection() throws {
        // iPhone 15 Pro Max, 2026-10-24 17:54:25–26Z, alignment 14,
        // pulse 6. Minimized to the immutable ingress endpoints plus their
        // separate control reads; these reproduce the logged rate/overhead/cap.
        let profile = Planner.Profile(purpose: .alignment)
        let seed = initial(1.6160948671430133)
        let first = sample(1.517722599984274, 2677, 1482627.672592041,
            collected: 1482627.7102962919)
        let last = sample(2.0431603854449154, 2719, 1482628.372649791,
            collected: 1482628.411234125)
        let captured = Planner.Response(operationID: 1, generation: 7, targetYaw: seed.targetYaw,
            clockDomain: "test_uptime", requestedBudget: 0.08,
            sendEntryUptime: 1482627.7154253335, sendResponseUptime: 1482627.7606583333,
            stopObligationUptime: 1482627.7974909167, stopAcknowledgementUptime: 1482628.1124181252,
            samples: [first, last],
            planningEvaluation: sample(first.yaw, 2677, first.sourceTimestamp, collected: 1482627.7149122918),
            stoppedEvaluation: sample(last.yaw, 2719, last.sourceTimestamp, collected: 1482628.41537775))
        let measured = Planner.recording(captured, in: seed, profile: profile)
        XCTAssertNil(measured.rejection)
        XCTAssertEqual(measured.calibration.responseOverhead, 0.3169927916768938, accuracy: 1e-9)
        let ceiling = try XCTUnwrap(measured.calibration.overshootCeiling)
        XCTAssertEqual(ceiling, 0.0016909030703662143, accuracy: 1e-9)
        let decision = Planner.plan(.init(actualYaw: 2.042520687077125, profile: profile,
            calibration: measured.calibration, sendEntryUptime: 1482628.42, provisionalProbeIssued: true))
        guard case .burst(let direction, let budget) = decision else {
            return XCTFail("A complete confirmed overshoot must permit one smaller corrective probe, got \(decision)")
        }
        XCTAssertEqual(direction, -1)
        XCTAssertGreaterThan(budget, 0)
        XCTAssertLessThanOrEqual(budget, ceiling)
    }

    func testCapturedAlignmentCanRepeatWithinSweptHeadingRangeDespiteSmallReversals() {
        // iPhone trace, 2026-10-23 17:45:53Z. Total yaw variation exceeds the
        // corridor, but every sampled heading remains inside a smaller range.
        let captured: [(yaw: Double, source: Double, collected: Double)] = [
            (1.1949832373110556, 9203.938061666, 9204.022649458333),
            (1.1940817401726649, 9203.971396791, 9204.056034416668),
            (1.190861089928887, 9204.004731958, 9204.088726458334),
            (1.1902719236301442, 9204.038066916, 9204.125574458334),
            (1.1925323921063509, 9204.071401875, 9204.161078791667),
            (1.2148584935181572, 9204.104737125, 9204.191682333334),
            (1.2444113096701694, 9204.138072125, 9204.223500791668),
            (1.2814643372919368, 9204.17140725, 9204.259908291668),
            (1.3214325300794338, 9204.20474225, 9204.292108666667),
            (1.3304701942019193, 9204.238077375, 9204.322926833334),
            (1.3243433123842794, 9204.271412291, 9204.354776458335),
            (1.3276229105685151, 9204.304747541, 9204.388831583334),
            (1.3216905310702693, 9204.338082541, 9204.424146791667),
            (1.3239118201860158, 9204.371417625, 9204.457533958333),
            (1.3269333311310412, 9204.404752541, 9204.492133791668),
        ]
        for direction in [-1.0, 1.0] {
            for offset in [0.0, direction * 1.9] { // Also cross the +/-pi representation seam.
                func yaw(_ value: Double) -> Double { atan2(sin(direction * value + offset), cos(direction * value + offset)) }
                let target = yaw(1.422101571912323)
                let seed = initial(target)
                let profile = Planner.Profile(purpose: .alignment)
                let samples = captured.enumerated().map { i, point in
                    Planner.Sample(yaw: yaw(point.yaw), sequence: UInt64(i + 1), generation: 7,
                        sourceTimestamp: point.source, collectedUptime: point.collected,
                        clockDomain: "test_uptime", healthy: true)
                }
                let response = Planner.Response(operationID: 1, generation: 7, targetYaw: target,
                    clockDomain: "test_uptime", requestedBudget: 0.08,
                    sendEntryUptime: 9204.031909416668, sendResponseUptime: 9204.098584083335,
                    stopObligationUptime: 9204.135600791667, stopAcknowledgementUptime: 9204.192158666667,
                    samples: samples,
                    planningEvaluation: .init(yaw: samples[0].yaw, sequence: 1, generation: 7,
                        sourceTimestamp: samples[0].sourceTimestamp, collectedUptime: 9204.030813708334,
                        clockDomain: "test_uptime", healthy: true),
                    stoppedEvaluation: .init(yaw: samples.last!.yaw, sequence: UInt64(samples.count), generation: 7,
                        sourceTimestamp: samples.last!.sourceTimestamp, collectedUptime: 9204.501202041667,
                        clockDomain: "test_uptime", healthy: true))
                let measured = Planner.recording(response, in: seed, profile: profile)
                XCTAssertNil(measured.rejection)
                XCTAssertEqual(measured.sampledAbsoluteTravel ?? -1, 0.1654912438135796, accuracy: 1e-10)
                XCTAssertEqual(measured.calibration.responseRate, seed.responseRate,
                    "Preserve the conservative rate model; this fix concerns swept heading geometry")
                XCTAssertEqual(Planner.plan(.init(actualYaw: yaw(1.422101571912323 - 0.09794924136092731),
                    profile: profile, calibration: measured.calibration, sendEntryUptime: 9205,
                    provisionalProbeIssued: true)), .burst(direction: Int(direction), budget: 0.08))
            }
        }
    }

    func testOvershootCorrectionRequiresImprovingMeasuredResponseBeforeRepeating() throws {
        let profile = Planner.Profile(purpose: .alignment)
        for sign in [-1.0, 1.0] {
            let seed = initial(sign * 0.1)
            let overshoot = Planner.recording(response(seed, samples: [sample(0), sample(sign * 0.6, 2, 100.8)],
                stop: 0.3), in: seed, profile: profile)
            XCTAssertNil(overshoot.rejection)
            let calibration = overshoot.calibration
            let ceiling = try XCTUnwrap(calibration.overshootCeiling)
            for end in [0.6, 0.7, -0.6] { // zero progress, wrong direction, another overshoot
                let reduced = Planner.recording(response(seed, budget: ceiling,
                    samples: [sample(sign * 0.6, 3, 100.9), sample(sign * end, 4, 101.8)],
                    stop: 0.3, entry: 101), in: calibration, profile: profile)
                XCTAssertNil(reduced.rejection)
                XCTAssertEqual(Planner.plan(.init(actualYaw: sign * end, profile: profile,
                    calibration: reduced.calibration, sendEntryUptime: 102)),
                    .resolutionFailure(.rotationResolutionInsufficient))
            }

            var current = calibration
            var previous = sign * 0.6
            for (index, end) in [0.45, 0.3, 0.14].enumerated() {
                let entry = Double(101 + index)
                let before = Planner.plan(.init(actualYaw: previous, profile: profile,
                    calibration: current, sendEntryUptime: entry))
                guard case .burst(let direction, let budget) = before else { return XCTFail("\(before)") }
                XCTAssertEqual(direction, -Int(sign))
                XCTAssertLessThanOrEqual(budget, ceiling)
                let reduced = Planner.recording(response(seed, budget: budget,
                    samples: [sample(previous, UInt64(3 + index * 2), entry - 0.1),
                        sample(sign * end, UInt64(4 + index * 2), entry + 0.8)],
                    stop: 0.3, entry: entry), in: current, profile: profile)
                XCTAssertNil(reduced.rejection)
                current = reduced.calibration
                previous = sign * end
            }
            XCTAssertEqual(Planner.plan(.init(actualYaw: previous, profile: profile,
                calibration: current, sendEntryUptime: 104)), .arrived)
            XCTAssertGreaterThanOrEqual(current.responseOverhead, calibration.responseOverhead,
                "Correction must not erase observed latency")
        }
    }

    func testRepeatCannotHideWideOutAndBackExcursionOrWrongDirection() {
        let seed = initial(0.8)
        let profile = Planner.Profile(purpose: .alignment)
        for middle in [0.3, -0.3] {
            let measured = Planner.recording(response(seed, samples: [sample(0), sample(middle, 2, 100),
                sample(0.1, 3, 100.4)]), in: seed, profile: profile)
            XCTAssertNil(measured.rejection)
            for error in [-0.1, 0.1] {
                XCTAssertEqual(Planner.plan(.init(actualYaw: seed.targetYaw - error, profile: profile,
                    calibration: measured.calibration, sendEntryUptime: 101)),
                    .resolutionFailure(.rotationResolutionInsufficient))
            }
        }
    }

    func testInheritedSearchGainReducesNewStepProbeWithoutReusingTargetOrSourceHistory() {
        let seed = Planner.Calibration(operationID: 2, generation: 7, targetYaw: .pi / 18,
            clockDomain: "test_uptime", responseRateFloor: 10)
        let decision = Planner.plan(.init(actualYaw: 0, profile: .init(purpose: .scan), calibration: seed, sendEntryUptime: 10))
        guard case .burst(_, let budget) = decision else { return XCTFail() }
        XCTAssertEqual(budget, 0.00523598775598299, accuracy: 1e-12)
        XCTAssertEqual(seed.completedResponses, 0)
        XCTAssertNil(seed.lastSourceSample)
        XCTAssertEqual(seed.targetYaw, .pi / 18)
    }
    func testRecordedSmallResponseCanRepeatDespiteLongStopOverhead() {
        let calibration = initial(0.5230183705041618)
        let profile = Planner.Profile(purpose: .scan)
        let response = Planner.Response(operationID: 1, generation: 7, targetYaw: calibration.targetYaw,
            clockDomain: "test_uptime", requestedBudget: 0.080, sendEntryUptime: 100,
            sendResponseUptime: 100.119902333332, stopObligationUptime: 100.080,
            stopAcknowledgementUptime: 100.234568791666,
            samples: [sample(0, 1, 99.99), sample(0.094628303215284, 2, 100.24),
                      sample(0.21568743294197268, 3, 100.55)])
        let reduced = Planner.recording(response, in: calibration, profile: profile)
        XCTAssertNil(reduced.rejection)
        XCTAssertEqual(reduced.calibration.responseRate, 2.0943951023931953, accuracy: 1e-10)
        XCTAssertEqual(reduced.calibration.responseOverhead, 0.154568791666, accuracy: 1e-10)
        XCTAssertEqual(reduced.calibration.observedPostAckTravel ?? -1, 0.12105912972668875, accuracy: 1e-10)
        let actual = calibration.targetYaw - 0.27021642704802606
        XCTAssertEqual(Planner.plan(.init(actualYaw: actual, profile: profile,
            calibration: reduced.calibration, sendEntryUptime: 101, provisionalProbeIssued: true)),
            .burst(direction: 1, budget: 0.080))
    }
    private typealias Planner = FollowTurnBurstPlanner
    func testLearnedOverheadResolutionBoundaryDoesNotRoundUpOrHideUnrepresentableDeadline() {
        let target = 0.25 + 5 * Double.pi / 180
        let calibration = initial(target)
        let profile = Planner.Profile(purpose: .alignment)
        // Exactly representable timing: .5 rad / .125 s = 4 rad/s,
        // overhead .125 - .0625 = .0625 s. Zero-budget travel is .25 rad.
        let measured = Planner.recording(.init(operationID: 1, generation: 7, targetYaw: target,
            clockDomain: "test_uptime", requestedBudget: 0.0625, sendEntryUptime: 100,
            sendResponseUptime: 100.01, stopObligationUptime: 100.0625, stopAcknowledgementUptime: 100.125,
            samples: [sample(-1), sample(-0.5, 2, 100.5)]), in: calibration, profile: profile)
        XCTAssertNil(measured.rejection)
        for actual in [0.0, 0.004] {
            XCTAssertEqual(Planner.plan(.init(actualYaw: actual, profile: profile,
                calibration: measured.calibration, sendEntryUptime: 101)),
                .resolutionFailure(.rotationResolutionInsufficient))
        }
        let positive = Planner.plan(.init(actualYaw: -0.004, profile: profile,
            calibration: measured.calibration, sendEntryUptime: 101))
        guard case .burst(let direction, let budget) = positive else { return XCTFail() }
        XCTAssertEqual(direction, 1)
        XCTAssertEqual(budget, 0.001, accuracy: 1e-12)
        XCTAssertEqual(Planner.plan(.init(actualYaw: -0.004, profile: profile,
            calibration: measured.calibration, sendEntryUptime: 1e20)),
            .resolutionFailure(.rotationResolutionInsufficient))
    }

    func testLearningUsesCommandStopWindowNotPeakSourceVelocity() {
        let calibration = initial(1)
        let profile = Planner.Profile(purpose: .scan)
        // The same total response sampled at different source cadences must
        // produce the same end-to-end gain, not a peak-velocity time model.
        for middleTime in [99.901, 100.0] {
            let reduced = Planner.recording(response(calibration, samples: [sample(0),
                sample(0.1, 2, middleTime), sample(0.2, 3, 100.4)]), in: calibration, profile: profile)
            XCTAssertNil(reduced.rejection)
            XCTAssertEqual(reduced.calibration.responseRate, 2.22222222222222, accuracy: 1e-10)
        }
        let reversing = Planner.recording(response(calibration, samples: [sample(0),
            sample(0.5, 2, 100), sample(0.2, 3, 100.4)]), in: calibration, profile: profile)
        XCTAssertEqual(reversing.calibration.responseRate, 8.88888888888889, accuracy: 1e-10,
            "Reversal must not cancel travelled angle and underestimate the next response")
    }

    func testIncompleteResponseCannotAuthorizeAnotherProbeAndPurposeTolerancesStayDistinct() {
        let calibration = initial(0.3)
        let unknown = Planner.recording(response(calibration), in: calibration, profile: .init(purpose: .scan))
        XCTAssertEqual(Planner.plan(.init(actualYaw: 0, profile: .init(purpose: .scan),
            calibration: unknown.calibration, sendEntryUptime: 101, provisionalProbeIssued: true)),
            .resolutionFailure(.rotationResolutionInsufficient))
        let measured = Planner.recording(response(calibration, samples: [sample(0), sample(0.2, 2, 100.4)]),
            in: calibration, profile: .init(purpose: .scan))
        XCTAssertEqual(Planner.plan(.init(actualYaw: 0.18, profile: .init(purpose: .scan),
            calibration: measured.calibration, sendEntryUptime: 101)), .arrived)
        guard case .burst(_, let budget) = Planner.plan(.init(actualYaw: 0.18, profile: .init(purpose: .alignment),
            calibration: measured.calibration, sendEntryUptime: 101)) else { return XCTFail("Alignment retains its tighter gate") }
        XCTAssertEqual(budget, 0.080, accuracy: 1e-10, "Observed pulse fits the remaining five-degree corridor")
        let laterMissing = Planner.recording(response(calibration, entry: 101),
            in: measured.calibration, profile: .init(purpose: .alignment))
        XCTAssertEqual(Planner.plan(.init(actualYaw: 0, profile: .init(purpose: .alignment),
            calibration: laterMissing.calibration, sendEntryUptime: 102)),
            .resolutionFailure(.rotationResolutionInsufficient), "Earlier calibration cannot rescue missing current response evidence")
    }
    private func plan(target: Double, actual: Double = 0, purpose: Planner.Purpose = .alignment,
                      entry: Double = 100, authorized: Bool = true) -> Planner.Decision {
        Planner.plan(.init(actualYaw: actual, profile: .init(purpose: purpose),
            calibration: .init(operationID: 1, generation: 7, targetYaw: target, clockDomain: "test_uptime"),
            sendEntryUptime: entry, authorized: authorized))
    }

    func testFirstFivePointOneDegreeProbeUsesWorkedSubMillisecondExcessWithoutFloor() {
        guard case let .burst(direction, budget) = plan(target: 5.1 * .pi / 180) else {
            return XCTFail("Unknown response must permit one provisional probe")
        }
        XCTAssertEqual(direction, 1)
        XCTAssertEqual(budget, 0.0008333333333333334, accuracy: 1e-14)
    }

    func testCircularErrorUsesInclusivePurposeToleranceAndExactPiNegativeDirection() {
        XCTAssertEqual(plan(target: 5 * .pi / 180), .arrived)
        XCTAssertEqual(plan(target: -5 * .pi / 180), .arrived)
        XCTAssertEqual(plan(target: 7 * .pi / 180, purpose: .scan), .arrived)
        XCTAssertEqual(plan(target: .pi), .burst(direction: -1, budget: 0.080))
        XCTAssertEqual(plan(target: -.pi), .burst(direction: -1, budget: 0.080))
        XCTAssertEqual(plan(target: 0.5235987755982988, purpose: .scan), .burst(direction: 1, budget: 0.080))
        XCTAssertEqual(plan(target: -0.5235987755982988, purpose: .scan), .burst(direction: -1, budget: 0.080))
        for (target, actual, sign) in [(-3.09, 3.09, 1), (3.09, -3.09, -1)] {
            guard case let .burst(direction, budget) = plan(target: target, actual: actual) else { return XCTFail() }
            XCTAssertEqual(direction, sign)
            XCTAssertEqual(budget, 0.00760068840959383, accuracy: 1e-14)
        }
    }

    func testInvalidAuthorityAndNonfiniteInputsAreUnavailableNotZeroError() {
        for value in [Double.nan, .infinity, -.infinity] {
            XCTAssertEqual(plan(target: value), .unavailable)
            XCTAssertEqual(plan(target: 0, actual: value), .unavailable)
            XCTAssertEqual(plan(target: 1, entry: value), .unavailable)
        }
        XCTAssertEqual(plan(target: Double.greatestFiniteMagnitude, actual: -Double.greatestFiniteMagnitude), .unavailable)
        XCTAssertEqual(plan(target: 0, authorized: false), .unavailable)
        XCTAssertEqual(plan(target: 1, entry: -1), .unavailable)
    }

    private func initial(_ target: Double = 0.1) -> Planner.Calibration {
        .init(operationID: 1, generation: 7, targetYaw: target, clockDomain: "test_uptime")
    }
    private func response(_ calibration: Planner.Calibration, budget: Double = 0.080,
                          samples: [Planner.Sample] = [], send: Double = 0.010,
                          stop: Double = 0.010, completion: Planner.Completion = .completed,
                          unambiguous: Bool = true, entry: Double = 100) -> Planner.Response {
        .init(operationID: calibration.operationID, generation: calibration.generation,
            targetYaw: calibration.targetYaw, clockDomain: calibration.clockDomain, completion: completion,
            requestedBudget: budget, sendEntryUptime: entry, sendResponseUptime: entry + send,
            stopObligationUptime: entry + 0.080, stopAcknowledgementUptime: entry + 0.080 + stop,
            samples: samples, traversalUnambiguous: unambiguous)
    }

    func testCompletedLatencyIsRetainedWithUnknownYawAndCoast() {
        let calibration = initial()
        let reduced = Planner.recording(response(calibration), in: calibration, profile: .init(purpose: .alignment))
        XCTAssertNil(reduced.rejection)
        XCTAssertEqual(reduced.calibration.completedResponses, 1)
        XCTAssertEqual(reduced.calibration.maximumSendDuration ?? -1, 0.010, accuracy: 1e-12)
        XCTAssertEqual(reduced.calibration.maximumStopDuration ?? -1, 0.010, accuracy: 1e-12)
        XCTAssertNil(reduced.calibration.observedPostAckTravel, "Unknown does not mean measured zero")
        XCTAssertEqual(reduced.calibration.responseOverhead, 0, "Incomplete evidence cannot teach a response model")
        XCTAssertEqual(reduced.calibration.responseRate, 2.0943951023931953, accuracy: 1e-14)
    }

    func testUnmeasuredResponseAndUnrepresentableDeadlineCannotAuthorizeRetry() {
        let calibration = initial()
        let reduced = Planner.recording(response(calibration), in: calibration, profile: .init(purpose: .alignment)).calibration
        XCTAssertEqual(Planner.plan(.init(actualYaw: 0, profile: .init(purpose: .alignment), calibration: reduced,
            sendEntryUptime: 100)), .resolutionFailure(.rotationResolutionInsufficient))
        XCTAssertEqual(Planner.plan(.init(actualYaw: 0, profile: .init(purpose: .alignment), calibration: calibration,
            sendEntryUptime: 1e20)), .resolutionFailure(.rotationResolutionInsufficient))
        XCTAssertEqual(Planner.plan(.init(actualYaw: 0, profile: .init(purpose: .alignment), calibration: calibration,
            sendEntryUptime: 100, provisionalProbeIssued: true)), .resolutionFailure(.rotationResolutionInsufficient))
        XCTAssertEqual(Planner.plan(.init(actualYaw: calibration.targetYaw, profile: .init(purpose: .alignment),
            calibration: reduced, sendEntryUptime: 100)), .arrived)
    }

    private func sample(_ yaw: Double, _ sequence: UInt64? = 1, _ time: Double? = 99.9,
                        collected: Double? = nil, generation: UInt64? = 7,
                        domain: String? = "test_uptime", healthy: Bool = true) -> Planner.Sample {
        .init(yaw: yaw, sequence: sequence, generation: generation, sourceTimestamp: time,
            collectedUptime: collected ?? time ?? 99.9, clockDomain: domain, healthy: healthy)
    }

    func testInvalidResponseBracketsNeverLearnOrEraseOperationEvidence() {
        let calibration = initial(1)
        let profile = Planner.Profile(purpose: .alignment)
        let first = sample(0)
        let last = sample(0.1, 2, 100.4)
        var invalid: [Planner.Response] = []
        for completion: Planner.Completion in [.cancelled, .failed, .replaced, .ambiguous] {
            invalid.append(response(calibration, samples: [first, last], completion: completion))
        }
        for bad in [sample(.nan), sample(.infinity), sample(0, nil), sample(0, generation: nil),
                    sample(0, generation: 8), sample(0, domain: nil), sample(0, domain: "private_clock"),
                    sample(0, healthy: false), sample(0, 1, nil), sample(0, 1, .nan),
                    sample(0, collected: .infinity), sample(0, collected: 99.8), sample(0, collected: 100.401)] {
            invalid.append(response(calibration, samples: [bad, last]))
        }
        invalid.append(response(calibration, samples: [first, sample(0.1, 1, 100.4)]))
        invalid.append(response(calibration, samples: [first, sample(0.1, 2, 99.9)]))
        invalid.append(response(calibration, samples: [last, first]))
        invalid.append(response(calibration, samples: [first, last], unambiguous: false))
        invalid.append(response(calibration, budget: 0))
        invalid.append(response(calibration, budget: .nan))
        invalid.append(response(calibration, send: -0.01))
        invalid.append(response(calibration, stop: .infinity))
        invalid.append(response(initial(2), samples: [first, last]))
        invalid.append(.init(operationID: 2, generation: 7, targetYaw: 1, clockDomain: "test_uptime",
            requestedBudget: 0.08, sendEntryUptime: 100, sendResponseUptime: 100.01,
            stopObligationUptime: 100.08, stopAcknowledgementUptime: 100.09, samples: [first, last]))
        for evidence in invalid {
            let result = Planner.recording(evidence, in: calibration, profile: profile)
            XCTAssertNotNil(result.rejection)
            XCTAssertEqual(result.calibration, calibration, "Rejected evidence must not learn latency, rate, or coast")
        }
        let inclusive = Planner.recording(response(calibration, samples: [sample(0, 1, 99.5, collected: 100), last]),
            in: calibration, profile: profile)
        XCTAssertNil(inclusive.rejection, "Age 0.500 at collection is valid even when retained evidence ages")
    }

    func testWrappedSampledResponseRetainsRateLatencyAndPartialTravelMaxima() {
        let calibration = initial(0)
        let profile = Planner.Profile(purpose: .alignment)
        let wrapped = [sample(3.1), sample(-2.983185307179586, 2, 100.1), sample(-2.883185307179586, 3, 100.4)]
        let first = Planner.recording(response(calibration, samples: wrapped), in: calibration, profile: profile)
        XCTAssertNil(first.rejection)
        XCTAssertEqual(first.calibration.responseRate, 3.33333333333333, accuracy: 1e-12)
        XCTAssertEqual(first.calibration.observedPostAckTravel ?? -1, 0.1, accuracy: 1e-12)
        guard case let .burst(direction, budget) = Planner.plan(.init(actualYaw: -0.4, profile: profile,
            calibration: first.calibration, sendEntryUptime: 101)) else { return XCTFail() }
        XCTAssertEqual(direction, 1)
        XCTAssertEqual(budget, 0.080, accuracy: 1e-12, "A sufficiently large correction remains executable")
        let low = Planner.recording(response(calibration, samples: [sample(0, 4, 100.9), sample(0, 5, 101.4)],
            send: 0.001, stop: 0.001, entry: 101), in: first.calibration, profile: profile)
        XCTAssertNil(low.rejection)
        XCTAssertEqual(low.calibration.responseRate, first.calibration.responseRate)
        XCTAssertEqual(low.calibration.maximumSendDuration, first.calibration.maximumSendDuration)
        XCTAssertEqual(low.calibration.maximumStopDuration, first.calibration.maximumStopDuration)
        XCTAssertEqual(low.calibration.observedPostAckTravel, first.calibration.observedPostAckTravel)
        XCTAssertEqual(low.calibration.responseOverhead, first.calibration.responseOverhead)
        XCTAssertEqual(low.calibration.responseOverhead, 0.01, accuracy: 1e-12)
        let higher = Planner.recording(response(calibration,
            samples: [sample(0, 6, 101.9), sample(0.1, 7, 102.5)], stop: 0.04, entry: 102),
            in: low.calibration, profile: profile)
        XCTAssertNil(higher.rejection)
        XCTAssertEqual(higher.calibration.responseOverhead, 0.04, accuracy: 1e-12)
        let incomplete = Planner.recording(response(calibration,
            samples: [sample(0.1, 8, 103.6)], stop: 0.5, entry: 103),
            in: higher.calibration, profile: profile)
        XCTAssertNil(incomplete.rejection)
        XCTAssertEqual(incomplete.calibration.responseOverhead, higher.calibration.responseOverhead,
            "Incomplete later evidence cannot raise or erase the learned overhead")
        let single = Planner.recording(response(calibration, samples: [sample(0, 2, 100.4)]), in: calibration, profile: profile)
        XCTAssertNil(single.calibration.observedPostAckTravel, "A single post-ack frame cannot establish zero coast")
        let consecutive = Planner.recording(response(calibration, samples: [sample(0), sample(0.5, 2, 100),
            sample(0.2, 3, 100.4)]), in: calibration, profile: profile)
        XCTAssertEqual(consecutive.calibration.responseRate, 8.88888888888889, accuracy: 1e-10)
        let negative = Planner.recording(response(calibration, samples: [sample(-3.1), sample(2.983185307179586, 2, 100.1),
            sample(2.883185307179586, 3, 100.4)]), in: calibration, profile: profile)
        XCTAssertEqual(negative.calibration.responseRate, 3.33333333333333, accuracy: 1e-12)
    }

    func testDirectedOvershootShrinksNextCeilingWithoutMistakingPiSeamForCrossing() {
        let profile = Planner.Profile(purpose: .alignment)
        for sign in [1.0, -1.0] {
            let calibration = initial(sign * 0.1)
            let reduced = Planner.recording(response(calibration, samples: [sample(0), sample(sign * 0.2, 2, 100.4)],
                send: 0, stop: 0), in: calibration, profile: profile)
            XCTAssertNil(reduced.rejection)
            XCTAssertEqual(reduced.calibration.overshootCeiling ?? -1, 0.005093414960113412, accuracy: 1e-12)
            guard case let .burst(direction, budget) = Planner.plan(.init(actualYaw: sign * 0.2, profile: profile,
                calibration: reduced.calibration, sendEntryUptime: 101)) else { return XCTFail() }
            XCTAssertEqual(direction, sign > 0 ? -1 : 1)
            XCTAssertEqual(budget, 0.005093414960113412, accuracy: 1e-12)
            guard case let .burst(_, largerBudget) = Planner.plan(.init(actualYaw: -sign, profile: profile,
                calibration: reduced.calibration, sendEntryUptime: 101)) else { return XCTFail() }
            XCTAssertEqual(largerBudget, 0.005093414960113412, accuracy: 1e-12)
        }
        let seam = initial(-3.0)
        let notCrossed = Planner.recording(response(seam, samples: [sample(0), sample(0.2, 2, 100.4)],
            send: 0, stop: 0), in: seam, profile: profile)
        XCTAssertNil(notCrossed.calibration.overshootCeiling, "Normalized error changes sign here without directed target crossing")
        let ambiguous = Planner.recording(response(initial(0.1), samples: [sample(0), sample(.pi, 2, 100.4)]),
            in: initial(0.1), profile: profile)
        XCTAssertNotNil(ambiguous.rejection)
        XCTAssertEqual(Planner.plan(.init(actualYaw: 0.2, profile: profile, calibration: ambiguous.calibration,
            sendEntryUptime: 101, provisionalProbeIssued: true)), .resolutionFailure(.rotationResolutionInsufficient))
    }

    func testReplayedResponseAndFramesCannotCalibrateANewerBurst() {
        let calibration = initial(1)
        let profile = Planner.Profile(purpose: .alignment)
        let old = response(calibration, samples: [sample(0), sample(0.1, 2, 100.4)])
        let retained = Planner.recording(old, in: calibration, profile: profile).calibration
        let replay = Planner.recording(old, in: retained, profile: profile)
        XCTAssertNotNil(replay.rejection)
        XCTAssertEqual(replay.calibration, retained)
        let oldIDs = Planner.recording(response(calibration, samples: [sample(0.1, 2, 100.9), sample(0.2, 3, 101.4)],
            entry: 101), in: retained, profile: profile)
        XCTAssertNotNil(oldIDs.rejection)
        XCTAssertEqual(oldIDs.calibration, retained)
        let staleSourceTime = Planner.recording(response(calibration, samples: [sample(0.1, 3, 100.4, collected: 100.9),
            sample(0.2, 4, 101.4)], entry: 101), in: retained, profile: profile)
        XCTAssertNotNil(staleSourceTime.rejection)
        XCTAssertEqual(staleSourceTime.calibration, retained)
    }

    func testSampledTravelPreservesReversalsAndRejectsNonfiniteRateWithoutFullTurnInference() {
        let calibration = initial(1)
        let profile = Planner.Profile(purpose: .alignment)
        let reduced = Planner.recording(response(calibration, samples: [sample(0), sample(0.5, 2, 100),
            sample(0.2, 3, 100.4)]), in: calibration, profile: profile)
        XCTAssertEqual(reduced.signedResponse ?? -1, 0.2, accuracy: 1e-12)
        XCTAssertEqual(reduced.sampledAbsoluteTravel ?? -1, 0.8, accuracy: 1e-12)
        let nonfinite = Planner.recording(response(calibration, budget: Double.leastNonzeroMagnitude,
            samples: [sample(0), sample(1, 2, 100.4)]), in: calibration, profile: profile)
        XCTAssertNotNil(nonfinite.rejection)
        XCTAssertEqual(nonfinite.calibration, calibration)
        let unknown = Planner.recording(response(calibration), in: calibration, profile: profile)
        XCTAssertNil(unknown.signedResponse)
        XCTAssertNil(unknown.sampledAbsoluteTravel)
    }

    func testAdjacentBurstsMayShareTheExactSettledBoundaryWithoutFabricatedAdvancement() {
        let calibration = initial(1)
        let profile = Planner.Profile(purpose: .alignment)
        let settled = sample(0.1, 2, 100.4)
        let first = Planner.recording(response(calibration, samples: [sample(0), settled]), in: calibration, profile: profile)
        let next = Planner.recording(response(calibration, samples: [settled, sample(0.2, 3, 101.4)], entry: 101),
            in: first.calibration, profile: profile)
        XCTAssertNil(next.rejection)
        XCTAssertEqual(next.calibration.completedResponses, 2)
        let forged = Planner.recording(response(calibration, samples: [sample(0.15, 2, 100.4), sample(0.2, 3, 101.4)],
            entry: 101), in: first.calibration, profile: profile)
        XCTAssertNotNil(forged.rejection)
        XCTAssertEqual(forged.calibration, first.calibration)
    }

    func testPendingDrainContributesOnceToWindowOverhead() {
        let calibration = initial(0.3625)
        let profile = Planner.Profile(purpose: .alignment)
        let measured = Planner.recording(response(calibration, samples: [sample(0), sample(0.2, 2, 100.6)],
            send: 0, stop: 0.125), in: calibration, profile: profile)
        XCTAssertNil(measured.rejection)
        guard case let .burst(direction, budget) = Planner.plan(.init(actualYaw: 0, profile: profile,
            calibration: measured.calibration, sendEntryUptime: 101)) else { return XCTFail() }
        XCTAssertEqual(direction, 1)
        XCTAssertEqual(budget, 0.080, accuracy: 1e-12, "The measured response fits the remaining heading corridor")
        let drain = Planner.Response(operationID: 1, generation: 7, targetYaw: 1, clockDomain: "test_uptime",
            requestedBudget: 0.080, sendEntryUptime: 100, sendResponseUptime: 100.2,
            stopObligationUptime: 100.08, stopAcknowledgementUptime: 100.23, samples: [])
        let result = Planner.recording(drain, in: initial(1), profile: profile)
        XCTAssertNil(result.rejection)
        XCTAssertEqual(result.calibration.maximumSendDuration ?? -1, 0.2, accuracy: 1e-12)
        XCTAssertEqual(result.calibration.maximumStopDuration ?? -1, 0.15, accuracy: 1e-12)
    }

    func testFiniteEndpointValuesCannotHideAnOverflowedTargetError() {
        let calibration = initial(Double.greatestFiniteMagnitude)
        let result = Planner.recording(response(calibration, samples: [sample(-Double.greatestFiniteMagnitude),
            sample(-Double.greatestFiniteMagnitude, 2, 100.4)]), in: calibration, profile: .init(purpose: .alignment))
        XCTAssertNotNil(result.rejection)
        XCTAssertEqual(result.calibration, calibration)
    }

    func testPartialPostAckBracketMeasuresTravelWithoutInventingNetBurstResponse() {
        let calibration = initial(1)
        let reduced = Planner.recording(response(calibration, samples: [sample(0.1, 2, 100.1), sample(0.2, 3, 100.4)]),
            in: calibration, profile: .init(purpose: .alignment))
        XCTAssertNil(reduced.rejection)
        XCTAssertEqual(reduced.calibration.completedResponses, 1)
        XCTAssertEqual(reduced.calibration.maximumSendDuration ?? -1, 0.010, accuracy: 1e-12)
        XCTAssertEqual(reduced.calibration.observedPostAckTravel ?? -1, 0.1, accuracy: 1e-12)
        XCTAssertEqual(reduced.calibration.responseRate, 2.0943951023931953, accuracy: 1e-12)
        XCTAssertNil(reduced.signedResponse, "No pre-send endpoint means no measured net burst response")
    }
}
