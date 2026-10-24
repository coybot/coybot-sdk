import XCTest
import RoverNav
import PhroverKit

final class FollowReacquisitionPlannerTests: XCTestCase {
    func testObservationSearchCanCapRecoverySegmentsAtTenDegrees() {
        guard case .turn(let delta, _) = FollowReacquisitionPlanner.resolveAbsoluteStage(
            stageHeading: 1, actualYaw: 0, maximumSegment: .pi / 18) else { return XCTFail() }
        XCTAssertEqual(delta, .pi / 18, accuracy: 1e-12)
        XCTAssertEqual(FollowReacquisitionPlanner.resolveAbsoluteStage(stageHeading: 1, actualYaw: 0,
            maximumSegment: .nan), .unavailable)
    }
    func testAbsoluteStageResolverContractUsesShortestSignedBoundedSegmentFromActualYaw() {
        let degrees = Double.pi / 180
        // Independently worked headings in degrees: crossing the seam, exact half
        // turns, measured overshoot, and a final segment shorter than the cap.
        for (stage, yaw, delta, target) in [
            (-130.0, 170.0, 30.0, -160.0),
            (130.0, -170.0, -30.0, 160.0),
            (180.0, 0.0, -30.0, -30.0),
            (-180.0, 0.0, -30.0, -30.0),
            (15.0, 40.0, -25.0, 15.0),
            (45.0, 30.0, 15.0, 45.0),
            (405.0, 30.0, 15.0, 45.0)
        ] {
            guard case .turn(let actualDelta, let actualTarget) = FollowReacquisitionPlanner.resolveAbsoluteStage(
                stageHeading: stage * degrees, actualYaw: yaw * degrees) else {
                XCTFail("Expected a turn from \(yaw) to stage \(stage)"); continue
            }
            XCTAssertEqual(actualDelta / degrees, delta, accuracy: 1e-10)
            XCTAssertEqual(actualTarget / degrees, target, accuracy: 1e-10)
        }
        for yaw in [-7.0, 0, 7] {
            XCTAssertEqual(FollowReacquisitionPlanner.resolveAbsoluteStage(stageHeading: 0, actualYaw: yaw * degrees), .stageArrived)
        }
        guard case .turn(let delta, _) = FollowReacquisitionPlanner.resolveAbsoluteStage(stageHeading: 0, actualYaw: 7.001 * degrees) else {
            return XCTFail("Outside the inclusive 7 degree arrival boundary must still turn")
        }
        XCTAssertEqual(delta / degrees, -7.001, accuracy: 1e-10)
        XCTAssertEqual(FollowReacquisitionPlanner.resolveAbsoluteStage(stageHeading: .nan, actualYaw: 0), .unavailable)
        XCTAssertEqual(FollowReacquisitionPlanner.resolveAbsoluteStage(stageHeading: 0, actualYaw: .infinity), .unavailable)
    }

    func testHistoricalHeadingOnlyFallbackDoesNotFabricatePairedPosition() {
        let anchor = FollowReliableMemory(position: nil, pairedPose: nil, bearing: 0.25,
            frameID: .init(generation: 4, sequence: 1), timestamp: 1, association: .continued, pairedYaw: 0.5)
        let current = FollowRecoveryPose(pose: .init(position: Vec2(3, 4), yaw: -1),
            frameID: .init(generation: 4, sequence: 10), timestamp: 8, trackingQuality: .normal)
        let selected = FollowReacquisitionPlanner.selectCenter(anchor: anchor, current: current, now: 8)
        XCTAssertTrue(selected?.heading == 0.75 && selected?.source == .historicalPairedViewHeading && anchor.pairedPose == nil)
    }

    func testSegmentCursorRetainsInterruptedTargetAndFinitePassExhausts() {
        let anchor = FollowReliableMemory(position: Vec2(1, 0), pairedPose: nil, bearing: nil,
            frameID: .init(generation: 4, sequence: 1), timestamp: 1, association: .continued)
        func sample(_ yaw: Double) -> FollowRecoveryPose {
            .init(pose: .init(position: .zero, yaw: yaw), frameID: .init(generation: 4, sequence: 10), timestamp: 8, trackingQuality: .normal)
        }
        var episode = FollowReacquisitionEpisode(firstLoss: 2, anchor: anchor).selectingCenter(current: sample(1), now: 8)
        let interrupted = episode.recordingArrival(actual: sample(0.75), now: 8)
        let segment = episode.recordingSegmentArrival(target: 0.5, actual: sample(0.5), now: 8)
        let notArrived = segment.recordingSegmentArrival(target: 0, actual: sample(0.5), now: 8)
        let cursors = [interrupted.segmentIndex, segment.segmentIndex, notArrived.segmentIndex]
        for yaw in [0.0, 15, -15, 30, -30, 45, -45].map({ $0 * .pi / 180 }) {
            episode = episode.recordingArrival(actual: sample(yaw), now: 8)
        }
        let exhausted = episode.recordingArrival(actual: sample(0), now: 8)
        XCTAssertEqual(cursors + [episode.stageIndex, exhausted.stageIndex], [0, 1, 1, 7, 7])
    }

    func testAcceptedMemoryRetainsSameObservationGeometryAndRejectsInvalidSource() {
        let observation = FollowPersonObservation(frameID: .init(generation: 4, sequence: 12), timestamp: 8,
            confidence: 0.9, boundingBox: .init(x: 0.4, y: 0.2, width: 0.1, height: 0.5),
            position: Vec2(3, 4), pose: .init(position: Vec2(3, 2), yaw: 0.5), rawPersonID: 7)
        let memory = FollowReliableMemory(accepted: observation, association: .acceptedPendingContinuity, now: 8.5, trackingQuality: .normal)
        XCTAssertEqual([memory?.position?.x, memory?.pairedPose?.position.y, memory?.pairedPose?.yaw,
            memory?.bearing, memory?.timestamp, memory.map { Double($0.frameID.sequence) }, memory?.rawPersonID.map(Double.init),
            FollowReliableMemory(accepted: observation, association: .initial, now: 8.501, trackingQuality: .normal) == nil ? 1 : 0,
            FollowReliableMemory(accepted: observation, association: .initial, now: 7.9, trackingQuality: .normal) == nil ? 1 : 0,
            FollowReliableMemory(accepted: observation, association: .initial, now: 8, trackingQuality: .limited) == nil ? 1 : 0],
            [3, 2, 0.5, Double.pi / 2 - 0.5, 8, 12, 7, 1, 1, 1])
    }

    func testEpisodeFreezesCenterAndAdvancesOnlyOnFreshMeasuredStageArrival() {
        let anchor = FollowReliableMemory(position: Vec2(1, 0), pairedPose: nil, bearing: nil,
            frameID: .init(generation: 4, sequence: 1), timestamp: 1, association: .continued)
        func sample(_ yaw: Double, position: Vec2 = .zero, generation: UInt64 = 4, time: Double = 8) -> FollowRecoveryPose {
            .init(pose: .init(position: position, yaw: yaw), frameID: .init(generation: generation, sequence: 10),
                timestamp: time, trackingQuality: .normal)
        }
        let original = FollowReacquisitionEpisode(firstLoss: 2, anchor: anchor)
        let selected = original.selectingCenter(current: sample(1), now: 8)
        let frozen = selected.selectingCenter(current: sample(1, position: Vec2(1, -1)), now: 8)
        let unfinished = frozen.recordingArrival(actual: sample(0.5), now: 8)
        let arrived = unfinished.recordingArrival(actual: sample(0), now: 8)
        let rejected = arrived.recordingArrival(actual: sample(15 * .pi / 180, generation: 5), now: 8)
        let expired = rejected.recordingArrival(actual: sample(15 * .pi / 180, time: 12), now: 12)
        XCTAssertEqual([original.stageIndex, selected.stageIndex, frozen.stageIndex, unfinished.stageIndex,
                        arrived.stageIndex, rejected.stageIndex, expired.stageIndex,
                        frozen.center?.heading == 0 ? 1 : 0,
                        frozen.id == original.id && frozen.deadline == 12 && frozen.anchor?.timestamp == 1 ? 1 : 0],
                       [0, 0, 0, 0, 1, 1, 1, 1, 1])
    }

    func testFixedStagesBoundSegmentsAndUseMeasuredOvershoot() {
        let planner = FollowReacquisitionPlanner.self
        let degrees = Double.pi / 180
        let decisions = (0..<8).map { planner.nextSegment(center: 0, stage: $0, actualYaw: 0) }
        XCTAssertEqual(decisions + [
            planner.nextSegment(center: 0.5, stage: 0, actualYaw: 0.75),
            planner.nextSegment(center: 0, stage: 4, actualYaw: 30 * degrees),
            planner.nextSegment(center: 0, stage: 0, actualYaw: 7 * degrees),
            planner.nextSegment(center: .pi, stage: 0, actualYaw: 0),
            planner.nextSegment(center: -179 * degrees, stage: 0, actualYaw: 179 * degrees)
        ], [
            .stageArrived, .turn(delta: 15 * degrees, target: 15 * degrees),
            .turn(delta: -15 * degrees, target: -15 * degrees),
            .turn(delta: 30 * degrees, target: 30 * degrees),
            .turn(delta: -30 * degrees, target: -30 * degrees),
            .turn(delta: 30 * degrees, target: 30 * degrees),
            .turn(delta: -30 * degrees, target: -30 * degrees), .exhausted,
            .turn(delta: -0.25, target: 0.5),
            .turn(delta: -30 * degrees, target: 0), .stageArrived,
            .turn(delta: -30 * degrees, target: -30 * degrees), .stageArrived
        ])
    }

    func testPairedFallbackWrapAndInvalidSources() {
        func heading(point: Vec2?, yaw: Double = 3, bearing: Double? = 0.5,
                     generation: UInt64 = 4, timestamp: Double = 8, now: Double = 8) -> Double? {
            let anchor = FollowReliableMemory(position: point, pairedPose: Pose2D(position: .zero, yaw: yaw),
                bearing: bearing, frameID: .init(generation: generation, sequence: 1), timestamp: 1, association: .initial)
            let current = FollowRecoveryPose(pose: Pose2D(position: .zero, yaw: 0),
                frameID: .init(generation: 4, sequence: 9), timestamp: timestamp, trackingQuality: .normal)
            return FollowReacquisitionPlanner.selectCenter(anchor: anchor, current: current, now: now)?.heading
        }
        XCTAssertEqual([
            heading(point: Vec2(-1, 0)), heading(point: nil), heading(point: .zero),
            heading(point: Vec2(.nan, 0)), heading(point: nil, bearing: nil),
            heading(point: nil, generation: 5), heading(point: Vec2(1, 0), timestamp: 7.499),
            heading(point: Vec2(1, 0), timestamp: 8.001), heading(point: Vec2(1, 0), timestamp: .nan),
            heading(point: Vec2(1, 0), timestamp: 7.5), heading(point: Vec2(0, -1)), heading(point: Vec2(1, 1))
        ], [-Double.pi, -2.7831853071795862, -2.7831853071795862, -2.7831853071795862,
            nil, nil, nil, nil, nil, 0, -Double.pi / 2, Double.pi / 4])
    }

    func testCenterUsesActualCurrentPositionInsteadOfHistoricalYaw() {
        let anchor = FollowReliableMemory(position: Vec2(2, 2), pairedPose: Pose2D(position: .zero, yaw: -1),
            bearing: 1.7853981633974483, frameID: .init(generation: 4, sequence: 1), timestamp: 1, association: .continued)
        let current = FollowRecoveryPose(pose: Pose2D(position: Vec2(2, 0), yaw: -2),
            frameID: .init(generation: 4, sequence: 9), timestamp: 8, trackingQuality: .normal)
        XCTAssertEqual(FollowReacquisitionPlanner.selectCenter(anchor: anchor, current: current, now: 8)?.heading,
            Double.pi / 2)
    }
}
