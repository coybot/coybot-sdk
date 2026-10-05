import XCTest
import CoreGraphics
import RoverNav
@testable import PhroverKit

@MainActor
final class RoomTransitionMissionTests: XCTestCase {
    func testSuccessfulTransitionPublishesOneOrderedTerminalCommandStatus() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 1, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1)],
            observations: crossingObservations(generation: 1)
        )
        var statuses: [MissionCommandStatus] = []
        let agent = makeAgent(
            motion: RoomMissionMotion(),
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology,
            commandStatusDidChange: { statuses.append($0) }
        )

        await agent.handle("Go to the other room")

        XCTAssertEqual(statuses, [
            .recognized(id: 1, command: "Go to the other room"),
            .working(id: 1, command: "Go to the other room"),
            .succeeded(id: 1, command: "Go to the other room", message: "Entered another room."),
        ])
        XCTAssertEqual(statuses.filter(\.isTerminal).count, 1)
    }

    func testExhaustionAndScanFailurePublishSpecificTerminalFailures() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 1, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [])
        let motion = RoomMissionMotion()
        var statuses: [MissionCommandStatus] = []
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology,
            commandStatusDidChange: { statuses.append($0) }
        )

        await agent.handle("enter the next room")
        XCTAssertEqual(
            statuses.last,
            .failed(
                id: 1,
                command: "enter the next room",
                message: "I couldn’t find a safe route into another room."
            )
        )

        motion.rotationOutcomes = [.failed("I can’t safely see the space needed to turn. Reposition the rover or camera and try again.")]
        await agent.handle("enter the next room")
        XCTAssertEqual(
            statuses.last,
            .failed(
                id: 2,
                command: "enter the next room",
                message: "I can’t safely see the space needed to turn. Reposition the rover or camera and try again."
            )
        )
        XCTAssertEqual(statuses.filter(\.isTerminal).count, 2)
    }

    func testMissingPosePublishesRecognizedThenFailureWithoutWorking() async {
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: RoomMissionMotion(),
            perception: RoomMissionPerception(pose: nil, frontiers: []),
            voice: RoomMissionVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { nil }
        )

        await agent.handle("go somewhere")

        XCTAssertEqual(statuses, [
            .recognized(id: 1, command: "go somewhere"),
            .failed(id: 1, command: "go somewhere", message: "I don’t have my bearings yet."),
        ])
    }

    func testMissingReferencePosePublishesOneTerminalFailure() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 1, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [])
        perception.poseReadResults = [pose(x: 0), nil]
        var statuses: [MissionCommandStatus] = []
        let agent = makeAgent(
            motion: RoomMissionMotion(),
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology,
            commandStatusDidChange: { statuses.append($0) }
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(statuses, [
            .recognized(id: 1, command: "go to the other room"),
            .working(id: 1, command: "go to the other room"),
            .failed(id: 1, command: "go to the other room", message: "I don’t have my bearings yet."),
        ])
        XCTAssertEqual(statuses.filter(\.isTerminal).count, 1)
    }

    func testMissingApproachPosePublishesOneTerminalFailure() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 1, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1)]
        )
        perception.poseReadResults = [pose(x: 0), pose(x: 0), nil]
        var statuses: [MissionCommandStatus] = []
        let agent = makeAgent(
            motion: RoomMissionMotion(),
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology,
            commandStatusDidChange: { statuses.append($0) }
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(statuses.last, .failed(
            id: 1,
            command: "go to the other room",
            message: "I don’t have my bearings yet."
        ))
        XCTAssertEqual(statuses.filter(\.isTerminal).count, 1)
    }

    func testCorrectedStartupReassessesAndNeverNavigatesToStaleGoal() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 1, initialPose: pose(x: -1))
        let perception = RoomMissionPerception(
            pose: pose(x: -1),
            frontiers: [doorwayFrontier(x: 0)],
            observations: [0.4, -0.1, -0.36, -0.37, -0.38].enumerated().map { index, x in
                observation(
                    x: x,
                    sequence: UInt64(index + 1),
                    quality: .normal,
                    generation: 1
                )
            }
        )
        perception.poseReadResults = [pose(x: -1), pose(x: -1), pose(x: 1)]
        let motion = RoomMissionMotion()
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(motion.assessedGoals, [Vec2(0.6, 0), Vec2(-0.6, 0)])
        XCTAssertEqual(motion.navigateCalls, [Vec2(-0.6, 0)])
        XCTAssertFalse(motion.navigateCalls.contains(Vec2(0.6, 0)))
    }
    func testStandaloneRoomTransitionBypassesBrainAndConfirmsAfterAwaitedStop() async {
        var timeline: [String] = []
        var telemetry: [(String, [String: String])] = []
        var debugStates: [RoomTransitionDebugState] = []
        let topology = SessionRoomTopology { event, fields in
            timeline.append(event)
            telemetry.append((event, fields))
        }
        topology.startSession(generation: 7, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1)],
            observations: crossingObservations(generation: 7)
        )
        let motion = RoomMissionMotion(state: .failed("stale"))
        motion.onStop = {
            XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
            timeline.append("stop_completed")
        }
        let voice = RoomMissionVoice()
        let brain = RoomMissionBrain()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: voice,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            roomTransitionStateDidChange: { debugStates.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the other room")

        XCTAssertEqual(brain.callCount, 0)
        XCTAssertEqual(motion.cancelCallCount, 0)
        XCTAssertEqual(motion.assessedGoals, [Vec2(1.6, 0)])
        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0)])
        XCTAssertEqual(motion.stopAndWaitCallCount, 2)
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_2"))
        XCTAssertLessThan(
            try XCTUnwrap(timeline.firstIndex(of: "stop_completed")),
            try XCTUnwrap(timeline.firstIndex(of: "room_transition_completed"))
        )
        XCTAssertTrue(timeline.contains("doorway_candidate_ranked"))
        XCTAssertTrue(timeline.contains("room_transition_started"))
        XCTAssertTrue(timeline.contains("doorway_crossed"))
        XCTAssertEqual(
            telemetry.first { $0.0 == "room_transition_started" }?.1["candidate_id"],
            "doorway_candidate_1"
        )
        XCTAssertEqual(telemetry.first { $0.0 == "doorway_crossed" }?.1["doorway_id"], "doorway_1")
        XCTAssertEqual(telemetry.first { $0.0 == "doorway_crossed" }?.1["from_room_id"], "room_1")
        XCTAssertEqual(telemetry.first { $0.0 == "doorway_crossed" }?.1["to_room_id"], "room_2")
        XCTAssertEqual(debugStates, [
            .scanning(step: 0, total: 12, openingCount: 1, candidateCount: 1),
            .candidateFound(id: DoorwayCandidateID("doorway_candidate_1"), reachable: true),
            .approaching(id: DoorwayCandidateID("doorway_candidate_1")),
            .confirmingCrossing(id: DoorwayCandidateID("doorway_candidate_1")),
            .completed(doorwayID: DoorwayID("doorway_1"), roomID: RoomID("room_2")),
        ])
        XCTAssertEqual(agent.phase, .idle)
    }

    func testMissionTelemetryReplaysSuccessfulSelectionAndCrossing() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 21, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1)],
            observations: crossingObservations(generation: 21)
        )
        var events: [(String, [String: String])] = []
        let agent = makeAgent(
            motion: RoomMissionMotion(),
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology,
            roomTransitionTelemetry: { events.append(($0, $1)) }
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(events.map(\.0), [
            "room_transition_scan_step",
            "doorway_candidate_assessed",
            "doorway_candidate_selected",
            "room_transition_approach_started",
            "room_transition_crossing_confirmation_started",
            "room_transition_mission_completed",
        ])
        XCTAssertEqual(events.first?.1["mission_id"], "1")
        XCTAssertEqual(events.first?.1["session_generation"], "21")
        XCTAssertEqual(events.first?.1["scan_step"], "0")
        XCTAssertEqual(events.first { $0.0 == "doorway_candidate_assessed" }?.1["rank"], "1")
        XCTAssertEqual(events.first { $0.0 == "doorway_candidate_selected" }?.1["candidate_id"], "doorway_candidate_1")
        XCTAssertEqual(events.last?.1["room_id"], "room_2")
    }

    func testObjectTargetMentioningOtherRoomRetainsBrainPath() async {
        let motion = RoomMissionMotion()
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [])
        let brain = RoomMissionBrain()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            currentBrain: { brain }
        )

        await agent.handle("go to the chair in the other room")

        XCTAssertEqual(brain.callCount, 1)
        XCTAssertTrue(motion.assessedGoals.isEmpty)
    }

    func testNoCandidateUsesOneTwelveStepScanBudgetAndExactExhaustionResponse() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 1, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [])
        let motion = RoomMissionMotion()
        let voice = RoomMissionVoice()
        var debugStates: [RoomTransitionDebugState] = []
        var missionTelemetry: [(String, [String: String])] = []
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: voice,
            topology: topology,
            roomTransitionStateDidChange: { debugStates.append($0) },
            roomTransitionTelemetry: { missionTelemetry.append(($0, $1)) }
        )

        await agent.handle("enter the next room")

        XCTAssertEqual(motion.rotateCalls, Array(repeating: .pi / 6, count: 12))
        XCTAssertEqual(perception.refreshCount, 13)
        XCTAssertEqual(voice.spoken, ["I couldn’t find a safe route into another room."])
        XCTAssertEqual(topology.snapshot.rooms.map(\.id), [RoomID("room_1")])
        XCTAssertEqual(
            debugStates.compactMap { state -> Int? in
                guard case .scanning(let step, _, _, _) = state else { return nil }
                return step
            },
            Array(0...12)
        )
        XCTAssertEqual(debugStates.last, .exhausted)
        XCTAssertEqual(missionTelemetry.filter { $0.0 == "room_transition_scan_step" }.count, 13)
        XCTAssertEqual(
            missionTelemetry.filter { $0.0 == "room_transition_scan_continued" }.map { $0.1["reason"] },
            Array(repeating: "no_openings", count: 13)
        )
        XCTAssertEqual(missionTelemetry.last?.0, "room_transition_exhausted")
        XCTAssertEqual(missionTelemetry.last?.1["scan_steps_used"], "12")
    }

    func testRejectedOpeningTelemetryExplainsWhyScanContinues() async {
        var topologyEvents: [(String, [String: String])] = []
        let topology = SessionRoomTopology { topologyEvents.append(($0, $1)) }
        topology.startSession(generation: 22, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1, widthMeters: 0.4)]
        )
        var missionTelemetry: [(String, [String: String])] = []
        let agent = makeAgent(
            motion: RoomMissionMotion(),
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology,
            roomTransitionTelemetry: { missionTelemetry.append(($0, $1)) }
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(
            topologyEvents.filter { $0.0 == "doorway_frontier_rejected" }.first?.1["reason"],
            "below_safe_width"
        )
        XCTAssertEqual(
            missionTelemetry.filter { $0.0 == "room_transition_scan_continued" }.map { $0.1["reason"] },
            Array(repeating: "no_admitted_candidates", count: 13)
        )
        XCTAssertEqual(missionTelemetry.last?.0, "room_transition_exhausted")
    }

    func testUnreachableCandidateIsReportedAndNeverApproached() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 10, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [doorwayFrontier(x: 1)])
        let motion = RoomMissionMotion()
        motion.assessmentReachability = [false]
        var debugStates: [RoomTransitionDebugState] = []
        var missionTelemetry: [(String, [String: String])] = []
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology,
            roomTransitionStateDidChange: { debugStates.append($0) },
            roomTransitionTelemetry: { missionTelemetry.append(($0, $1)) }
        )

        await agent.handle("go to the other room")

        let candidateID = DoorwayCandidateID("doorway_candidate_1")
        XCTAssertTrue(debugStates.contains(.candidateFound(id: candidateID, reachable: false)))
        XCTAssertTrue(debugStates.contains(.unreachable(id: candidateID)))
        XCTAssertFalse(debugStates.contains(.approaching(id: candidateID)))
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(debugStates.last, .exhausted)
        XCTAssertFalse(missionTelemetry.contains { $0.0 == "doorway_candidate_selected" })
        XCTAssertTrue(missionTelemetry.contains { $0.0 == "doorway_candidate_unreachable" })
        XCTAssertTrue(missionTelemetry.contains { $0.0 == "room_transition_scan_continued" && $0.1["reason"] == "no_reachable_candidates" })
        XCTAssertTrue(missionTelemetry.contains { $0.0 == "room_transition_scan_continued" && $0.1["reason"] == "no_ranked_candidates" })
        XCTAssertEqual(missionTelemetry.last?.0, "room_transition_exhausted")
    }

    func testScanRotationFailureStopsMissionWithFailedDiagnostic() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 11, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [])
        let motion = RoomMissionMotion()
        motion.rotationOutcomes = [.failed("Motion heading became unavailable during scan.")]
        let voice = RoomMissionVoice()
        var debugStates: [RoomTransitionDebugState] = []
        var missionTelemetry: [(String, [String: String])] = []
        var statuses: [MissionCommandStatus] = []
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: voice,
            topology: topology,
            roomTransitionStateDidChange: { debugStates.append($0) },
            roomTransitionTelemetry: { missionTelemetry.append(($0, $1)) },
            commandStatusDidChange: { statuses.append($0) }
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(motion.rotateCalls.count, 1)
        XCTAssertEqual(
            debugStates.last,
            .failed(reason: "Motion heading became unavailable during scan.")
        )
        XCTAssertFalse(debugStates.contains(.exhausted))
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
        XCTAssertEqual(missionTelemetry.last?.0, "room_transition_failed")
        XCTAssertEqual(missionTelemetry.last?.1["reason"], "Motion heading became unavailable during scan.")
        XCTAssertEqual(missionTelemetry.last?.1["scan_step"], "0")
        XCTAssertEqual(statuses.last, .failed(
            id: 1,
            command: "go to the other room",
            message: "Motion heading became unavailable during scan."
        ))
        XCTAssertEqual(statuses.filter(\.isTerminal).count, 1)
    }

    func testWrongGenerationHighSequenceDoesNotHideCurrentGenerationCrossing() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 12, initialPose: pose(x: 0))
        let observations = [
            observation(x: 0.2, sequence: 100, quality: .normal, generation: 99),
        ] + crossingObservations(generation: 12)
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1)],
            observations: observations
        )
        let motion = RoomMissionMotion()
        perception.onObservationQueueEmpty = { motion.state = .arrived }
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_2"))
    }

    func testCandidateFailuresRefreshAndAttemptAtMostThreeWithoutRepeating() async {
        var events: [(String, [String: String])] = []
        let topology = SessionRoomTopology { events.append(($0, $1)) }
        topology.startSession(generation: 2, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [
                doorwayFrontier(x: 1, y: 0),
                doorwayFrontier(x: 1, y: 1),
                doorwayFrontier(x: 1, y: 2),
                doorwayFrontier(x: 1, y: 3),
            ]
        )
        let motion = RoomMissionMotion()
        motion.navigationOutcomes = [.failed("blocked"), .failed("blocked"), .failed("blocked")]
        let voice = RoomMissionVoice()
        let agent = makeAgent(motion: motion, perception: perception, voice: voice, topology: topology)

        await agent.handle("move into another room")

        XCTAssertEqual(motion.navigateCalls.count, 3)
        XCTAssertEqual(Set(motion.navigateCalls.map(\.y)).count, 3)
        XCTAssertGreaterThanOrEqual(perception.refreshCount, 3)
        XCTAssertEqual(topology.snapshot.rooms.map(\.id), [RoomID("room_1")])
        XCTAssertTrue(topology.snapshot.doorways.isEmpty)
        XCTAssertEqual(events.filter { $0.0 == "room_transition_rejected" }.count, 3)
        XCTAssertFalse(events.contains { $0.0 == "doorway_crossed" || $0.0 == "room_transition_completed" })
        XCTAssertEqual(voice.spoken, ["I couldn’t find a safe route into another room."])
    }

    func testTotalScanBudgetIsSharedAcrossDiscoveryAndCandidateFailure() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 3, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [])
        let motion = RoomMissionMotion()
        motion.navigationOutcomes = [.failed("blocked")]
        motion.onRotate = {
            if motion.rotateCalls.count == 1 {
                perception.frontiers = [doorwayFrontier(x: 1)]
            }
        }
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(motion.rotateCalls.count, 12)
        XCTAssertEqual(motion.navigateCalls.count, 1)
    }

    func testMissionExclusionDoesNotPersistIntoNextMission() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 4, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [doorwayFrontier(x: 1)])
        let motion = RoomMissionMotion()
        motion.navigationOutcomes = [.failed("blocked")]
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )

        await agent.handle("go to the other room")
        perception.observationQueue = crossingObservations(generation: 4)
        await agent.handle("go to the other room")

        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0), Vec2(1.6, 0)])
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_2"))
    }

    func testKnownDoorwayCanBeTraversedInReverseWithoutAFrontier() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 5, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1)],
            observations: crossingObservations(generation: 5)
        )
        let motion = RoomMissionMotion()
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )

        await agent.handle("go to the other room")
        perception.pose = pose(x: 1.45)
        perception.frontiers = []
        perception.observationQueue = reverseCrossingObservations(generation: 5)
        await agent.handle("enter the next room")

        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0), Vec2(0.4, 0)])
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertEqual(topology.snapshot.rooms.count, 2)
        XCTAssertEqual(topology.snapshot.doorways.count, 1)
    }

    func testTrackingInterruptionStopsAndResumesOnlyForFreshNormalGeneration() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 6, initialPose: pose(x: 0))
        let observations = [
            observation(x: 0.2, sequence: 1, quality: .limited, generation: 6),
            observation(x: 0.3, sequence: 2, quality: .normal, generation: 99),
        ] + crossingObservations(generation: 6).map {
            observation(
                x: $0.pose.position.x,
                sequence: $0.frameSequence + 2,
                quality: .normal,
                generation: 6
            )
        }
        let perception = RoomMissionPerception(
            pose: pose(x: 0),
            frontiers: [doorwayFrontier(x: 1)],
            observations: observations
        )
        let motion = RoomMissionMotion()
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )

        await agent.handle("go to the other room")

        XCTAssertEqual(motion.stopAndWaitCallCount, 2)
        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0), Vec2(1.6, 0)])
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_2"))
    }

    func testStopAbandonsPendingTransitionWithoutGraphMutation() async {
        var events: [String] = []
        let topology = SessionRoomTopology { event, _ in events.append(event) }
        topology.startSession(generation: 8, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [doorwayFrontier(x: 1)])
        let motion = RoomMissionMotion()
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )
        let mission = Task { await agent.handle("go to the other room") }
        while topology.snapshot.pendingTransitionCandidateID == nil { await Task.yield() }

        await agent.handle("stop")
        await mission.value

        XCTAssertNil(topology.snapshot.pendingTransitionCandidateID)
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertTrue(topology.snapshot.doorways.isEmpty)
        XCTAssertTrue(events.contains("room_transition_rejected"))
        XCTAssertFalse(events.contains("doorway_crossed"))
        XCTAssertFalse(events.contains("room_transition_completed"))
    }

    func testTaskCancellationAbandonsPendingTransitionWithoutGraphMutation() async {
        var events: [String] = []
        let topology = SessionRoomTopology { event, _ in events.append(event) }
        topology.startSession(generation: 9, initialPose: pose(x: 0))
        let perception = RoomMissionPerception(pose: pose(x: 0), frontiers: [doorwayFrontier(x: 1)])
        let motion = RoomMissionMotion()
        let agent = makeAgent(
            motion: motion,
            perception: perception,
            voice: RoomMissionVoice(),
            topology: topology
        )
        let mission = Task { await agent.handle("go to the other room") }
        while topology.snapshot.pendingTransitionCandidateID == nil { await Task.yield() }

        mission.cancel()
        await mission.value

        XCTAssertNil(topology.snapshot.pendingTransitionCandidateID)
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertTrue(topology.snapshot.doorways.isEmpty)
        XCTAssertGreaterThanOrEqual(motion.cancelCallCount, 1)
        XCTAssertTrue(events.contains("room_transition_rejected"))
        XCTAssertFalse(events.contains("doorway_crossed"))
        XCTAssertFalse(events.contains("room_transition_completed"))
    }

    private func makeAgent(
        motion: RoomMissionMotion,
        perception: RoomMissionPerception,
        voice: RoomMissionVoice,
        topology: SessionRoomTopology,
        roomTransitionStateDidChange: ((RoomTransitionDebugState) -> Void)? = nil,
        roomTransitionTelemetry: @escaping RoomTopologyTelemetrySink = { _, _ in },
        commandStatusDidChange: ((MissionCommandStatus) -> Void)? = nil
    ) -> MissionAgent {
        let existingOnRotate = motion.onRotate
        motion.onRotate = {
            existingOnRotate?()
            perception.publishScanFrame(
                sessionGeneration: topology.snapshot.sessionGeneration
            )
        }
        return MissionAgent(
            motion: motion,
            perception: perception,
            voice: voice,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            roomTransitionStateDidChange: roomTransitionStateDidChange,
            roomTransitionTelemetry: roomTransitionTelemetry,
            commandStatusDidChange: commandStatusDidChange,
            currentBrain: { nil }
        )
    }
}

@MainActor
private final class RoomMissionBrain: RoverBrain {
    private(set) var callCount = 0

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        callCount += 1
        return BrainOutput(decision: .done)
    }
}

@MainActor
private final class RoomMissionMotion: RoverMotion {
    var state: NavigationController.State
    var navigateCalls: [Vec2] = []
    var assessedGoals: [Vec2] = []
    var rotateCalls: [Double] = []
    var cancelCallCount = 0
    var stopAndWaitCallCount = 0
    var assessmentReachability: [Bool] = []
    var navigationOutcomes: [NavigationController.State] = []
    var rotationOutcomes: [NavigationController.State] = []
    var onStop: (() -> Void)?
    var onRotate: (() -> Void)?

    init(state: NavigationController.State = .idle) {
        self.state = state
    }

    func navigate(to goal: Vec2) {
        navigateCalls.append(goal)
        state = navigationOutcomes.isEmpty ? .driving : navigationOutcomes.removeFirst()
    }

    func rotate(by angle: Double) async {
        rotateCalls.append(angle)
        state = rotationOutcomes.isEmpty ? .idle : rotationOutcomes.removeFirst()
        onRotate?()
    }

    func rotateForScan(by angle: Double) async {
        await rotate(by: angle)
    }

    func assessGoal(_ goal: Vec2) -> NavigationGoalAssessment {
        assessedGoals.append(goal)
        let reachable = assessmentReachability.isEmpty ? true : assessmentReachability.removeFirst()
        return NavigationGoalAssessment(goal: goal, isReachable: reachable, pathDistance: goal.x)
    }

    func stopAndWait() async {
        stopAndWaitCallCount += 1
        state = .idle
        onStop?()
    }

    func cancel() {
        cancelCallCount += 1
        state = .idle
    }
}

@MainActor
private final class RoomMissionPerception: RoverPerception {
    private var storedPose: Pose2D?
    var poseReadResults: [Pose2D?] = []
    var pose: Pose2D? {
        get { poseReadResults.isEmpty ? storedPose : poseReadResults.removeFirst() }
        set { storedPose = newValue }
    }
    var frontiers: [Frontier]
    var observationQueue: [PoseObservation]
    private var currentFrameSequence: UInt64 = 0
    private var pendingScanObservation: PoseObservation?
    var refreshCount = 0
    var onObservationQueueEmpty: (() -> Void)?

    init(pose: Pose2D?, frontiers: [Frontier], observations: [PoseObservation] = []) {
        self.storedPose = pose
        self.frontiers = frontiers
        self.observationQueue = observations
    }

    var latestObservation: PoseObservation? {
        if !observationQueue.isEmpty {
            let observation = observationQueue.removeFirst()
            currentFrameSequence = max(currentFrameSequence, observation.frameSequence)
            return observation
        }
        if let pendingScanObservation {
            self.pendingScanObservation = nil
            return pendingScanObservation
        }
        onObservationQueueEmpty?()
        return nil
    }

    var frameSequence: UInt64? { currentFrameSequence }

    func publishScanFrame(sessionGeneration: UInt64?) {
        guard let sessionGeneration else { return }
        currentFrameSequence += 1
        pendingScanObservation = PoseObservation(
            pose: storedPose ?? Pose2D(position: .zero, yaw: 0),
            frameSequence: currentFrameSequence,
            timestamp: Double(currentFrameSequence),
            trackingQuality: .normal,
            sessionGeneration: sessionGeneration
        )
    }
    func detectObjects() -> [PerceivedObject] { [] }
    func unproject(normalizedPoint: CGPoint) -> Vec2? { nil }
    func capturedFrameJPEG() -> Data? { nil }
    func explorationFrontiers() -> [Frontier] {
        refreshCount += 1
        return frontiers
    }
}

@MainActor
private final class RoomMissionVoice: RoverVoice {
    var spoken: [String] = []
    func speak(_ text: String) { spoken.append(text) }
    func ask(_ question: String, timeout: TimeInterval) async -> String? { nil }
}

private func pose(x: Double, y: Double = 0) -> Pose2D {
    Pose2D(position: Vec2(x, y), yaw: 0)
}

private func doorwayFrontier(
    x: Double,
    y: Double = 0,
    widthMeters: Double = 1
) -> Frontier {
    Frontier(
        centroid: Vec2(x, y),
        widthMeters: widthMeters,
        cellCount: 8,
        outwardDirection: Vec2(1, 0)
    )
}

private func crossingObservations(generation: UInt64) -> [PoseObservation] {
    [0.4, 0.9, 1.36, 1.38, 1.4].enumerated().map { index, x in
        PoseObservation(
            pose: pose(x: x),
            frameSequence: UInt64(index + 1),
            timestamp: Double(index + 1),
            trackingQuality: .normal,
            sessionGeneration: generation
        )
    }
}

private func reverseCrossingObservations(generation: UInt64) -> [PoseObservation] {
    [1.2, 0.8, 0.6, 0.55, 0.5].enumerated().map { index, x in
        observation(
            x: x,
            sequence: UInt64(index + 20),
            quality: .normal,
            generation: generation
        )
    }
}

private func observation(
    x: Double,
    sequence: UInt64,
    quality: PoseTrackingQuality,
    generation: UInt64
) -> PoseObservation {
    PoseObservation(
        pose: pose(x: x),
        frameSequence: sequence,
        timestamp: Double(sequence),
        trackingQuality: quality,
        sessionGeneration: generation
    )
}
