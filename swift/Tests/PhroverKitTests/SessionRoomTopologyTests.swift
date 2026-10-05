import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class SessionRoomTopologyTests: XCTestCase {
    private let initialPose = Pose2D(position: Vec2(-1, 0), yaw: 0)

    func testSessionStartsOnlyAtExplicitGenerationBoundary() {
        let topology = SessionRoomTopology()

        XCTAssertTrue(topology.refreshCandidates(from: [frontier()], referencePose: initialPose).isEmpty)
        XCTAssertNil(topology.snapshot.sessionGeneration)
        XCTAssertTrue(topology.snapshot.rooms.isEmpty)

        topology.startSession(generation: 41, initialPose: initialPose)

        XCTAssertEqual(topology.snapshot.sessionGeneration, 41)
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertEqual(topology.snapshot.rooms.map(\.id), [RoomID("room_1")])
        XCTAssertEqual(topology.snapshot.rooms.first?.representativePoses, [initialPose])
    }

    func testStablePoseIngestionUpdatesOnlyTheCurrentSessionRoom() {
        let topology = SessionRoomTopology()
        let currentPose = Pose2D(position: Vec2(1, 2), yaw: 0.4)
        topology.startSession(generation: 41, initialPose: initialPose)

        topology.ingestStablePose(currentPose, sessionGeneration: 40)
        topology.ingestStablePose(currentPose, sessionGeneration: 41)

        XCTAssertEqual(
            topology.snapshot.rooms.first?.representativePoses,
            [initialPose, currentPose]
        )
    }

    func testCandidateFilteringIncludesConventionalAndHallwayWidthDirectedFrontiers() {
        let topology = startedTopology()

        let candidates = topology.refreshCandidates(from: [
            frontier(x: 1, y: 0, width: 0.64),
            frontier(x: 2, y: 0, width: 0.65),
            frontier(x: 3, y: 0, width: 2.0),
            frontier(x: 4, y: 0, width: 3.5),
            frontier(x: 5, y: 0, width: 1.0, direction: nil),
        ], referencePose: initialPose)

        XCTAssertEqual(candidates.map(\.id), [
            DoorwayCandidateID("doorway_candidate_1"),
            DoorwayCandidateID("doorway_candidate_2"),
            DoorwayCandidateID("doorway_candidate_3"),
        ])
        XCTAssertEqual(candidates.map(\.widthMeters), [0.65, 2.0, 3.5])
    }

    func testFrontierAdmissionTelemetryExplainsEveryOutcome() {
        var telemetry: [(event: String, fields: [String: String])] = []
        let topology = startedTopology { event, fields in
            telemetry.append((event, fields))
        }

        _ = topology.refreshCandidates(from: [
            frontier(x: 1, width: 0.5),
            frontier(x: 2, width: 2.5),
            frontier(x: 3, width: 1.0, direction: nil),
            Frontier(
                centroid: Vec2(.nan, 0),
                widthMeters: 1.0,
                cellCount: 5,
                outwardDirection: Vec2(1, 0)
            ),
            Frontier(
                centroid: Vec2(4, 0),
                widthMeters: 1.0,
                cellCount: 5,
                outwardDirection: Vec2(.infinity, 0)
            ),
            Frontier(
                centroid: Vec2(.greatestFiniteMagnitude, 0),
                widthMeters: 1.0,
                cellCount: 5,
                outwardDirection: Vec2(1, 0)
            ),
        ], referencePose: initialPose)

        let outcomes = telemetry.filter {
            $0.event == "doorway_frontier_admitted" || $0.event == "doorway_frontier_rejected"
        }
        XCTAssertEqual(outcomes.count, 6)
        XCTAssertEqual(
            outcomes.filter { $0.event == "doorway_frontier_admitted" }.count,
            1
        )
        XCTAssertEqual(Set(outcomes.compactMap { $0.fields["reason"] }), [
            "below_safe_width",
            "missing_direction",
            "invalid_centroid",
            "invalid_direction",
            "invalid_beyond_plane_goal",
        ])
        XCTAssertEqual(
            outcomes.first { $0.event == "doorway_frontier_admitted" }?.fields["width_m"],
            "2.50"
        )
    }

    func testNearbyAlignedCandidateObservationsPreserveIdentity() throws {
        let topology = startedTopology()
        let original = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier(x: 1, y: 1)],
            referencePose: initialPose
        ).first)

        let matched = try XCTUnwrap(topology.refreshCandidates(from: [
            frontier(x: 1.49, y: 1, direction: direction(degrees: 35)),
        ], referencePose: initialPose).first)
        XCTAssertEqual(matched.id, original.id)

        let distinct = try XCTUnwrap(topology.refreshCandidates(from: [
            frontier(x: 2.0, y: 1, direction: direction(degrees: 36)),
        ], referencePose: initialPose).first)
        XCTAssertEqual(distinct.id, DoorwayCandidateID("doorway_candidate_2"))
    }

    func testRankingUsesExplicitAssessmentsAndMissionExclusionsDeterministically() {
        var events: [String] = []
        let topology = startedTopology { event, _ in events.append(event) }
        let candidates = topology.refreshCandidates(from: [
            frontier(x: 1, y: -1, width: 1.0, cells: 8),
            frontier(x: 1, y: 0, width: 1.0, cells: 8),
            frontier(x: 1, y: 1, width: 1.0, cells: 8),
        ], referencePose: initialPose)
        let assessments = [
            DoorwayCandidateAssessment(candidateID: candidates[0].id,
                                        isReachable: false,
                                        beyondPlaneGoal: Vec2(1.5, -1),
                                        pathDistance: 1),
            DoorwayCandidateAssessment(candidateID: candidates[1].id,
                                        isReachable: true,
                                        beyondPlaneGoal: Vec2(1.5, 0),
                                        pathDistance: 4),
            DoorwayCandidateAssessment(candidateID: candidates[2].id,
                                        isReachable: true,
                                        beyondPlaneGoal: Vec2(1.5, 1),
                                        pathDistance: 2),
        ]

        let ranked = topology.rankedCandidates(
            assessments: assessments,
            visualBoosts: [candidates[1].id: 1, candidates[2].id: 0],
            excluding: [candidates[0].id]
        )

        XCTAssertEqual(ranked.map(\.candidate.id), [candidates[1].id, candidates[2].id])
        XCTAssertEqual(ranked.map(\.assessment.pathDistance), [4, 2],
                       "visual evidence breaks equal-geometry ties before path length")
        XCTAssertEqual(topology.snapshot.candidates.count, 3,
                       "mission exclusions must not mutate session candidates")
        XCTAssertEqual(events.filter { $0 == "doorway_candidate_ranked" }.count, 2)
    }

    func testHallwayWidthCandidateStillRequiresReachability() throws {
        let topology = startedTopology()
        let candidate = try XCTUnwrap(topology.refreshCandidates(from: [
            frontier(x: 1, width: 3.5),
        ], referencePose: initialPose).first)

        let ranked = topology.rankedCandidates(assessments: [
            DoorwayCandidateAssessment(
                candidateID: candidate.id,
                isReachable: false,
                beyondPlaneGoal: candidate.beyondPlaneGoal(),
                pathDistance: 1
            ),
        ])

        XCTAssertEqual(ranked.map(\.candidate.id), [candidate.id])
        XCTAssertEqual(ranked.first?.assessment.isReachable, false)
    }

    func testCandidateDirectionIsOrientedAwayFromCurrentRoom() throws {
        let topology = SessionRoomTopology()
        let roomPose = Pose2D(position: Vec2(2, 0), yaw: .pi)
        topology.startSession(generation: 41, initialPose: roomPose)

        let candidate = try XCTUnwrap(topology.refreshCandidates(from: [
            frontier(x: 1, y: 0, direction: Vec2(1, 0)),
        ], referencePose: roomPose).first)

        XCTAssertEqual(candidate.outwardDirection.x, -1, accuracy: 1e-9)
        XCTAssertEqual(candidate.outwardDirection.y, 0, accuracy: 1e-9)
        XCTAssertEqual(candidate.beyondPlaneGoal().x, 0.4, accuracy: 1e-9)
        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: roomPose), .started)
    }

    func testCandidateDirectionUsesCurrentReferencePoseAndReportsCorrectionGeometry() throws {
        var telemetry: [(event: String, fields: [String: String])] = []
        let topology = SessionRoomTopology { telemetry.append(($0, $1)) }
        topology.startSession(
            generation: 41,
            initialPose: Pose2D(position: Vec2(2, -2), yaw: 0)
        )
        let referencePose = Pose2D(position: Vec2(-0.01, 0.06), yaw: 0)

        let candidate = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier(x: 0.10, y: -1.66, direction: Vec2(1.00, 0.09))],
            referencePose: referencePose
        ).first)

        XCTAssertLessThan(candidate.outwardDirection.x, 0)
        XCTAssertLessThan(candidate.beyondPlaneGoal().x, candidate.planePoint.x)
        let fields = try XCTUnwrap(
            telemetry.first { $0.event == "doorway_frontier_admitted" }?.fields
        )
        XCTAssertEqual(fields["raw_direction_x"], "1.00")
        XCTAssertEqual(fields["raw_direction_y"], "0.09")
        XCTAssertEqual(fields["reference_x"], "-0.01")
        XCTAssertEqual(fields["reference_y"], "0.06")
        XCTAssertNotNil(fields["signed_distance"])
        XCTAssertEqual(fields["direction_flipped"], "true")
        XCTAssertEqual(fields["direction_x"], "-1.00")
        XCTAssertEqual(fields["direction_y"], "-0.09")
    }

    func testTransitionStartupReturnsTypedOutcomesAndCorrectionDoesNotStart() throws {
        let missingRoom = SessionRoomTopology()
        XCTAssertEqual(
            missingRoom.beginTransition(
                candidateID: DoorwayCandidateID("missing"),
                approachPose: initialPose
            ),
            .rejected(.missingCurrentRoom)
        )

        var telemetry: [(event: String, fields: [String: String])] = []
        let topology = startedTopology { telemetry.append(($0, $1)) }
        XCTAssertEqual(
            topology.beginTransition(
                candidateID: DoorwayCandidateID("missing"),
                approachPose: initialPose
            ),
            .rejected(.missingCandidate)
        )
        let candidate = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier()],
            referencePose: initialPose
        ).first)
        XCTAssertEqual(
            topology.beginTransition(
                candidateID: candidate.id,
                approachPose: Pose2D(position: candidate.planePoint, yaw: 0)
            ),
            .rejected(.invalidApproachSide)
        )
        XCTAssertEqual(
            topology.beginTransition(
                candidateID: candidate.id,
                approachPose: Pose2D(position: Vec2(1, 0), yaw: 0)
            ),
            .correctedOrientation(DoorwayCandidate(
                id: candidate.id,
                planePoint: candidate.planePoint,
                outwardDirection: candidate.outwardDirection * -1,
                widthMeters: candidate.widthMeters,
                frontierCellCount: candidate.frontierCellCount
            ))
        )
        XCTAssertNil(topology.snapshot.pendingTransitionCandidateID)
        let correctionFields = try XCTUnwrap(
            telemetry.last { $0.event == "room_transition_orientation_corrected" }?.fields
        )
        XCTAssertEqual(correctionFields["raw_direction_x"], "1.00")
        XCTAssertEqual(correctionFields["raw_direction_y"], "0.00")
        XCTAssertEqual(correctionFields["corrected_direction_x"], "-1.00")
        XCTAssertEqual(correctionFields["corrected_direction_y"], "-0.00")
        XCTAssertEqual(correctionFields["reference_x"], "1.00")
        XCTAssertEqual(correctionFields["reference_y"], "0.00")
        XCTAssertEqual(correctionFields["signed_distance"], "1.00")
        XCTAssertEqual(correctionFields["direction_flipped"], "true")

        let corrected = try XCTUnwrap(topology.snapshot.candidates.first)
        XCTAssertEqual(
            topology.beginTransition(
                candidateID: corrected.id,
                approachPose: Pose2D(position: Vec2(1, 0), yaw: 0)
            ),
            .started
        )
        XCTAssertEqual(
            topology.beginTransition(
                candidateID: corrected.id,
                approachPose: Pose2D(position: Vec2(1, 0), yaw: 0)
            ),
            .rejected(.alreadyActive)
        )

        let invalid = startedTopology()
        let invalidCandidate = try XCTUnwrap(invalid.refreshCandidates(
            from: [frontier()],
            referencePose: initialPose
        ).first)
        XCTAssertEqual(
            invalid.beginTransition(
                candidateID: invalidCandidate.id,
                approachPose: Pose2D(position: Vec2(.nan, 0), yaw: 0)
            ),
            .rejected(.invalidGeometry)
        )
    }

    func testResetClearsGraphPendingStateAndDeterministicIDs() throws {
        var events: [String] = []
        let topology = startedTopology { event, _ in events.append(event) }
        let candidate = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier()], referencePose: initialPose
        ).first)
        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: initialPose), .started)

        topology.reset(forSessionGeneration: 7)

        XCTAssertEqual(topology.snapshot.sessionGeneration, 7)
        XCTAssertNil(topology.snapshot.currentRoomID)
        XCTAssertNil(topology.snapshot.pendingTransitionCandidateID)
        XCTAssertTrue(topology.snapshot.rooms.isEmpty)
        XCTAssertTrue(topology.snapshot.doorways.isEmpty)
        XCTAssertTrue(topology.snapshot.candidates.isEmpty)

        topology.startSession(generation: 7, initialPose: initialPose)
        let restarted = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier()], referencePose: initialPose
        ).first)
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertEqual(restarted.id, DoorwayCandidateID("doorway_candidate_1"))
        XCTAssertTrue(events.contains("room_session_reset"))
    }

    func testTransitionRequiresThreeConsecutiveFreshTrackedPosesBeyondPlane() throws {
        let (topology, candidate) = try topologyWithCandidate()
        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: initialPose), .started)

        XCTAssertEqual(topology.observeTransition(observation(x: -0.4, sequence: 1)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.1, sequence: 2)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.35, sequence: 3)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.35, sequence: 3)), .ignored)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.35,
                                                              sequence: 4,
                                                              timestamp: 3)), .ignored)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.35,
                                                              sequence: 5,
                                                              trackingNormal: false)), .ignored)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.35,
                                                              sequence: 6,
                                                              generation: 2)), .ignored)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.34, sequence: 7)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.35, sequence: 8)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.36, sequence: 9)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 0.37, sequence: 10)),
                       .readyForConfirmation)

        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertTrue(topology.snapshot.doorways.isEmpty,
                      "observation readiness must not mutate the graph")
    }

    func testLateralMotionAndSinglePoseJumpCannotConfirmCrossing() throws {
        let (topology, candidate) = try topologyWithCandidate()
        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: initialPose), .started)

        XCTAssertEqual(topology.observeTransition(observation(x: -0.4, y: 0.4, sequence: 1)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: -0.4, y: 0.8, sequence: 2)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 1.5, y: 0.8, sequence: 3)), .ignored)
        XCTAssertEqual(topology.observeTransition(observation(x: 1.5, y: 0.8, sequence: 4)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 1.5, y: 0.8, sequence: 5)), .observing)
        XCTAssertEqual(topology.observeTransition(observation(x: 1.5, y: 0.8, sequence: 6)), .observing,
                       "a discontinuity must require renewed approach-side evidence")
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
    }

    func testConfirmAtomicallyCreatesFirstRoomAndDoorwayAfterReadiness() throws {
        var events: [String] = []
        let (topology, candidate) = try topologyWithCandidate { event, _ in events.append(event) }
        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: initialPose), .started)
        makeReady(topology)

        XCTAssertEqual(topology.snapshot.rooms.map(\.id), [RoomID("room_1")])
        XCTAssertEqual(topology.confirmTransition(), RoomID("room_2"))

        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_2"))
        XCTAssertEqual(topology.snapshot.rooms.map(\.id), [RoomID("room_1"), RoomID("room_2")])
        XCTAssertEqual(topology.snapshot.doorways.map(\.id), [DoorwayID("doorway_1")])
        XCTAssertEqual(topology.snapshot.doorways.first?.candidateID, candidate.id)
        XCTAssertNil(topology.snapshot.pendingTransitionCandidateID)
        XCTAssertEqual(events, [
            "room_session_started",
            "doorway_frontier_admitted",
            "room_transition_started",
            "doorway_crossed",
            "room_transition_completed",
        ])
    }

    func testKnownIncidentDoorwayIsSelectableWithoutFrontierAndTraversesInReverse() throws {
        let (topology, candidate) = try topologyWithCandidate()
        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: initialPose), .started)
        makeReady(topology)
        XCTAssertEqual(topology.confirmTransition(), RoomID("room_2"))

        let reversePose = Pose2D(position: Vec2(0.5, 0), yaw: .pi)
        let reverse = try XCTUnwrap(topology.refreshCandidates(
            from: [], referencePose: reversePose
        ).first)
        XCTAssertEqual(reverse.id, candidate.id)
        XCTAssertEqual(reverse.doorwayID, DoorwayID("doorway_1"))
        XCTAssertEqual(reverse.oppositeRoomID, RoomID("room_1"))
        XCTAssertEqual(reverse.outwardDirection, Vec2(-1, 0))
        let duplicateObservation = try XCTUnwrap(topology.refreshCandidates(from: [
            frontier(x: 0.1, direction: Vec2(-1, 0)),
        ], referencePose: reversePose).first)
        XCTAssertEqual(duplicateObservation.id, candidate.id)
        XCTAssertEqual(duplicateObservation.doorwayID, DoorwayID("doorway_1"))
        let ranked = topology.rankedCandidates(assessments: [
            DoorwayCandidateAssessment(candidateID: reverse.id,
                                        isReachable: true,
                                        beyondPlaneGoal: reverse.beyondPlaneGoal(),
                                        pathDistance: 1),
        ])
        XCTAssertEqual(ranked.map(\.candidate.id), [candidate.id])

        XCTAssertEqual(topology.beginTransition(
            candidateID: reverse.id,
            approachPose: reversePose
        ), .started)
        makeReady(topology, positions: [0.2, -0.35, -0.36, -0.37])
        XCTAssertEqual(topology.confirmTransition(), RoomID("room_1"))
        XCTAssertEqual(topology.snapshot.rooms.count, 2)
        XCTAssertEqual(topology.snapshot.doorways.count, 1)
    }

    func testShortestDoorwayPathReturnsForwardAndReverseStepsWithoutCycles() throws {
        let topology = startedTopology()
        let room1 = try XCTUnwrap(topology.snapshot.currentRoomID)
        let firstCandidate = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier()],
            referencePose: initialPose
        ).first)
        XCTAssertEqual(
            topology.beginTransition(candidateID: firstCandidate.id, approachPose: initialPose),
            .started
        )
        makeReady(topology)
        let room2 = try XCTUnwrap(topology.confirmTransition())

        let room2Pose = Pose2D(position: Vec2(0.37, 0), yaw: 0)
        let secondCandidate = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier(x: 1)],
            referencePose: room2Pose
        ).first { $0.doorwayID == nil })
        XCTAssertEqual(
            topology.beginTransition(candidateID: secondCandidate.id, approachPose: room2Pose),
            .started
        )
        makeReady(topology, positions: [0.65, 1.35, 1.36, 1.37])
        let room3 = try XCTUnwrap(topology.confirmTransition())

        let outward = topology.shortestDoorwayPath(from: room1, to: room3)
        XCTAssertEqual(outward?.map(\.doorwayID), [DoorwayID("doorway_1"), DoorwayID("doorway_2")])
        XCTAssertEqual(outward?.map(\.fromRoomID), [room1, room2])
        XCTAssertEqual(outward?.map(\.toRoomID), [room2, room3])

        let returning = topology.shortestDoorwayPath(from: room3, to: room1)
        XCTAssertEqual(returning, outward?.reversed().map(\.reversed))
        XCTAssertEqual(topology.shortestDoorwayPath(from: room2, to: room2), [])
        XCTAssertNil(topology.shortestDoorwayPath(from: room1, to: RoomID("unknown_room")))

        topology.reset(forSessionGeneration: 2)
        topology.startSession(generation: 2, initialPose: initialPose)
        XCTAssertNil(topology.shortestDoorwayPath(from: room3, to: room1),
                     "routes must not survive a room-mapping session generation change")
    }

    func testRejectAndAbandonClearPendingTransitionWithoutGraphMutation() throws {
        var events: [String] = []
        let (topology, candidate) = try topologyWithCandidate { event, _ in events.append(event) }
        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: initialPose), .started)
        makeReady(topology)

        topology.rejectTransition(reason: "blocked")

        XCTAssertNil(topology.confirmTransition())
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertTrue(topology.snapshot.doorways.isEmpty)
        XCTAssertTrue(events.contains("room_transition_rejected"))
        XCTAssertFalse(events.contains("doorway_crossed"))
        XCTAssertFalse(events.contains("room_transition_completed"))

        XCTAssertEqual(topology.beginTransition(candidateID: candidate.id, approachPose: initialPose), .started)
        topology.abandonTransition()
        XCTAssertNil(topology.snapshot.pendingTransitionCandidateID)
        XCTAssertNil(topology.confirmTransition())
        XCTAssertEqual(topology.snapshot.rooms.count, 1)
    }

    private func startedTopology(
        telemetry: @escaping RoomTopologyTelemetrySink = { _, _ in }
    ) -> SessionRoomTopology {
        let topology = SessionRoomTopology(telemetry: telemetry)
        topology.startSession(generation: 1, initialPose: initialPose)
        return topology
    }

    private func topologyWithCandidate(
        telemetry: @escaping RoomTopologyTelemetrySink = { _, _ in }
    ) throws -> (SessionRoomTopology, DoorwayCandidate) {
        let topology = startedTopology(telemetry: telemetry)
        let candidate = try XCTUnwrap(topology.refreshCandidates(
            from: [frontier()], referencePose: initialPose
        ).first)
        return (topology, candidate)
    }

    private func observation(
        x: Double,
        y: Double = 0,
        sequence: UInt64,
        timestamp: TimeInterval? = nil,
        generation: UInt64 = 1,
        trackingNormal: Bool = true
    ) -> TransitionObservation {
        TransitionObservation(
            pose: Pose2D(position: Vec2(x, y), yaw: 0),
            frameSequence: sequence,
            timestamp: timestamp ?? TimeInterval(sequence),
            isTrackingNormal: trackingNormal,
            sessionGeneration: generation
        )
    }

    private func makeReady(
        _ topology: SessionRoomTopology,
        positions: [Double] = [-0.4, 0.35, 0.36, 0.37]
    ) {
        var result = TransitionObservationResult.observing
        for (index, x) in positions.enumerated() {
            result = topology.observeTransition(observation(x: x, sequence: UInt64(index + 1)))
        }
        XCTAssertEqual(result, .readyForConfirmation)
    }

    private func frontier(
        x: Double = 0,
        y: Double = 0,
        width: Double = 1,
        cells: Int = 6,
        direction: Vec2? = Vec2(1, 0)
    ) -> Frontier {
        Frontier(centroid: Vec2(x, y),
                 widthMeters: width,
                 cellCount: cells,
                 outwardDirection: direction)
    }

    private func direction(degrees: Double) -> Vec2 {
        let radians = degrees * .pi / 180
        return Vec2(cos(radians), sin(radians))
    }
}
