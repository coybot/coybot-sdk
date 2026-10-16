import XCTest
import CoreGraphics
import RoverNav
@testable import PhroverKit

/// Exercises `MissionAgent`'s decision loop against scripted fakes for `RoverBrain`,
/// `RoverMotion`, `RoverPerception`, and `RoverVoice` — no live ARKit session, no bundled
/// CoreML model, no network. This is the "does the loop do the right thing given a
/// decision" layer; live grounding accuracy and on-device Foundation Model behavior can
/// only be verified on a real device (see rover/README.md status notes).
@MainActor
final class MissionAgentTests: XCTestCase {
    func testPlainFollowRequestsAreRecognisedWithoutABrain() {
        let cases: [(String, String?)] = [
            ("follow me", "person"),
            ("Follow me, please.", "person"),
            ("hey rover follow me to the gate", "person"),
            ("come with me", "person"),
            ("tag along", "person"),
            ("follow the guy with the hat", "the guy with the hat"),
            ("keep up with that woman", "that woman"),
            ("follow the hallway", nil),
            ("follow me and then come back", nil),
            ("go to the chair", nil),
            ("is anyone there", nil),
        ]
        for (utterance, expected) in cases {
            XCTAssertEqual(MissionAgent.directFollowQuery(utterance), expected, utterance)
        }
    }

    /// iPhone 13 mini offline: no Apple Intelligence, no cloud — no brain at all.
    func testFollowMeWorksWithNoBrainAtAll() async {
        let perception = FakePerception()
        perception.objects = [PerceivedObject(label: "person", confidence: 0.95,
                                              normalizedPoint: CGPoint(x: 0.5, y: 0.5))]
        let follower = FakeFollower()
        let voice = FakeVoice()
        let agent = MissionAgent(motion: FakeMotion(), perception: perception, voice: voice,
                                 follower: follower) { nil }

        await agent.handle("follow me")

        XCTAssertEqual(follower.followCalls.map(\.query), ["person"])
        XCTAssertFalse(voice.spoken.contains { $0.contains("think") }, "\(voice.spoken)")
        XCTAssertTrue(voice.spoken.contains("lost them"), "the follow's end is still reported")
    }

    /// A brain that can't answer (Apple Intelligence unavailable, cloud down) must not end
    /// a follow that is going fine.
    func testAFollowKeepsGoingWhenTheBrainFails() async {
        let perception = FakePerception()
        perception.objects = [PerceivedObject(label: "person", confidence: 0.95,
                                              normalizedPoint: CGPoint(x: 0.5, y: 0.5))]
        let follower = FakeFollower()
        follower.endsAfter = 0.5
        let voice = FakeVoice()
        let brain = ThrowingBrain(error: RoverBrainError.unavailable)
        let agent = MissionAgent(motion: FakeMotion(), perception: perception, voice: voice,
                                 follower: follower, brainErrorLogger: { _, _ in }) { brain }

        await agent.handle("follow me")

        XCTAssertEqual(follower.followCalls.count, 1, "a re-stated follow must not restart the lock")
        XCTAssertEqual(follower.stopCallCount, 0)
        XCTAssertFalse(voice.spoken.contains { $0.contains("trouble thinking") }, "\(voice.spoken)")
    }

    func testFollowDecisionStartsTheFollower() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [PerceivedObject(label: "person", confidence: 0.95,
                                              normalizedPoint: CGPoint(x: 0.5, y: 0.5))]
        let follower = FakeFollower()
        let brain = FakeBrain(script: [.follow(.visualQuery("person")), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: FakeVoice(),
                                 follower: follower) { brain }

        await agent.handle("follow me")

        XCTAssertEqual(follower.followCalls.map(\.query), ["person"])
        XCTAssertTrue(brain.seenContexts.last?.followState != nil,
                      "the brain's look-in during a follow must be told a follow is running")
    }

    /// A brain that decides to drive somewhere mid-follow must end the follow first, or
    /// the follower and the navigator both write wheel commands.
    func testNavigatingDuringAFollowStopsTheFollowerFirst() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [PerceivedObject(label: "person", confidence: 0.95,
                                              normalizedPoint: CGPoint(x: 0.5, y: 0.5))]
        var order: [String] = []
        let follower = FakeFollower()
        follower.onEvent = { order.append($0) }
        motion.onNavigate = { _ in order.append("navigate") }
        let brain = FakeBrain(script: [.follow(.visualQuery("person")),
                                       .navigate(.imagePoint(CGPoint(x: 0.5, y: 0.5))),
                                       .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: FakeVoice(),
                                 follower: follower) { brain }

        await agent.handle("follow me, then go to the chair")

        XCTAssertEqual(order, ["follow", "stopFollowing", "navigate"])
    }

    func testLookingAroundDuringAFollowStopsTheFollowerFirst() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [PerceivedObject(label: "person", confidence: 0.95,
                                              normalizedPoint: CGPoint(x: 0.5, y: 0.5))]
        let follower = FakeFollower()
        let brain = FakeBrain(script: [.follow(.visualQuery("person")), .lookAround(angle: 1), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: FakeVoice(),
                                 follower: follower) { brain }

        await agent.handle("follow me")

        XCTAssertEqual(follower.stopCallCount, 1)
        XCTAssertEqual(motion.rotateCalls, [1])
    }

    func testVisualMatchPrefersTheLabelTheQueryDescribesMostSpecifically() {
        let objects = [
            PerceivedObject(label: "blue_toolbox", confidence: 0.97, normalizedPoint: CGPoint(x: 0.2, y: 0.5)),
            PerceivedObject(label: "red_toolbox", confidence: 0.91, normalizedPoint: CGPoint(x: 0.8, y: 0.5)),
        ]

        let red = MissionAgent.bestVisualTargetMatch(query: "the red toolbox", objects: objects, minimumConfidence: 0.5)
        let any = MissionAgent.bestVisualTargetMatch(query: "the toolbox", objects: objects, minimumConfidence: 0.5)

        XCTAssertEqual(red?.label, "red_toolbox")
        XCTAssertEqual(any?.label, "blue_toolbox", "no attribute named: the most confident instance")
    }

    func testSpokenTargetDoesNotDoubleTheArticle() {
        XCTAssertEqual(MissionAgent.spokenTarget("the person with the hat"), "the person with the hat")
        XCTAssertEqual(MissionAgent.spokenTarget("person"), "the person")
    }

    func testEverydayWordsForAHumanMatchThePersonLabel() {
        for query in ["the guy", "that woman", "the kid in red", "someone"] {
            XCTAssertTrue(MissionAgent.visualQueryMatches(query: query, label: "person"), query)
        }
        XCTAssertFalse(MissionAgent.visualQueryMatches(query: "the guy", label: "chair"))
    }

    func testRepeatedIdenticalCommandsReceiveDistinctStableIDs() async {
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: FakeMotion(),
            perception: FakePerception(),
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { FakeBrain(script: [.done]) }
        )

        await agent.handle("look around")
        await agent.handle("look around")

        XCTAssertEqual(statuses.map(\.id), [1, 1, 1, 2, 2, 2])
        XCTAssertEqual(statuses.filter(\.isTerminal).count, 2)
    }

    func testGeneralMissionPublishesSuccessAndMissingBrainFailure() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        var statuses: [MissionCommandStatus] = []
        let successful = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { FakeBrain(script: [.done]) }
        )

        await successful.handle("look around")
        XCTAssertEqual(
            statuses.last,
            .succeeded(id: 1, command: "look around", message: "Command completed.")
        )

        let failed = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { nil }
        )
        await failed.handle("find the chair")
        XCTAssertEqual(
            statuses.last,
            .failed(id: 1, command: "find the chair", message: "Sorry, I can’t think right now.")
        )
    }
    func testGroundsVisibleTargetAndFinishes() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.imagePoint(CGPoint(x: 0.5, y: 0.5))), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertEqual(agent.phase, .idle)
    }

    func testVisualTargetMissionStopsAfterArrival() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.34, y: 0.5))
        ]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [
            .navigate(.visualQuery("the refrigerator")),
            .navigate(.visualQuery("the refrigerator")),
        ])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertEqual(brain.seenContexts.count, 1)
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testMissionWaitsThroughPlanningBeforeRequestingAnotherDecision() async {
        let motion = FakeMotion()
        motion.beginsNavigationInPlanning = true
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.34, y: 0.5))
        ]
        let brain = FakeBrain(script: [
            .navigate(.visualQuery("the refrigerator")),
            .navigate(.visualQuery("the refrigerator")),
        ])
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: { brain }
        )

        await agent.handle("go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls.count, 1)
        XCTAssertEqual(brain.seenContexts.count, 1)
        XCTAssertEqual(motion.state, .arrived)
    }

    func testMissionCancellationDuringPlanningStopsMotion() async {
        let motion = FakeMotion()
        motion.beginsNavigationInPlanning = true
        motion.planningDelay = 1
        let navigationStarted = expectation(description: "navigation entered planning")
        motion.onNavigate = { _ in navigationStarted.fulfill() }
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.34, y: 0.5))
        ]
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: { FakeBrain(script: [.navigate(.visualQuery("the refrigerator"))]) }
        )
        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }

        await fulfillment(of: [navigationStarted], timeout: 1)
        mission.cancel()
        await mission.value

        XCTAssertEqual(motion.cancelCallCount, 1)
        XCTAssertEqual(motion.navigateCalls.count, 1)
    }

    func testSessionChangeDuringPlanningStopsMotion() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 1,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.beginsNavigationInPlanning = true
        motion.planningDelay = 1
        let navigationStarted = expectation(description: "navigation entered planning")
        motion.onNavigate = { _ in navigationStarted.fulfill() }
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.34, y: 0.5))
        ]
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            roomTopology: topology,
            currentBrain: { FakeBrain(script: [.navigate(.visualQuery("the refrigerator"))]) }
        )
        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }

        await fulfillment(of: [navigationStarted], timeout: 1)
        topology.reset(forSessionGeneration: 2)
        await mission.value

        XCTAssertGreaterThanOrEqual(motion.stopAndWaitCallCount, 1)
        XCTAssertEqual(motion.navigateCalls.count, 1)
    }

    func testVisualTargetReturnMissionUsesCurrentCommandStartPose() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.34, y: 0.5))
        ]
        motion.onNavigate = { _ in
            perception.pose = Pose2D(position: motion.navigateCalls.last!, yaw: 0)
        }
        let brain = FakeBrain(script: [
            .navigate(.visualQuery("the refrigerator")),
            .navigate(.visualQuery("the refrigerator")),
            .navigate(.visualQuery("the green refrigerator")),
        ])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: FakeVoice(),
                                 currentBrain: { brain })

        perception.unprojectResult = Vec2(1, 2)
        await agent.handle("go to refrigerator")

        let secondMissionStart = perception.pose!.position
        perception.unprojectResult = Vec2(4, 5)
        await agent.handle("go to refrigerator and come back")

        XCTAssertEqual(motion.navigateCalls, [Vec2(1, 2), Vec2(4, 5), secondMissionStart])
        XCTAssertEqual(brain.seenContexts.count, 2,
                       "the return leg should not ask the brain to search for the refrigerator again")
        XCTAssertEqual(brain.seenContexts[1].memory.missionStartPose?.position, secondMissionStart)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testVisualTargetReturnMissionTurnsTowardStartBeforeNavigatingBack() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(1, 0), yaw: 0)
        }
        let brain = FakeBrain(script: [.navigate(.visualQuery("the refrigerator"))])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: FakeVoice(),
                                 currentBrain: { brain })

        await agent.handle("go to refrigerator and come back")

        XCTAssertEqual(motion.navigateCalls, [Vec2(1, 0), .zero])
        XCTAssertFalse(motion.scanRotateCalls.isEmpty)
        XCTAssertEqual(motion.scanRotateCalls.reduce(0, +), .pi, accuracy: 0.001)
        XCTAssertTrue(motion.scanRotateCalls.allSatisfy { abs($0) <= .pi / 6 + 0.001 })
    }

    func testVisualTargetReturnMissionRetriesAfterBlockedHeading() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.navigateOutcomes = [
            .arrived,
            .failed("Obstacle ahead at 0.40 m."),
            .arrived,
        ]
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(1, 0), yaw: .pi)
        }
        let brain = FakeBrain(script: [.navigate(.visualQuery("the refrigerator"))])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: FakeVoice(),
                                 currentBrain: { brain })

        await agent.handle("go to refrigerator and come back")

        XCTAssertEqual(motion.navigateCalls, [Vec2(1, 0), .zero, .zero])
        XCTAssertEqual(motion.scanRotateCalls, [.pi / 6])
        XCTAssertEqual(agent.plan, "Primary target reached; returned to mission start.")
    }

    func testReturnToVisualTargetDoesNotCreateRoundTrip() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.34, y: 0.5))
        ]
        let brain = FakeBrain(script: [
            .navigate(.visualQuery("the refrigerator")),
            .done,
        ])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: FakeVoice(),
                                 currentBrain: { brain })

        await agent.handle("return to refrigerator")

        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertEqual(brain.seenContexts.count, 1)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testNewCommandClearsPlanAndRecentActionsButPreservesRememberedObjects() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "chair",
                            confidence: 0.96,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        let brain = OutputBrain(outputs: [
            BrainOutput(decision: .say("I see it."), updatedPlan: "old mission plan"),
            BrainOutput(decision: .done),
            BrainOutput(decision: .done),
        ])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: FakeVoice(),
                                 currentBrain: { brain })

        await agent.handle("remember the chair")
        XCTAssertEqual(agent.plan, "old mission plan")

        perception.pose = Pose2D(position: Vec2(3, 4), yaw: 0)
        await agent.handle("new mission")

        let secondMissionContext = brain.seenContexts[2]
        XCTAssertNil(secondMissionContext.plan)
        XCTAssertTrue(secondMissionContext.recentActions.isEmpty)
        XCTAssertEqual(secondMissionContext.memory.missionStartPose?.position, Vec2(3, 4))
        XCTAssertEqual(secondMissionContext.memory.currentMissionTurns.map(\.utterance),
                       ["new mission"])
        XCTAssertTrue(secondMissionContext.memory.rememberedObjects.contains { $0.label == "chair" })
    }

    func testVisualTargetNavigationUsesThirtyCentimeterStopDistance() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "chair",
                            confidence: 0.96,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.navigateStopClearances, [0.30])
    }

    func testStalledVisualNavigationScansAndResumesWithFreshTarget() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.failed("Navigation stalled."), .arrived]
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.34, y: 0.5))
        ]
        motion.onNavigate = { callCount in
            if callCount == 1 {
                perception.objects = [
                    PerceivedObject(label: "book",
                                    confidence: 0.99,
                                    normalizedPoint: CGPoint(x: 0.5, y: 0.5))
                ]
            }
        }
        motion.onRotate = { _ in
            perception.objects = [
                PerceivedObject(label: "refrigerator",
                                confidence: 0.98,
                                normalizedPoint: CGPoint(x: 0.52, y: 0.5))
            ]
        }
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("the refrigerator")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 currentBrain: { brain })

        await agent.handle("go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult, perception.unprojectResult])
        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testBlockedVisualNavigationSlowlyScansUntilConfidentTargetIsReacquired() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.failed("Obstacle ahead at 0.42 m."), .arrived]
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.25, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 2)
        motion.onNavigate = { callCount in
            guard callCount == 1 else { return }
            perception.objects = [
                PerceivedObject(label: "refrigerator",
                                confidence: 0.89,
                                normalizedPoint: CGPoint(x: 0.5, y: 0.5))
            ]
            perception.unprojectResult = Vec2(3, 4)
        }
        motion.onScanRotate = { _ in
            perception.objects = [
                PerceivedObject(label: "refrigerator",
                                confidence: 0.95,
                                normalizedPoint: CGPoint(x: 0.72, y: 0.5))
            ]
        }
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("the refrigerator")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 currentBrain: { brain })

        await agent.handle("go to the refrigerator")

        XCTAssertEqual(motion.scanRotateCalls, [.pi / 6])
        XCTAssertEqual(motion.navigateCalls, [Vec2(1, 2), Vec2(3, 4)])
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testVisualTargetScanWaitsForCameraFrameCapturedAfterTurn() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.failed("Obstacle ahead at 0.42 m."), .arrived]
        let perception = FakePerception()
        perception.frameSequence = 10
        perception.objects = [
            PerceivedObject(label: "chair",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        motion.onNavigate = { callCount in
            guard callCount == 1 else { return }
            perception.objects = [
                PerceivedObject(label: "chair",
                                confidence: 0.89,
                                normalizedPoint: CGPoint(x: 0.5, y: 0.5))
            ]
        }
        motion.onScanRotate = { _ in
            perception.objects = [
                PerceivedObject(label: "chair",
                                confidence: 0.96,
                                normalizedPoint: CGPoint(x: 0.7, y: 0.5))
            ]
            Task { @MainActor in
                try? await Task.sleep(for: .milliseconds(20))
                perception.frameSequence = 11
            }
        }
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: FakeVoice(),
                                 visualTargetScanDelay: 0.1,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(perception.unprojectFrameSequences.last, 11)
        XCTAssertEqual(motion.navigateCalls.count, 2)
    }

    func testLooksAroundWhenNothingVisible() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.lookAround(angle: .pi), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.rotateCalls, [.pi])
    }

    func testExploresOpeningWhenVisualTargetIsNotCurrentlyDetected() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "book",
                            confidence: 0.95,
                            normalizedPoint: CGPoint(x: 0.4, y: 0.6))
        ]
        perception.frontiers = [Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5)]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 1,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(motion.navigateCalls, [Vec2(2, 1)])
        XCTAssertTrue(voice.spoken.isEmpty)
    }

    func testStopsMissionWhenRoverCommandFails() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("Rover command failed: The request timed out.")
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.worldPoint(Vec2(4, 5))), .explore(candidateId: "opening_1")])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go to the black chair in the other room")

        XCTAssertEqual(motion.navigateCalls, [Vec2(4, 5)])
        XCTAssertEqual(brain.seenContexts.count, 1)
        XCTAssertEqual(voice.spoken, ["Rover command failed: The request timed out."])
        XCTAssertTrue(agent.phase == .idle)
    }

    func testRotatesAndContinuesWhenHeadingIsBlockedByObstacle() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("Obstacle ahead at 0.36 m.")
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.worldPoint(Vec2(4, 5))), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go to the black chair in the other room")

        XCTAssertEqual(motion.navigateCalls, [Vec2(4, 5)])
        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(brain.seenContexts.count, 2)
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testBlockedHeadingRecoveryCancelsScanTurnThatDoesNotSettle() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("Obstacle ahead at 0.20 m.")
        motion.rotateNeverCompletes = true
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.worldPoint(Vec2(4, 5))), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 blockedHeadingRecoveryTimeout: 0.03,
                                 currentBrain: { brain })

        await agent.handle("go to the table")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(motion.cancelCallCount, 1)
        XCTAssertEqual(brain.seenContexts.count, 2)
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testBlockedHeadingRecoveryContinuesAsSoonAsScanSettles() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("Obstacle ahead at 0.20 m.")
        motion.rotateDelay = 0.02
        let brain = FakeBrain(script: [.navigate(.worldPoint(Vec2(4, 5))), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: FakePerception(),
                                 voice: FakeVoice(),
                                 blockedHeadingRecoveryTimeout: 0.5,
                                 currentBrain: { brain })
        let clock = ContinuousClock()

        let elapsed = await clock.measure {
            await agent.handle("go to the table")
        }

        XCTAssertLessThan(elapsed, .milliseconds(250))
        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(brain.seenContexts.count, 2)
    }

    func testNewMissionClearsStaleMotionFailureBeforeBrainContext() async {
        let motion = FakeMotion()
        motion.state = .failed("ARKit pose did not stabilize after scan turn.")
        motion.stopAndWaitDelay = 0.02
        let brain = FakeBrain(script: [.navigate(.worldPoint(Vec2(1, 2))), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: FakePerception(),
                                 voice: FakeVoice(),
                                 currentBrain: { brain })

        await agent.handle("go to the table")

        XCTAssertEqual(brain.seenContexts.first?.navState, .idle)
        XCTAssertEqual(motion.stopAndWaitCallCount, 1)
        XCTAssertEqual(motion.lifecycleEvents.prefix(3), ["stopStarted", "stopFinished", "navigate"])
    }

    func testMarksBlockedExplorationCandidateVisitedBeforeNextDecision() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("Obstacle ahead at 0.26 m.")
        let perception = FakePerception()
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
            Frontier(centroid: Vec2(3, -1), widthMeters: 1.0, cellCount: 5),
        ]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.explore(candidateId: "opening_1"), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 blockedHeadingRecoveryTimeout: 0.03,
                                 currentBrain: { brain })

        await agent.handle("go to the oranges")

        XCTAssertEqual(motion.navigateCalls, [Vec2(2, 1)])
        XCTAssertEqual(brain.seenContexts.count, 2)
        let nextContext = brain.seenContexts[1]
        XCTAssertEqual(nextContext.explorationCandidates.first { $0.id == "opening_1" }?.status, .visited)
        XCTAssertEqual(nextContext.explorationCandidates.first { $0.id == "opening_2" }?.status, .unexplored)
        XCTAssertTrue(voice.spoken.isEmpty)
    }

    func testDepthSafetyStopMarksExplorationCandidateVisitedAndContinues() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("Depth safety stop: stop.")
        let perception = FakePerception()
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
            Frontier(centroid: Vec2(3, -1), widthMeters: 1.0, cellCount: 5),
        ]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.explore(candidateId: "opening_1"), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 blockedHeadingRecoveryTimeout: 0.03,
                                 currentBrain: { brain })

        await agent.handle("go to our room")

        XCTAssertEqual(motion.navigateCalls, [Vec2(2, 1)])
        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(brain.seenContexts.count, 2)
        let nextContext = brain.seenContexts[1]
        XCTAssertEqual(nextContext.explorationCandidates.first { $0.id == "opening_1" }?.status, .visited)
        XCTAssertEqual(nextContext.explorationCandidates.first { $0.id == "opening_2" }?.status, .unexplored)
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testScansWhenVisualTargetIsNotDetectedAndNoOpeningIsKnown() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "book",
                            confidence: 0.95,
                            normalizedPoint: CGPoint(x: 0.4, y: 0.6))
        ]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("table")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 1,
                                 currentBrain: { brain })

        await agent.handle("go to the table")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(voice.spoken, ["I couldn't quite figure out where that is."])
    }

    func testScansBeforeGivingUpWhenNoObjectsAreVisibleForVisualTarget() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 2,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6, .pi / 6])
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(brain.seenContexts.count, 2)
        XCTAssertEqual(voice.spoken, ["I couldn't quite figure out where that is."])
        XCTAssertEqual(agent.phase, .idle)
    }

    func testRepeatedVisitedOpeningFallsBackToNextUnexploredCandidate() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [
            .failed("Obstacle ahead at 0.26 m."),
            .arrived,
        ]
        let perception = FakePerception()
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
            Frontier(centroid: Vec2(3, -1), widthMeters: 1.0, cellCount: 5),
        ]
        let brain = FakeBrain(script: [
            .explore(candidateId: "opening_1"),
            .explore(candidateId: "opening_1"),
            .done,
        ])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: FakeVoice(),
                                 blockedHeadingRecoveryTimeout: 0.03,
                                 currentBrain: { brain })

        await agent.handle("go to the chair in the other room")

        XCTAssertEqual(motion.navigateCalls, [Vec2(2, 1), Vec2(3, -1)])
    }

    func testScansSlowlyWhenTargetIsOutOfViewAndNoObjectsAreVisible() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        motion.onRotate = { _ in
            perception.objects = [
                PerceivedObject(label: "chair",
                                confidence: 0.95,
                                normalizedPoint: CGPoint(x: 0.45, y: 0.5))
            ]
        }
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0.01,
                                 maxVisualTargetScanSteps: 3,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(brain.seenContexts.count, 1)
        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testWaitsForDetectorAfterScanTurnBeforeGivingUp() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        motion.onRotate = { _ in
            Task { @MainActor in
                try? await Task.sleep(for: .milliseconds(20))
                perception.objects = [
                    PerceivedObject(label: "chair",
                                    confidence: 0.95,
                                    normalizedPoint: CGPoint(x: 0.52, y: 0.5))
                ]
            }
        }
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0.5,
                                 maxVisualTargetScanSteps: 1,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testEmergencyStopCancelsActiveVisualTargetScanImmediately() async {
        let motion = FakeMotion()
        motion.rotateDelay = 0.05
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "book",
                            confidence: 0.95,
                            normalizedPoint: CGPoint(x: 0.4, y: 0.6))
        ]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 3,
                                 currentBrain: { brain })

        let mission = Task { @MainActor in await agent.handle("go to the chair") }
        while motion.rotateCalls.isEmpty {
            try? await Task.sleep(for: .milliseconds(5))
        }

        await agent.handle("stop")
        await mission.value

        XCTAssertEqual(motion.stopAndWaitCallCount, 1)
        XCTAssertEqual(motion.rotateCalls.count, 1)
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testFridgeAliasMatchesRefrigeratorAndDirectionIsDefined() {
        let objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.97,
                            normalizedPoint: CGPoint(x: 0.2, y: 0.5))
        ]

        let match = MissionAgent.bestVisualTargetMatch(query: "the fridge",
                                                       objects: objects,
                                                       minimumConfidence: 0.90)

        XCTAssertEqual(match?.label, "refrigerator")
        XCTAssertEqual(MissionAgent.visualTargetDirection(for: objects[0].normalizedPoint), "left")
        XCTAssertEqual(MissionAgent.visualTargetDirection(for: CGPoint(x: 0.5, y: 0.5)), "ahead")
        XCTAssertEqual(MissionAgent.visualTargetDirection(for: CGPoint(x: 0.8, y: 0.5)), "right")
    }

    func testScreenRequestMatchesScreenTvAndLaptopCategories() {
        for label in ["screen", "tv", "television", "laptop"] {
            let object = PerceivedObject(
                label: label,
                confidence: 0.95,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )

            XCTAssertEqual(
                MissionAgent.bestVisualTargetMatch(
                    query: "the monitor",
                    objects: [object],
                    minimumConfidence: 0.90
                ),
                object,
                label
            )
        }

        let poster = PerceivedObject(
            label: "poster",
            confidence: 0.99,
            normalizedPoint: CGPoint(x: 0.5, y: 0.5)
        )
        XCTAssertNil(
            MissionAgent.bestVisualTargetMatch(
                query: "the monitor",
                objects: [poster],
                minimumConfidence: 0.90
            )
        )
    }

    func testBlackChairRequiresCategoryAndBlackConfidenceThresholds() {
        let intent = OfflineObjectMissionIntent(
            objectQuery: "black chair",
            targetLabel: "chair",
            requestedColors: [.black],
            searchOtherRooms: false,
            shouldReturn: false
        )
        let grayChair = PerceivedObject(
            label: "chair",
            confidence: 0.99,
            normalizedPoint: CGPoint(x: 0.2, y: 0.5),
            colorEvidence: [ObjectColorEvidence(color: .gray, confidence: 0.95)]
        )
        let weakBlackChair = PerceivedObject(
            label: "chair",
            confidence: 0.99,
            normalizedPoint: CGPoint(x: 0.5, y: 0.5),
            colorEvidence: [ObjectColorEvidence(color: .black, confidence: 0.69)]
        )
        let blackChair = PerceivedObject(
            label: "chair",
            confidence: 0.90,
            normalizedPoint: CGPoint(x: 0.8, y: 0.5),
            colorEvidence: [ObjectColorEvidence(color: .black, confidence: 0.70)]
        )

        XCTAssertEqual(
            MissionAgent.bestVisualTargetMatch(
                intent: intent,
                objects: [grayChair, weakBlackChair, blackChair]
            ),
            blackChair
        )
    }

    func testBlackChairRejectsBlackWheelchairCategory() {
        let intent = OfflineObjectMissionIntent(
            objectQuery: "black chair",
            targetLabel: "chair",
            requestedColors: [.black],
            searchOtherRooms: false,
            shouldReturn: false
        )
        let blackWheelchair = PerceivedObject(
            label: "wheelchair",
            confidence: 0.99,
            normalizedPoint: CGPoint(x: 0.5, y: 0.5),
            colorEvidence: [ObjectColorEvidence(color: .black, confidence: 0.99)]
        )

        XCTAssertNil(
            MissionAgent.bestVisualTargetMatch(intent: intent, objects: [blackWheelchair])
        )
    }

    func testUnqualifiedChairDoesNotRequireColorEvidence() {
        let intent = OfflineObjectMissionIntent(
            objectQuery: "chair",
            targetLabel: "chair",
            requestedColors: [],
            searchOtherRooms: false,
            shouldReturn: false
        )
        let chair = PerceivedObject(
            label: "chair",
            confidence: 0.90,
            normalizedPoint: CGPoint(x: 0.5, y: 0.5)
        )

        XCTAssertEqual(
            MissionAgent.bestVisualTargetMatch(intent: intent, objects: [chair]),
            chair
        )
    }

    func testUnqualifiedChairLogsColorEvidenceAsNotRequired() {
        let intent = OfflineObjectMissionIntent(
            objectQuery: "chair",
            targetLabel: "chair",
            requestedColors: [],
            searchOtherRooms: false,
            shouldReturn: false
        )
        let chair = PerceivedObject(
            label: "chair",
            confidence: 0.90,
            normalizedPoint: CGPoint(x: 0.5, y: 0.5)
        )
        let logBefore = runtimeLog()

        _ = MissionAgent.bestVisualTargetMatch(intent: intent, objects: [chair])

        let logDelta = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertTrue(logDelta.contains("mission_attribute_match"))
        XCTAssertTrue(logDelta.contains("color_confidence=not_required"))
        XCTAssertFalse(logDelta.contains("color_confidence=1.00"))
    }

    func testOriginalBlackChairTargetDoesNotChangeWhenCameraShowsRefrigerator() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let refrigeratorPoint = Vec2(1, 0)
        let chairPoint = Vec2(2, 0)
        var scanCount = 0
        motion.onRotate = { _ in
            scanCount += 1
            if scanCount == 1 {
                perception.objects = [
                    PerceivedObject(
                        label: "refrigerator",
                        confidence: 0.99,
                        normalizedPoint: CGPoint(x: 0.5, y: 0.5),
                        colorEvidence: [ObjectColorEvidence(color: .black, confidence: 0.95)]
                    )
                ]
                perception.unprojectResult = refrigeratorPoint
            } else if scanCount == 2 {
                perception.objects = [
                    PerceivedObject(
                        label: "chair",
                        confidence: 0.95,
                        normalizedPoint: CGPoint(x: 0.5, y: 0.5),
                        colorEvidence: [ObjectColorEvidence(color: .black, confidence: 0.90)]
                    )
                ]
                perception.unprojectResult = chairPoint
            }
        }
        let brain = FakeBrain(script: [.navigate(.visualQuery("refrigerator")), .done])
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 3,
            currentBrain: { brain }
        )

        await agent.handle("go to black chair")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6, .pi / 6])
        XCTAssertEqual(motion.navigateCalls, [chairPoint])
        XCTAssertFalse(motion.navigateCalls.contains(refrigeratorPoint))
    }

    func testScansUntilVisualTargetIsConfidentlyDetected() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "book",
                            confidence: 0.95,
                            normalizedPoint: CGPoint(x: 0.4, y: 0.6))
        ]
        motion.onRotate = { _ in
            perception.objects = [
                PerceivedObject(label: "chair",
                                confidence: 0.95,
                                normalizedPoint: CGPoint(x: 0.4, y: 0.6))
            ]
        }
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertTrue(voice.spoken.isEmpty)
    }

    func testKeepsScanningForVisualTargetWithoutWaitingForAnotherBrainDecision() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "book",
                            confidence: 0.95,
                            normalizedPoint: CGPoint(x: 0.4, y: 0.6))
        ]
        var scanCount = 0
        motion.onRotate = { _ in
            scanCount += 1
            guard scanCount == 2 else { return }
            perception.objects = [
                PerceivedObject(label: "chair",
                                confidence: 0.95,
                                normalizedPoint: CGPoint(x: 0.4, y: 0.6))
            ]
        }
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0.01,
                                 maxVisualTargetScanSteps: 3,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(brain.seenContexts.count, 1)
        XCTAssertEqual(motion.rotateCalls, [.pi / 6, .pi / 6])
        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertTrue(voice.spoken.isEmpty)
    }

    func testDoesNotLockLowConfidenceVisualTarget() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "chair",
                            confidence: 0.89,
                            normalizedPoint: CGPoint(x: 0.4, y: 0.6))
        ]
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.visualQuery("chair")), .done])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 1,
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(voice.spoken, ["I couldn't quite figure out where that is."])
    }

    func testKeepsOriginalVisualTargetWhenBrainDrifts() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "book",
                            confidence: 0.95,
                            normalizedPoint: CGPoint(x: 0.4, y: 0.6))
        ]
        motion.onRotate = { _ in
            perception.objects = [
                PerceivedObject(label: "refrigerator",
                                confidence: 0.95,
                                normalizedPoint: CGPoint(x: 0.4, y: 0.6))
            ]
        }
        let voice = FakeVoice()
        let brain = FakeBrain(script: [
            .navigate(.visualQuery("refrigerator")),
            .navigate(.visualQuery("green chair")),
            .done
        ])
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 currentBrain: { brain })

        await agent.handle("go to the refrigerator")

        XCTAssertEqual(motion.rotateCalls, [.pi / 6])
        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertTrue(voice.spoken.isEmpty)
    }

    func testReportsPhaseChangesWhileHandlingCommand() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.lookAround(angle: .pi), .done])
        var observedPhases: [MissionAgent.Phase] = []
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 phaseDidChange: { observedPhases.append($0) },
                                 currentBrain: { brain })

        await agent.handle("turn left")

        XCTAssertTrue(observedPhases.contains(.thinking))
        XCTAssertTrue(observedPhases.contains(.acting))
        XCTAssertEqual(observedPhases.last, .idle)
    }

    func testAsksThenActsOnTheAnswer() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        voice.scriptedReplies = ["the kitchen"]
        let brain = FakeBrain(script: [.ask("Where should I go?"), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go somewhere")

        XCTAssertEqual(voice.spoken, ["Where should I go?"])
        // The reply becomes both a memory entry and the next think-tick's utterance.
        XCTAssertEqual(brain.seenContexts.map(\.utterance), ["go somewhere", "the kitchen"])
        XCTAssertTrue(agent.memory.turns.contains { $0.utterance == "the kitchen" })
    }

    func testProceedsBestEffortOnNoAnswer() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        voice.scriptedReplies = [nil]
        let brain = FakeBrain(script: [.ask("Where?"), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go somewhere")

        // No reply came back, so the next tick carries no fresh utterance but is flagged
        // so the brain knows not to just ask the same question again.
        XCTAssertEqual(brain.seenContexts.count, 2)
        XCTAssertNil(brain.seenContexts[1].utterance)
        XCTAssertTrue(brain.seenContexts[1].lastAnswerWasInconclusive)
    }

    func testStopsGracefullyWhenNoBrainIsAvailable() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 1) { nil }

        await agent.handle("go to the chair")

        XCTAssertEqual(agent.phase, .idle)
        XCTAssertFalse(voice.spoken.isEmpty)
        XCTAssertTrue(motion.navigateCalls.isEmpty)
    }

    func testBlankUtteranceIsIgnored() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.navigate(.worldPoint(Vec2(4, 5)))])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("   ")

        XCTAssertTrue(brain.seenContexts.isEmpty)
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testLogsBrainErrorsWithContext() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [PerceivedObject(label: "chair", confidence: 0.82, normalizedPoint: CGPoint(x: 0.4, y: 0.6))]
        perception.frontiers = [Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5)]
        let voice = FakeVoice()
        let brain = ThrowingBrain(error: FakeBrainError.modelUnavailable)
        var logged: [(error: Error, context: MissionContext)] = []
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 1,
                                 brainErrorLogger: { error, context in logged.append((error, context)) },
                                 currentBrain: { brain })

        await agent.handle("go to the chair")

        XCTAssertEqual(logged.count, 1)
        XCTAssertEqual(logged.first?.error as? FakeBrainError, .modelUnavailable)
        XCTAssertEqual(logged.first?.context.utterance, "go to the chair")
        XCTAssertEqual(logged.first?.context.visibleObjects.map(\.label), ["chair"])
        XCTAssertEqual(logged.first?.context.explorationCandidates.map(\.id), ["opening_1"])
        XCTAssertEqual(voice.spoken, ["Sorry, I'm having trouble thinking right now."])
    }

    func testBrainUnavailableFallsBackToVisibleRefrigeratorAndDoesNotReturn() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        var statuses: [MissionCommandStatus] = []
        let brain = ThrowingBrain(
            error: RoverBrainError.onDeviceUnavailable(.modelNotReady)
        )
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
        XCTAssertFalse(motion.navigateCalls.contains(.zero))
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected a successful terminal status")
        }
    }

    func testUnreachableVisualSurfaceUsesReachableStandOffGoal() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(1, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: goal.x <= 0.7,
                pathDistance: goal.x
            )
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [Vec2(0.7, 0)])
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
    }

    func testUnreachableVisualSurfaceBacksOffUntilApproachGoalIsReachable() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(2, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: goal.x <= 1.4,
                pathDistance: goal.x
            )
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls.count, 1)
        XCTAssertEqual(motion.navigateCalls[0].x, 1.4, accuracy: 0.001)
        XCTAssertEqual(motion.navigateCalls[0].y, 0, accuracy: 0.001)
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
    }

    func testUnreachableVisualSurfaceDoesNotSubmitKnownUnreachableGoal() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(2, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: false,
                pathDistance: .infinity
            )
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertTrue(motion.navigateCalls.isEmpty)
    }

    func testVisualTargetWaitsForStablePoseAfterInferenceBeforeAssessing() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(4, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        var planningReady = false
        var unprojectionCountAtPreparation = 0
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: planningReady,
                pathDistance: planningReady ? goal.length : .infinity,
                rejectionReason: planningReady ? nil : .trackingUnstable
            )
        }
        motion.onPlanningRecovery = {
            unprojectionCountAtPreparation = perception.unprojectFrameSequences.count
            planningReady = true
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.planningContextRequirements, [.stablePose])
        XCTAssertEqual(perception.unprojectFrameSequences.count, unprojectionCountAtPreparation)
        XCTAssertEqual(motion.navigateCalls, [Vec2(4, 0)])
    }

    func testInitialVisualPlanningFailsClosedWhenStablePoseTimesOut() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(4, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: false,
                pathDistance: .infinity,
                rejectionReason: .trackingUnstable
            )
        }
        motion.planningContextOutcomes[.stablePose] = .trackingTimeout
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.planningContextRequirements, [.stablePose])
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        guard case .failed(_, _, let message) = statuses.last else {
            return XCTFail("Expected a terminal tracking failure")
        }
        XCTAssertEqual(message, "AR tracking did not recover in time.")
    }

    func testBrainDrivenUnreachableVisualTargetStagesAndCompletes() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .arrived]
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.frameSequence = 1
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(3, 0), widthMeters: 1.0, cellCount: 5)
        ]
        motion.goalAssessment = { goal in
            if goal == Vec2(3, 0), motion.navigateCalls.isEmpty {
                return NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: 3)
            }
            let targetBecameReachable = !motion.navigateCalls.isEmpty
                && goal.x >= 4.69 && goal.x <= 4.71
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: targetBecameReachable,
                pathDistance: targetBecameReachable ? 1.7 : .infinity
            )
        }
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(3, 0), yaw: 0)
            perception.frameSequence = 2
        }
        var statuses: [MissionCommandStatus] = []
        let brain = FakeBrain(script: [
            .navigate(.visualQuery("refrigerator"))
        ])
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls.count, 2)
        XCTAssertEqual(motion.navigateCalls[0], Vec2(3, 0))
        XCTAssertEqual(motion.navigateCalls[1].x, 4.7, accuracy: 0.001)
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected brain-driven staged navigation to succeed")
        }
    }

    func testBrainDrivenUnreachableVisualTargetFailsWithoutAskingBrainAgain() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(goal: goal, isReachable: false, pathDistance: .infinity)
        }
        var statuses: [MissionCommandStatus] = []
        let brain = FakeBrain(script: [
            .navigate(.visualQuery("refrigerator"))
        ])
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(brain.seenContexts.count, 1)
        guard case .failed(_, _, let message) = statuses.last else {
            return XCTFail("Expected a terminal no-route failure")
        }
        XCTAssertEqual(message, "No safe route toward target.")
    }

    func testAllUnreachableTargetReacquiresBeforeRefreshedMapPreparation() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(
                label: "refrigerator",
                confidence: 0.99,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )
        ]
        var refreshed = false
        var events: [String] = []
        perception.onDetectObjects = { events.append("detect") }
        motion.goalAssessment = { goal in
            events.append("assess")
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: refreshed,
                pathDistance: refreshed ? goal.length : .infinity,
                rejectionReason: refreshed ? nil : .noConnectedPath
            )
        }
        motion.onPlanningRecovery = {
            events.append("prepare")
            refreshed = true
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.planningRecoveryCallCount, 1)
        XCTAssertEqual(motion.planningContextRequirements, [.refreshedTrustedMesh])
        XCTAssertEqual(Array(events.suffix(3)), ["detect", "prepare", "assess"])
        XCTAssertEqual(motion.navigateCalls, [Vec2(5, 0)])
        XCTAssertGreaterThanOrEqual(perception.unprojectFrameSequences.count, 2)
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected refreshed planning context to complete the mission")
        }
    }

    func testRefreshedMapRetryDoesNotInferAfterReadiness() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(
                label: "refrigerator",
                confidence: 0.99,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )
        ]
        var didPrepare = false
        perception.onDetectObjects = {
            if didPrepare {
                XCTFail("Detector ran after final planning readiness")
            }
        }
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: didPrepare,
                pathDistance: didPrepare ? goal.length : .infinity,
                rejectionReason: didPrepare ? nil : .noConnectedPath
            )
        }
        motion.onPlanningRecovery = { didPrepare = true }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.planningContextRequirements, [.refreshedTrustedMesh])
        XCTAssertEqual(motion.navigateCalls, [Vec2(5, 0)])
    }

    func testStillUnreachableRefreshedMapFailsWithoutSecondRecovery() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(
                label: "refrigerator",
                confidence: 0.99,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: false,
                pathDistance: .infinity,
                rejectionReason: .noConnectedPath
            )
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.planningRecoveryCallCount, 1)
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        guard case .failed(_, _, let message) = statuses.last else {
            return XCTFail("Expected one terminal no-route failure")
        }
        XCTAssertEqual(message, "No safe route toward target.")
    }

    func testUnreachableFarVisualTargetUsesIncrementalRayStepBeforeRetryingGoal() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .arrived]
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.frameSequence = 1
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        motion.goalAssessment = { goal in
            let incrementalStep = abs(goal.x - 0.75) < 0.001 && abs(goal.y) < 0.001
            let adjustedTargetBecameReachable = !motion.navigateCalls.isEmpty
                && abs(goal.x - 4.70) < 0.001
                && abs(goal.y) < 0.001
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: incrementalStep || adjustedTargetBecameReachable,
                pathDistance: incrementalStep ? 0.75 : 3.95
            )
        }
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(0.75, 0), yaw: 0)
            perception.frameSequence = 2
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls.count, 2)
        XCTAssertEqual(motion.navigateCalls[0].x, 0.75, accuracy: 0.001)
        XCTAssertEqual(motion.navigateCalls[0].y, 0, accuracy: 0.001)
        XCTAssertEqual(motion.navigateCalls[1].x, 4.70, accuracy: 0.001)
        XCTAssertEqual(motion.navigateCalls[1].y, 0, accuracy: 0.001)
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected incremental visual approach to complete")
        }
    }

    func testVisibleTargetStagingPrefersCandidateWithMostTargetProgress() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("staging test stop")
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
            Frontier(centroid: Vec2(3, 0), widthMeters: 1.0, cellCount: 5),
            Frontier(centroid: Vec2(-1, 0), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            let isStagingPoint = goal == Vec2(2, 1) || goal == Vec2(3, 0) || goal == Vec2(-1, 0)
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: isStagingPoint,
                pathDistance: goal.length
            )
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls.first, Vec2(3, 0))
    }

    func testVisibleTargetStagingSkipsUnreachableCandidate() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("staging test stop")
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
            Frontier(centroid: Vec2(3, 0), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: goal == Vec2(2, 1),
                pathDistance: goal.length
            )
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls.first, Vec2(2, 1))
    }

    func testVisibleTargetStagingRejectsCandidateWithoutTargetProgress() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(-1, 0), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: goal == Vec2(-1, 0),
                pathDistance: goal.length
            )
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertTrue(motion.navigateCalls.isEmpty)
        guard case .failed = statuses.last else {
            return XCTFail("Expected a no-progress staging failure")
        }
    }

    func testUnreachableVisibleTargetStagesThroughReachableOpeningAndResumes() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .arrived]
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(3, 0), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            if goal == Vec2(3, 0), motion.navigateCalls.isEmpty {
                return NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: 3)
            }
            let targetStandOffBecameReachable = !motion.navigateCalls.isEmpty
                && goal.x >= 4.69 && goal.x <= 4.71
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: targetStandOffBecameReachable,
                pathDistance: targetStandOffBecameReachable ? 1.7 : .infinity
            )
        }
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(3, 0), yaw: 0)
        }
        var phases: [MissionAgent.Phase] = []
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            phaseDidChange: { phases.append($0) },
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls.count, 2)
        XCTAssertEqual(motion.navigateCalls[0], Vec2(3, 0))
        XCTAssertEqual(motion.navigateCalls[1].x, 4.7, accuracy: 0.001)
        XCTAssertEqual(motion.navigateCalls[1].y, 0, accuracy: 0.001)
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
        XCTAssertTrue(phases.contains(.acting))
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected staged refrigerator mission to succeed")
        }
    }

    func testDepthBlockedVisualTargetStagesThroughReachableOpeningAndRetries() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [
            .failed("I can’t safely see the space needed to turn. Reposition the rover or camera and try again."),
            .arrived,
            .arrived,
        ]
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: goal.length)
        }
        motion.onNavigate = { call in
            guard call == 2 else { return }
            perception.pose = Pose2D(position: Vec2(2, 1), yaw: 0)
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [Vec2(5, 0), Vec2(2, 1), Vec2(5, 0)])
        XCTAssertEqual(
            motion.navigateStopClearances,
            [RoverConfig.visualTargetStopDistance, RoverConfig.visualTargetStopDistance]
        )
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected blocked target turn to recover through staging")
        }
    }

    func testDepthBlockedStagingCandidateTriesNextDistinctOpening() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [
            .failed("Depth safety stop: stop."),
            .failed("Depth safety stop: stop."),
            .arrived,
            .arrived,
        ]
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(4, 0), widthMeters: 1.0, cellCount: 5),
            Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: goal.length)
        }
        motion.onNavigate = { call in
            guard call == 3 else { return }
            perception.pose = Pose2D(position: Vec2(2, 1), yaw: 0)
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(
            motion.navigateCalls,
            [Vec2(5, 0), Vec2(4, 0), Vec2(2, 1), Vec2(5, 0)]
        )
        XCTAssertEqual(
            motion.navigateStopClearances,
            [RoverConfig.visualTargetStopDistance, RoverConfig.visualTargetStopDistance]
        )
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected the next safe staging opening to recover the mission")
        }
    }

    func testVisibleTargetStagingDoesNotReuseCandidateAfterFrontierRefresh() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 0), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: goal == Vec2(2, 0),
                pathDistance: goal.length
            )
        }
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(2, 0), yaw: 0)
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [Vec2(2, 0)])
    }

    func testVisibleTargetStagingStopsAfterThreeDistinctCandidates() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.frameSequence = 1
        perception.unprojectResult = Vec2(10, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiersForFrameSequence = { sequence in
            let x: Double
            switch sequence ?? 1 {
            case ...1: x = 2
            case 2: x = 4
            case 3: x = 6
            default: x = 8
            }
            return [Frontier(centroid: Vec2(x, 0), widthMeters: 1.0, cellCount: 5)]
        }
        motion.goalAssessment = { goal in
            let isStagingPoint = [2.0, 4.0, 6.0, 8.0].contains(goal.x) && goal.y == 0
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: isStagingPoint,
                pathDistance: goal.length
            )
        }
        motion.onNavigate = { call in
            let goal = motion.navigateCalls[call - 1]
            perception.pose = Pose2D(position: goal, yaw: 0)
            perception.frameSequence = UInt64(call + 1)
        }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [Vec2(2, 0), Vec2(4, 0), Vec2(6, 0)])
    }

    func testVisibleTargetStagingPreservesTransportFailure() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .failed("Rover command link lost.")
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 0), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: goal == Vec2(2, 0),
                pathDistance: goal.length
            )
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        guard case .failed(_, _, let message) = statuses.last else {
            return XCTFail("Expected transport failure status")
        }
        XCTAssertEqual(message, "Rover command link lost.")
    }

    func testVisibleTargetStagingStopsWhenSessionGenerationChanges() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 1,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.pose = Pose2D(position: .zero, yaw: 0)
        perception.unprojectResult = Vec2(5, 0)
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.frontiers = [
            Frontier(centroid: Vec2(2, 0), widthMeters: 1.0, cellCount: 5),
        ]
        motion.goalAssessment = { goal in
            NavigationGoalAssessment(
                goal: goal,
                isReachable: goal == Vec2(2, 0),
                pathDistance: goal.length
            )
        }
        motion.onNavigate = { call in
            guard call == 1 else { return }
            topology.startSession(
                generation: 2,
                initialPose: Pose2D(position: .zero, yaw: 0)
            )
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            roomTopology: topology,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.stopAndWaitCallCount, 1)
        guard case .failed = statuses.last else {
            return XCTFail("Expected session change to fail the mission")
        }
    }

    func testBrainUnavailableFallsBackToVisibleRefrigeratorAndReturnsWhenRequested() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let startPose = perception.pose!.position
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(1, 0), yaw: .pi)
        }
        var statuses: [MissionCommandStatus] = []
        let brain = ThrowingBrain(
            error: RoverBrainError.onDeviceUnavailable(.modelNotReady)
        )
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the refrigerator and come back")

        XCTAssertEqual(motion.navigateCalls, [Vec2(1, 0), startPose])
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected a successful terminal status")
        }
    }

    func testOfflineTargetScanRetriesOppositeDirectionAfterRotationSafetyFailure() async {
        let motion = FakeMotion()
        motion.scanRotateOutcomes = [
            .failed("Depth safety stop while rotating: blind_swept_volume."),
            .arrived,
        ]
        let perception = FakePerception()
        perception.frameSequence = 1
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            guard motion.scanRotateCalls.count == 2 else { return }
            perception.objects = [
                PerceivedObject(label: "chair",
                                confidence: 0.99,
                                normalizedPoint: CGPoint(x: 0.5, y: 0.5))
            ]
        }
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 3,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the chair")

        XCTAssertEqual(motion.scanRotateCalls, [.pi / 6, -.pi / 6],
                       "a failed turn should retry the same scan step in the opposite direction")
        XCTAssertEqual(motion.navigateCalls, [Vec2(1, 2)])
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected the opposite-direction scan to find the target")
        }
        let logDelta = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertTrue(logDelta.contains("mission_target_scan_rotation_retry"))
    }

    func testBrainUnavailableSearchesAnotherRoomWithFreshScansAndLockedTarget() async throws {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 7,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = [
            PoseObservation(
                pose: Pose2D(position: .zero, yaw: 0),
                frameSequence: 2,
                timestamp: 2,
                trackingQuality: .normal,
                sessionGeneration: 7
            )
        ] + crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 7
        )
        perception.unprojectResult = Vec2(2, 0)
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            if motion.scanRotateCalls.count == 1 {
                perception.objects = [self.blackObject(label: "refrigerator")]
            } else {
                perception.objects = [self.blackObject(label: "chair",
                                                       confidence: 0.90,
                                                       colorConfidence: 0.70)]
                perception.observations.insert(
                    PoseObservation(
                        pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                        frameSequence: 15,
                        timestamp: 15,
                        trackingQuality: .normal,
                        sessionGeneration: 7
                    ),
                    at: 0
                )
            }
        }
        let brain = ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the black chair in the other room")

        XCTAssertEqual(brain.callCount, 1, "offline room search must not ask another brain")
        XCTAssertEqual(motion.scanRotateCalls, [.pi / 6, .pi / 6])
        XCTAssertTrue(motion.scanRotateCalls.allSatisfy {
            (20.0...30.0).contains(abs($0 * 180 / .pi))
        })
        XCTAssertEqual(perception.unprojectFrameSequences, [15],
                       "the other-room match must come from a frame newer than its scan turn")
        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0), Vec2(2, 0)])
        XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_2"))
        XCTAssertEqual(topology.snapshot.doorways.map(\.id), [DoorwayID("doorway_1")])
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected other-room target navigation to succeed")
        }
        let logDelta = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertTrue(logDelta.contains("mission_offline_room_searched"))
        XCTAssertTrue(logDelta.contains("mission_offline_doorway_crossed"))
        XCTAssertTrue(logDelta.contains("target=black chair"))
        XCTAssertFalse(logDelta.contains("mission_offline_return_route_started"))
    }

    func testExplicitOfflineReturnUsesReverseDoorwayRouteThenStartPose() async throws {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 9,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 9
        ) + crossingObservations(
            positions: [1.5, 1.35, 0.65, 0.64, 0.63],
            startingSequence: 20,
            generation: 9,
            yaw: .pi
        )
        perception.unprojectResult = Vec2(2, 0)
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            guard motion.scanRotateCalls.count == 2 else { return }
            perception.objects = [self.blackObject(label: "chair")]
            perception.observations.insert(
                PoseObservation(
                    pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                    frameSequence: 15,
                    timestamp: 15,
                    trackingQuality: .normal,
                    sessionGeneration: 9
                ),
                at: 0
            )
        }
        motion.onNavigate = { call in
            guard call == 2 else { return }
            perception.pose = Pose2D(position: Vec2(2, 0), yaw: .pi)
        }
        let brain = ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the black chair in the other room and come back")

        XCTAssertEqual(brain.callCount, 1)
        XCTAssertEqual(
            motion.navigateCalls,
            [Vec2(1.6, 0), Vec2(2, 0), Vec2(0.4, 0), .zero]
        )
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_1"))
        XCTAssertEqual(topology.snapshot.doorways.count, 1)
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected explicit doorway return to succeed")
        }
        let logDelta = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertTrue(logDelta.contains("mission_offline_return_route_started"))
        XCTAssertTrue(logDelta.contains("mission_offline_return_route_step"))
        XCTAssertTrue(logDelta.contains("mission_offline_return_route_completed"))
        XCTAssertTrue(logDelta.contains("doorway_id=doorway_1"))
    }

    func testOfflineOtherRoomSearchExhaustionStopsWithoutRecrossingSearchedDoorway() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 11,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 11
        )
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            guard motion.scanRotateCalls.count >= 2,
                  let frameSequence = perception.frameSequence else { return }
            perception.observations.insert(
                PoseObservation(
                    pose: perception.pose ?? Pose2D(position: .zero, yaw: 0),
                    frameSequence: frameSequence,
                    timestamp: TimeInterval(frameSequence),
                    trackingQuality: .normal,
                    sessionGeneration: 11
                ),
                at: 0
            )
        }
        let brain = ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
        let voice = FakeVoice()
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: voice,
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Go to the black chair in the other room")

        XCTAssertEqual(brain.callCount, 1)
        XCTAssertEqual(motion.scanRotateCalls, Array(repeating: .pi / 6, count: 14),
                       "two room scans plus one bounded twelve-turn opening search")
        XCTAssertTrue(motion.scanRotateCalls.allSatisfy {
            (20.0...30.0).contains(abs($0 * 180 / .pi))
        })
        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0)],
                       "the searched doorway must not be selected again from room_2")
        XCTAssertGreaterThanOrEqual(motion.cancelCallCount, 1)
        XCTAssertEqual(topology.snapshot.currentRoomID, RoomID("room_2"))
        guard case .failed = statuses.last else {
            return XCTFail("Expected bounded room search exhaustion to fail")
        }
        XCTAssertFalse(voice.spoken.isEmpty)
        let logDelta = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertTrue(logDelta.contains("mission_offline_doorway_marked_searched"))
        XCTAssertTrue(logDelta.contains("mission_offline_fallback_failed"))
        XCTAssertTrue(logDelta.contains("reason=search_exhausted"))
    }

    func testOfflineOtherRoomSearchAbortsWhenSessionGenerationChangesDuringCrossing() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 13,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
        }
        motion.onNavigate = { call in
            guard call == 1 else { return }
            topology.reset(forSessionGeneration: 14)
        }
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the black chair in the other room")

        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0)])
        XCTAssertGreaterThanOrEqual(motion.cancelCallCount, 1)
        XCTAssertEqual(topology.snapshot.sessionGeneration, 14)
        XCTAssertTrue(topology.snapshot.doorways.isEmpty)
        guard case .failed = statuses.last else {
            return XCTFail("Expected session replacement to abort the fallback mission")
        }
        let logDelta = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertTrue(logDelta.contains("mission_offline_fallback_failed"))
        XCTAssertTrue(logDelta.contains("reason=session_generation_changed"))
    }

    func testSameRoomFallbackStopsSuspendedScanWhenSessionGenerationChanges() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 31,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.scanRotationUsesIndependentTask = true
        let perception = FakePerception()
        perception.frameSequence = 1
        let scanStopped = expectation(description: "same-room stale scan stops")
        motion.onScanRotate = { _ in
            motion.onCancel = { scanStopped.fulfill() }
            topology.reset(forSessionGeneration: 32)
        }
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )
        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }

        await fulfillment(of: [scanStopped], timeout: 2)
        if motion.cancelCallCount == 0 {
            mission.cancel()
            motion.cancel()
        }
        await mission.value

        guard case .failed = statuses.last else {
            return XCTFail("Expected the stale same-room scan to fail")
        }
        XCTAssertTrue(
            String(runtimeLog().dropFirst(logBefore.count))
                .contains("reason=session_generation_changed")
        )
    }

    func testSameRoomFallbackStopsTargetNavigationWhenSessionGenerationChanges() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 33,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.navigateOutcome = .driving
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(
                label: "refrigerator",
                confidence: 0.99,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.onNavigate = { call in
            guard call == 1 else { return }
            topology.reset(forSessionGeneration: 34)
        }
        let navigationStopped = expectation(description: "same-room stale navigation stops")
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            roomTopology: topology,
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )
        let observer = Task { @MainActor in
            while !Task.isCancelled {
                if motion.stopAndWaitCallCount > 0 {
                    navigationStopped.fulfill()
                    return
                }
                try? await Task.sleep(for: .milliseconds(5))
            }
        }
        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }

        await fulfillment(of: [navigationStopped], timeout: 2)
        observer.cancel()
        if motion.stopAndWaitCallCount == 0 {
            mission.cancel()
            motion.cancel()
        }
        await mission.value

        XCTAssertGreaterThanOrEqual(motion.stopAndWaitCallCount, 1)
    }

    func testDoorwayDiscoveryStopsSuspendedScanWhenSessionGenerationChanges() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 35,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.scanRotationUsesIndependentTask = true
        let perception = FakePerception()
        perception.frameSequence = 1
        let scanStopped = expectation(description: "stale doorway scan stops")
        motion.onScanRotate = { _ in
            motion.onCancel = { scanStopped.fulfill() }
            topology.reset(forSessionGeneration: 36)
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { nil }
        )
        let mission = Task { @MainActor in
            await agent.handle("Go to another room")
        }

        await fulfillment(of: [scanStopped], timeout: 2)
        if motion.cancelCallCount == 0 {
            mission.cancel()
            motion.cancel()
        }
        await mission.value

        XCTAssertGreaterThanOrEqual(motion.cancelCallCount, 1)
        guard case .failed = statuses.last else {
            return XCTFail("Expected stale doorway discovery to fail")
        }
    }

    func testDoorwayDiscoveryWaitsForNewNormalTrackedFrameBeforeRefreshingFrontiers() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 37,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiersForFrameSequence = { sequence in
            sequence == 4 ? [self.doorwayFrontier()] : []
        }
        perception.observations = [
            PoseObservation(
                pose: Pose2D(position: .zero, yaw: 0),
                frameSequence: 2,
                timestamp: 2,
                trackingQuality: .normal,
                sessionGeneration: 36
            ),
            PoseObservation(
                pose: Pose2D(position: .zero, yaw: 0),
                frameSequence: 3,
                timestamp: 3,
                trackingQuality: .limited,
                sessionGeneration: 37
            ),
            PoseObservation(
                pose: Pose2D(position: .zero, yaw: 0),
                frameSequence: 4,
                timestamp: 4,
                trackingQuality: .normal,
                sessionGeneration: 37
            ),
        ] + crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 37
        )
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0.08,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            currentBrain: { nil }
        )

        await agent.handle("Go to another room")

        XCTAssertEqual(Array(perception.frontierFrameSequences.prefix(2)), [1, 4])
        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0)])
    }

    func testDoorwayDiscoveryDoesNotRefreshFrontiersWithoutFrameIdentity() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 38,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        var frontierReadCount = 0
        perception.frontiersForFrameSequence = { _ in
            frontierReadCount += 1
            return frontierReadCount == 1 ? [] : [self.doorwayFrontier()]
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { nil }
        )

        await agent.handle("Go to another room")

        XCTAssertEqual(motion.scanRotateCalls, [.pi / 6])
        XCTAssertEqual(frontierReadCount, 1)
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        guard case .failed = statuses.last else {
            return XCTFail("Expected missing frame identity to fail closed")
        }
    }

    func testOfflineOtherRoomSearchStopsStaleTraversalBeforeMotionSettles() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 15,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.navigateOutcome = .driving
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
        }
        motion.onNavigate = { call in
            guard call == 1 else { return }
            topology.reset(forSessionGeneration: 16)
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )
        let staleTraversalStopped = expectation(
            description: "stale traversal stops without waiting for motion to settle"
        )
        let stopObserver = Task { @MainActor in
            while !Task.isCancelled {
                if motion.stopAndWaitCallCount > 0 {
                    staleTraversalStopped.fulfill()
                    return
                }
                do {
                    try await Task.sleep(for: .milliseconds(5))
                } catch {
                    return
                }
            }
        }
        let mission = Task { @MainActor in
            await agent.handle("Go to the black chair in the other room")
        }

        await fulfillment(of: [staleTraversalStopped], timeout: 2)
        stopObserver.cancel()
        if motion.stopAndWaitCallCount == 0 {
            mission.cancel()
        }
        await mission.value

        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0)])
        XCTAssertGreaterThanOrEqual(motion.stopAndWaitCallCount, 1)
        guard case .failed = statuses.last else {
            return XCTFail("Expected stale room traversal to fail after its session changes")
        }
    }

    func testOfflineOtherRoomSearchStopsSuspendedScanWhenSessionGenerationChanges() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 17,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 17
        )
        let scanStopped = expectation(
            description: "stale post-doorway scan stops without waiting for rotation"
        )
        motion.onScanRotate = { _ in
            guard motion.scanRotateCalls.count == 2 else {
                perception.frameSequence = (perception.frameSequence ?? 0) + 1
                return
            }
            motion.scanRotationUsesIndependentTask = true
            motion.onCancel = { scanStopped.fulfill() }
            topology.reset(forSessionGeneration: 18)
        }
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )
        let mission = Task { @MainActor in
            await agent.handle("Go to the black chair in the other room")
        }

        await fulfillment(of: [scanStopped], timeout: 2)
        if motion.cancelCallCount == 0 {
            mission.cancel()
            motion.cancel()
        }
        await mission.value

        XCTAssertGreaterThanOrEqual(motion.cancelCallCount, 1)
        guard case .failed = statuses.last else {
            return XCTFail("Expected stale post-doorway scan to fail the mission")
        }
        let logDelta = String(runtimeLog().dropFirst(logBefore.count))
        XCTAssertTrue(logDelta.contains("reason=session_generation_changed"))
    }

    func testOtherRoomScanWaitsForNormalTrackedFrameFromCurrentGeneration() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 19,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 19
        )
        perception.unprojectResult = Vec2(2, 0)
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            guard motion.scanRotateCalls.count == 2 else { return }
            perception.objects = [self.blackObject(label: "chair")]
            perception.observations.insert(contentsOf: [
                PoseObservation(
                    pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                    frameSequence: 20,
                    timestamp: 20,
                    trackingQuality: .normal,
                    sessionGeneration: 18
                ),
                PoseObservation(
                    pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                    frameSequence: 21,
                    timestamp: 21,
                    trackingQuality: .limited,
                    sessionGeneration: 19
                ),
                PoseObservation(
                    pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                    frameSequence: 22,
                    timestamp: 22,
                    trackingQuality: .normal,
                    sessionGeneration: 19
                ),
            ], at: 0)
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0.08,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the black chair in the other room")

        XCTAssertEqual(perception.unprojectFrameSequences, [22])
        XCTAssertEqual(motion.navigateCalls, [Vec2(1.6, 0), Vec2(2, 0)])
        guard case .succeeded = statuses.last else {
            return XCTFail("Expected target grounding only after the current normal frame")
        }
    }

    func testOtherRoomTargetNavigationStopsWhenSessionGenerationChanges() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 21,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .driving]
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 21
        )
        perception.unprojectResult = Vec2(2, 0)
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            guard motion.scanRotateCalls.count == 2 else { return }
            perception.objects = [self.blackObject(label: "chair")]
            perception.observations.insert(
                PoseObservation(
                    pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                    frameSequence: 20,
                    timestamp: 20,
                    trackingQuality: .normal,
                    sessionGeneration: 21
                ),
                at: 0
            )
        }
        motion.onNavigate = { call in
            if call == 2 { topology.reset(forSessionGeneration: 22) }
        }
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0.05,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )
        let stopped = expectation(description: "stale target navigation stops")
        let observer = Task { @MainActor in
            while !Task.isCancelled {
                if motion.stopAndWaitCallCount > 0 {
                    stopped.fulfill()
                    return
                }
                try? await Task.sleep(for: .milliseconds(5))
            }
        }
        let mission = Task { @MainActor in
            await agent.handle("Go to the black chair in the other room")
        }

        await fulfillment(of: [stopped], timeout: 2)
        observer.cancel()
        if motion.stopAndWaitCallCount == 0 {
            mission.cancel()
            motion.cancel()
        }
        await mission.value

        guard case .failed = statuses.last else {
            return XCTFail("Expected stale target navigation to fail")
        }
        XCTAssertTrue(
            String(runtimeLog().dropFirst(logBefore.count))
                .contains("reason=session_generation_changed")
        )
    }

    func testOtherRoomFinalReturnStopsWhenSessionGenerationChanges() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 23,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .arrived, .arrived, .driving]
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 23
        ) + crossingObservations(
            positions: [1.5, 1.35, 0.65, 0.64, 0.63],
            startingSequence: 30,
            generation: 23,
            yaw: .pi
        )
        perception.unprojectResult = Vec2(2, 0)
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            guard motion.scanRotateCalls.count == 2 else { return }
            perception.objects = [self.blackObject(label: "chair")]
            perception.observations.insert(
                PoseObservation(
                    pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                    frameSequence: 20,
                    timestamp: 20,
                    trackingQuality: .normal,
                    sessionGeneration: 23
                ),
                at: 0
            )
        }
        motion.onNavigate = { call in
            if call == 2 {
                perception.pose = Pose2D(position: Vec2(2, 0), yaw: .pi)
            } else if call == 4 {
                topology.reset(forSessionGeneration: 24)
            }
        }
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0.05,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )
        let stopped = expectation(description: "stale final return stops")
        let observer = Task { @MainActor in
            while !Task.isCancelled {
                if motion.stopAndWaitCallCount > 0 {
                    stopped.fulfill()
                    return
                }
                try? await Task.sleep(for: .milliseconds(5))
            }
        }
        let mission = Task { @MainActor in
            await agent.handle("Go to the black chair in the other room and come back")
        }

        await fulfillment(of: [stopped], timeout: 2)
        observer.cancel()
        if motion.stopAndWaitCallCount == 0 {
            mission.cancel()
            motion.cancel()
        }
        await mission.value

        guard case .failed = statuses.last else {
            return XCTFail("Expected stale final return to fail")
        }
        XCTAssertTrue(
            String(runtimeLog().dropFirst(logBefore.count))
                .contains("reason=session_generation_changed")
        )
    }

    func testBrainUnavailableRejectsUnsupportedCommandWithoutMotion() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        var statuses: [MissionCommandStatus] = []
        let brain = ThrowingBrain(
            error: RoverBrainError.onDeviceUnavailable(.modelNotReady)
        )
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        await agent.handle("Tell me a joke")

        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertTrue(motion.rotateCalls.isEmpty)
        XCTAssertTrue(motion.scanRotateCalls.isEmpty)
        guard case .failed = statuses.last else {
            return XCTFail("Expected an unsupported command failure")
        }
    }

    func testUnavailableBrainFallbackEmitsOrderedMissionTelemetry() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(
                label: "refrigerator",
                confidence: 0.99,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )
        ]
        perception.unprojectResult = Vec2(1, 0)
        var events: [(name: String, fields: [String: String])] = []
        let telemetry: MissionTelemetrySink = { name, fields in
            events.append((name, fields))
        }
        let brain = TelemetryThrowingBrain(
            telemetry: telemetry,
            error: RoverBrainError.onDeviceUnavailable(.modelNotReady)
        )
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
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
        XCTAssertEqual(events[0].fields["reason"], "on_device_failed")
        XCTAssertEqual(events[1].fields["brain"], "offline_object_fallback")
        XCTAssertEqual(events[1].fields["reason"], "brain_error")
        XCTAssertEqual(events[2].fields["reason"], "brain_error")
        XCTAssertEqual(events[3].fields["reason"], "matched_required_attributes")
    }

    func testOtherRoomFallbackEmitsStructuredSearchAndReturnTelemetry() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(generation: 27, initialPose: Pose2D(position: .zero, yaw: 0))
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.frameSequence = 1
        perception.frontiers = [doorwayFrontier()]
        perception.observations = crossingObservations(
            positions: [0.5, 0.65, 1.35, 1.36, 1.37],
            startingSequence: 10,
            generation: 27
        ) + crossingObservations(
            positions: [1.5, 1.35, 0.65, 0.64, 0.63],
            startingSequence: 30,
            generation: 27,
            yaw: .pi
        )
        perception.unprojectResult = Vec2(2, 0)
        motion.onScanRotate = { _ in
            perception.frameSequence = (perception.frameSequence ?? 0) + 1
            guard motion.scanRotateCalls.count == 2 else { return }
            perception.objects = [self.blackObject(label: "chair")]
            perception.observations.insert(
                PoseObservation(
                    pose: Pose2D(position: Vec2(1.37, 0), yaw: 0),
                    frameSequence: 20,
                    timestamp: 20,
                    trackingQuality: .normal,
                    sessionGeneration: 27
                ),
                at: 0
            )
        }
        motion.onNavigate = { call in
            guard call == 2 else { return }
            perception.pose = Pose2D(position: Vec2(2, 0), yaw: .pi)
        }
        var events: [(name: String, fields: [String: String])] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 1,
            roomTopology: topology,
            roomTransitionPollInterval: 0,
            missionTelemetry: { events.append(($0, $1)) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        await agent.handle("Go to the black chair in the other room and come back")

        let names = events.map(\.name)
        XCTAssertTrue(names.contains("mission_room_search_started"))
        XCTAssertEqual(names.filter { $0 == "mission_room_searched" }.count, 2)
        XCTAssertTrue(names.contains("mission_doorway_crossed"))
        XCTAssertTrue(names.contains("mission_return_route_started"))
        XCTAssertTrue(names.contains("mission_return_route_step"))
        XCTAssertTrue(names.contains("mission_return_route_completed"))
        for event in events where event.name.hasPrefix("mission_room_")
            || event.name.hasPrefix("mission_doorway_")
            || event.name.hasPrefix("mission_return_route_") {
            XCTAssertEqual(event.fields["mission"], "1")
            XCTAssertNotNil(event.fields["reason"])
        }
    }

    func testBrainFailureAfterUsableOutputDoesNotMixInOfflineFallback() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        let voice = FakeVoice()
        let brain = OutputThenThrowingBrain(
            output: BrainOutput(decision: .say("I am checking.")),
            error: RoverBrainError.onDeviceUnavailable(.modelNotReady)
        )
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: voice,
            currentBrain: { brain }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(voice.spoken, [
            "I am checking.",
            "Sorry, I'm having trouble thinking right now."
        ])
    }

    func testCallerCancellationDoesNotStartOfflineFallback() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        var statuses: [MissionCommandStatus] = []
        let brain = CancellableBlockingBrain()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            brainDecisionTimeout: 0.05,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )

        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }
        await brain.waitUntilEntered()
        mission.cancel()
        await mission.value

        XCTAssertTrue(motion.navigateCalls.isEmpty)
        guard case .cancelled = statuses.last else {
            return XCTFail("Expected caller cancellation to remain authoritative")
        }
    }

    func testCallerCancellationRacingBrainOutputDoesNotExecuteDecision() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        var statuses: [MissionCommandStatus] = []
        let brain = CancellingOutputBrain(
            output: BrainOutput(decision: .navigate(.worldPoint(Vec2(3, 4))))
        )
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { brain }
        )
        var mission: Task<Void, Never>?
        brain.beforeReturningOutput = { mission?.cancel() }

        mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }
        await mission?.value

        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .cancelled(id: 1, command: "Go to the refrigerator")
        )
    }

    func testCallerCancellationDuringOfflineScanStopsRotation() async {
        let motion = FakeMotion()
        motion.scanRotationUsesIndependentTask = true
        let perception = FakePerception()
        var statuses: [MissionCommandStatus] = []
        let scanStarted = expectation(description: "offline scan started")
        let scanCancelled = expectation(description: "offline scan cancelled")
        motion.onScanRotate = { _ in scanStarted.fulfill() }
        motion.onCancel = { scanCancelled.fulfill() }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            visualTargetScanDelay: 0,
            maxVisualTargetScanSteps: 2,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }
        await fulfillment(of: [scanStarted], timeout: 1)
        mission.cancel()
        await fulfillment(of: [scanCancelled], timeout: 2)
        if motion.cancelCallCount == 0 {
            motion.cancel()
        }
        await mission.value

        XCTAssertGreaterThanOrEqual(motion.cancelCallCount, 1)
        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .cancelled(id: 1, command: "Go to the refrigerator")
        )
    }

    func testCallerCancellationDuringOfflineNavigationStopsDriving() async {
        let motion = FakeMotion()
        motion.navigateOutcome = .driving
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        var statuses: [MissionCommandStatus] = []
        let navigationStarted = expectation(description: "offline navigation started")
        let navigationCancelled = expectation(description: "offline navigation cancelled")
        motion.onNavigate = { _ in navigationStarted.fulfill() }
        motion.onCancel = { navigationCancelled.fulfill() }
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator")
        }
        await fulfillment(of: [navigationStarted], timeout: 1)
        mission.cancel()
        await fulfillment(of: [navigationCancelled], timeout: 1)
        await mission.value

        XCTAssertGreaterThanOrEqual(motion.cancelCallCount, 1)
        XCTAssertEqual(motion.state, .idle)
        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .cancelled(id: 1, command: "Go to the refrigerator")
        )
    }

    func testEmergencyStopDuringOfflineReturnRemainsCancelledWithoutMoreMotion() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .driving]
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 0)
        let returnNavigationStarted = expectation(description: "return navigation started")
        motion.onNavigate = { call in
            if call == 1 {
                perception.pose = Pose2D(position: Vec2(1, 0), yaw: .pi)
            } else if call == 2 {
                returnNavigationStarted.fulfill()
            }
        }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator and come back")
        }
        await fulfillment(of: [returnNavigationStarted], timeout: 1)
        await agent.handle("stop")
        let motionCountAfterStop = motion.navigateCalls.count + motion.scanRotateCalls.count
        await mission.value

        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .cancelled(id: 1, command: "Go to the refrigerator and come back")
        )
        XCTAssertEqual(motion.navigateCalls.count + motion.scanRotateCalls.count, motionCountAfterStop)
    }

    func testCallerCancellationDuringReturnAlignmentStopsIndependentRotation() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived]
        motion.scanRotationUsesIndependentTask = true
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(1, 0), yaw: 0)
        }
        let alignmentStarted = expectation(description: "return alignment started")
        let alignmentCancelled = expectation(description: "return alignment cancelled")
        motion.onScanRotate = { _ in alignmentStarted.fulfill() }
        motion.onCancel = { alignmentCancelled.fulfill() }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator and come back")
        }
        await fulfillment(of: [alignmentStarted], timeout: 1)
        mission.cancel()
        await fulfillment(of: [alignmentCancelled], timeout: 2)
        if motion.cancelCallCount == 0 {
            motion.cancel()
        }
        await mission.value

        XCTAssertEqual(motion.navigateCalls.count, 1)
        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .cancelled(id: 1, command: "Go to the refrigerator and come back")
        )
    }

    func testCallerCancellationDuringBlockedReturnRecoveryStopsIndependentRotation() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .failed("Obstacle ahead")]
        motion.scanRotationUsesIndependentTask = true
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(1, 0), yaw: .pi)
        }
        let recoveryStarted = expectation(description: "blocked return recovery started")
        let recoveryCancelled = expectation(description: "blocked return recovery cancelled")
        motion.onScanRotate = { _ in recoveryStarted.fulfill() }
        motion.onCancel = { recoveryCancelled.fulfill() }
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            blockedHeadingRecoveryTimeout: 0.15,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator and come back")
        }
        await fulfillment(of: [recoveryStarted], timeout: 1)
        mission.cancel()
        await fulfillment(of: [recoveryCancelled], timeout: 2)
        if motion.cancelCallCount == 0 {
            motion.cancel()
        }
        await mission.value

        XCTAssertEqual(motion.navigateCalls.count, 2)
        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .cancelled(id: 1, command: "Go to the refrigerator and come back")
        )
    }

    func testSessionResetDuringBlockedReturnRecoveryStopsIndependentRotation() async {
        let topology = SessionRoomTopology(telemetry: { _, _ in })
        topology.startSession(
            generation: 39,
            initialPose: Pose2D(position: .zero, yaw: 0)
        )
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .failed("Obstacle ahead")]
        motion.scanRotationUsesIndependentTask = true
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(
                label: "refrigerator",
                confidence: 0.99,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            )
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(1, 0), yaw: .pi)
        }
        let recoveryStopped = expectation(description: "stale return recovery stops")
        motion.onScanRotate = { _ in
            motion.onCancel = { recoveryStopped.fulfill() }
            topology.reset(forSessionGeneration: 40)
        }
        var statuses: [MissionCommandStatus] = []
        let logBefore = runtimeLog()
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            blockedHeadingRecoveryTimeout: 1,
            roomTopology: topology,
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )
        let mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator and come back")
        }

        await fulfillment(of: [recoveryStopped], timeout: 1)
        if motion.cancelCallCount == 0 {
            mission.cancel()
            motion.cancel()
        }
        await mission.value

        guard case .failed = statuses.last else {
            return XCTFail("Expected stale return recovery to fail")
        }
        XCTAssertTrue(
            String(runtimeLog().dropFirst(logBefore.count))
                .contains("reason=session_generation_changed")
        )
    }

    func testCallerCancellationImmediatelyBeforeOfflineReturnSuccessWinsTerminalBoundary() async {
        let motion = FakeMotion()
        motion.navigateOutcomes = [.arrived, .arrived]
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        perception.unprojectResult = Vec2(1, 0)
        motion.onNavigate = { call in
            guard call == 1 else { return }
            perception.pose = Pose2D(position: Vec2(1, 0), yaw: .pi)
        }
        var statuses: [MissionCommandStatus] = []
        var mission: Task<Void, Never>?
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            phaseDidChange: { phase in
                guard phase == .idle, motion.navigateCalls.count == 2 else { return }
                mission?.cancel()
            },
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: {
                ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
            }
        )

        mission = Task { @MainActor in
            await agent.handle("Go to the refrigerator and come back")
        }
        await mission?.value

        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .cancelled(id: 1, command: "Go to the refrigerator and come back")
        )
    }

    func testBrainCancellationErrorFallsBackWhenMissionIsStillActive() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        perception.objects = [
            PerceivedObject(label: "refrigerator",
                            confidence: 0.99,
                            normalizedPoint: CGPoint(x: 0.5, y: 0.5))
        ]
        var statuses: [MissionCommandStatus] = []
        let agent = MissionAgent(
            motion: motion,
            perception: perception,
            voice: FakeVoice(),
            commandStatusDidChange: { statuses.append($0) },
            currentBrain: { ThrowingBrain(error: CancellationError()) }
        )

        await agent.handle("Go to the refrigerator")

        XCTAssertEqual(motion.navigateCalls, [perception.unprojectResult])
        XCTAssertEqual(
            statuses.first { $0.id == 1 && $0.isTerminal },
            .succeeded(id: 1, command: "Go to the refrigerator", message: "Command completed.")
        )
    }

    func testStopDecisionCancelsMotion() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.stop])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("never mind")

        XCTAssertEqual(motion.cancelCallCount, 1)
    }

    func testEmergencyStopBypassesMissingPoseAndBrain() async {
        let motion = FakeMotion()
        motion.state = .driving
        motion.stopAndWaitDelay = 0.02
        let perception = FakePerception()
        perception.pose = nil
        let voice = FakeVoice()
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { nil }

        await agent.handle("stop")

        XCTAssertEqual(motion.stopAndWaitCallCount, 1)
        XCTAssertEqual(motion.lifecycleEvents, ["stopStarted", "stopFinished"])
        XCTAssertEqual(motion.state, .idle)
        XCTAssertEqual(agent.phase, .idle)
        XCTAssertTrue(voice.spoken.isEmpty)
    }

    func testEmergencyStopBypassesBrainErrors() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = ThrowingBrain(error: FakeBrainError.modelUnavailable)
        var logged: [(error: Error, context: MissionContext)] = []
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 brainErrorLogger: { error, context in logged.append((error, context)) },
                                 currentBrain: { brain })

        await agent.handle("emergency stop")

        XCTAssertEqual(motion.stopAndWaitCallCount, 1)
        XCTAssertEqual(agent.phase, .idle)
        XCTAssertTrue(voice.spoken.isEmpty)
        XCTAssertTrue(logged.isEmpty)
    }

    func testNormalCommandDoesNotEnterBrainWhileMissionIsThinking() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = BlockingBrain()
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        let first = Task { @MainActor in await agent.handle("go to the chair") }
        await brain.waitUntilEntered()

        let second = Task { @MainActor in await agent.handle("go to the table") }
        try? await Task.sleep(for: .milliseconds(20))

        XCTAssertEqual(brain.seenContexts.map(\.utterance), ["go to the chair"])
        XCTAssertEqual(voice.spoken, ["I'm still working on the previous command. Say stop if you want me to cancel it."])

        brain.finishAll(with: .done)
        await first.value
        await second.value
    }

    func testEmergencyStopInvalidatesInFlightBrainDecision() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = BlockingBrain()
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        let first = Task { @MainActor in await agent.handle("go to the chair") }
        await brain.waitUntilEntered()

        await agent.handle("stop")
        brain.finishAll(with: .navigate(.worldPoint(Vec2(4, 5))))
        await first.value

        XCTAssertEqual(motion.stopAndWaitCallCount, 1)
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(agent.phase, .idle)
    }

    func testBrainDecisionTimeoutReturnsMissionToIdle() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = BlockingBrain()
        let agent = MissionAgent(motion: motion,
                                 perception: perception,
                                 voice: voice,
                                 brainDecisionTimeout: 0.03,
                                 visualTargetScanDelay: 0,
                                 maxVisualTargetScanSteps: 1,
                                 currentBrain: { brain })

        await agent.handle("go to the refrigerator")

        XCTAssertEqual(brain.seenContexts.map(\.utterance), ["go to the refrigerator"])
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(voice.spoken, ["Sorry, I'm having trouble thinking right now."])
        XCTAssertEqual(agent.phase, .idle)
    }

    private func runtimeLog() -> String {
        guard let url = RuntimeFileLog.logFileURL else { return "" }
        return (try? String(contentsOf: url, encoding: .utf8)) ?? ""
    }

    private func doorwayFrontier() -> Frontier {
        Frontier(
            centroid: Vec2(1, 0),
            widthMeters: 1,
            cellCount: 6,
            outwardDirection: Vec2(1, 0)
        )
    }

    private func crossingObservations(
        positions: [Double],
        startingSequence: UInt64,
        generation: UInt64,
        yaw: Double = 0
    ) -> [PoseObservation] {
        positions.enumerated().map { index, x in
            let sequence = startingSequence + UInt64(index)
            return PoseObservation(
                pose: Pose2D(position: Vec2(x, 0), yaw: yaw),
                frameSequence: sequence,
                timestamp: TimeInterval(sequence),
                trackingQuality: .normal,
                sessionGeneration: generation
            )
        }
    }

    private func blackObject(
        label: String,
        confidence: Float = 0.95,
        colorConfidence: Float = 0.90
    ) -> PerceivedObject {
        PerceivedObject(
            label: label,
            confidence: confidence,
            normalizedPoint: CGPoint(x: 0.5, y: 0.5),
            colorEvidence: [
                ObjectColorEvidence(color: .black, confidence: colorConfidence),
            ]
        )
    }
}

// MARK: - Fakes

@MainActor
private final class FakeFollower: RoverFollowing {
    private(set) var followingSpec: TargetSpec?
    var followState: FollowController.State = .idle
    private(set) var followCalls: [TargetSpec] = []
    private(set) var stopCallCount = 0
    /// Shared with the fake motion so a test can assert what happened first.
    var onEvent: ((String) -> Void)?
    /// Ends the follow on its own after this long, so a test doesn't sit out the whole
    /// brain review interval.
    var endsAfter: TimeInterval = 0.03

    func follow(_ spec: TargetSpec, seed: Vec2?) {
        followCalls.append(spec)
        followingSpec = spec
        followState = .following
        onEvent?("follow")
        Task { @MainActor in
            try? await Task.sleep(for: .seconds(endsAfter))
            if followingSpec != nil { followState = .ended("lost them") }
        }
    }

    func stopFollowing() {
        stopCallCount += 1
        followingSpec = nil
        followState = .idle
        onEvent?("stopFollowing")
    }
}

private enum FakeBrainError: Error, Equatable {
    case modelUnavailable
}

@MainActor
private final class FakeBrain: RoverBrain {
    private var script: [RoverDecision]
    private(set) var seenContexts: [MissionContext] = []

    init(script: [RoverDecision]) { self.script = script }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        seenContexts.append(context)
        return BrainOutput(decision: script.isEmpty ? .done : script.removeFirst())
    }
}

@MainActor
private final class OutputBrain: RoverBrain {
    private var outputs: [BrainOutput]
    private(set) var seenContexts: [MissionContext] = []

    init(outputs: [BrainOutput]) { self.outputs = outputs }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        seenContexts.append(context)
        return outputs.isEmpty ? BrainOutput(decision: .done) : outputs.removeFirst()
    }
}

@MainActor
private final class ThrowingBrain: RoverBrain {
    private let error: Error
    private(set) var callCount = 0

    init(error: Error) { self.error = error }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        callCount += 1
        throw error
    }
}

@MainActor
private final class TelemetryThrowingBrain: RoverBrain {
    private let telemetry: MissionTelemetrySink
    private let error: Error

    init(telemetry: @escaping MissionTelemetrySink, error: Error) {
        self.telemetry = telemetry
        self.error = error
    }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        telemetry("mission_brain_selected", [
            "mission": context.missionID.map(String.init) ?? "none",
            "brain": "on_device",
            "reason": "on_device_failed",
        ])
        throw error
    }
}

@MainActor
private final class OutputThenThrowingBrain: RoverBrain {
    private var output: BrainOutput?
    private let error: Error

    init(output: BrainOutput, error: Error) {
        self.output = output
        self.error = error
    }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        if let output {
            self.output = nil
            return output
        }
        throw error
    }
}

@MainActor
private final class CancellingOutputBrain: RoverBrain {
    let output: BrainOutput
    var beforeReturningOutput: (() -> Void)?

    init(output: BrainOutput) {
        self.output = output
    }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        beforeReturningOutput?()
        return output
    }
}

@MainActor
private final class CancellableBlockingBrain: RoverBrain {
    private var enteredContinuation: CheckedContinuation<Void, Never>?
    private var entered = false

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        entered = true
        enteredContinuation?.resume()
        enteredContinuation = nil
        while true {
            try await Task.sleep(for: .seconds(1))
        }
    }

    func waitUntilEntered() async {
        guard !entered else { return }
        await withCheckedContinuation { continuation in
            enteredContinuation = continuation
        }
    }
}

@MainActor
private final class BlockingBrain: RoverBrain {
    private(set) var seenContexts: [MissionContext] = []
    private var enteredContinuation: CheckedContinuation<Void, Never>?
    private var decision: RoverDecision?

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        seenContexts.append(context)
        enteredContinuation?.resume()
        enteredContinuation = nil
        while decision == nil {
            try? await Task.sleep(for: .milliseconds(5))
        }
        return BrainOutput(decision: decision ?? .done)
    }

    func waitUntilEntered() async {
        guard seenContexts.isEmpty else { return }
        await withCheckedContinuation { continuation in
            enteredContinuation = continuation
        }
    }

    func finishAll(with decision: RoverDecision) {
        self.decision = decision
    }
}

@MainActor
private final class FakeMotion: RoverMotion {
    var state: NavigationController.State = .idle
    private(set) var navigateCalls: [Vec2] = []
    private(set) var navigateStopClearances: [Double] = []
    private(set) var rotateCalls: [Double] = []
    private(set) var scanRotateCalls: [Double] = []
    private(set) var cancelCallCount = 0
    private(set) var stopAndWaitCallCount = 0
    private(set) var planningRecoveryCallCount = 0
    private(set) var lifecycleEvents: [String] = []
    /// What `state` settles to shortly after `navigate(to:)` — simulates the real drive
    /// loop reaching `.arrived` asynchronously.
    var navigateOutcome: NavigationController.State = .arrived
    var navigateOutcomes: [NavigationController.State] = []
    var beginsNavigationInPlanning = false
    var planningDelay: TimeInterval = 0.03
    var rotateNeverCompletes = false
    var scanRotationUsesIndependentTask = false
    var scanRotateOutcomes: [NavigationController.State] = []
    var rotateDelay: TimeInterval = 0
    var stopAndWaitDelay: TimeInterval = 0
    var onNavigate: ((Int) -> Void)?
    var onRotate: ((Double) -> Void)?
    var onScanRotate: ((Double) -> Void)?
    var onCancel: (() -> Void)?
    var goalAssessment: ((Vec2) -> NavigationGoalAssessment)?
    var planningContextOutcomes: [PlanningContextRequirement: PlanningRecoveryOutcome] = [
        .stablePose: .ready,
        .refreshedTrustedMesh: .ready,
    ]
    var planningRecoveryOutcome: PlanningRecoveryOutcome {
        get { planningContextOutcomes[.refreshedTrustedMesh] ?? .ready }
        set { planningContextOutcomes[.refreshedTrustedMesh] = newValue }
    }
    private(set) var planningContextRequirements: [PlanningContextRequirement] = []
    var onPlanningRecovery: (() -> Void)?
    private var independentScanTask: Task<Void, Never>?

    func navigate(to goal: Vec2) {
        lifecycleEvents.append("navigate")
        navigateCalls.append(goal)
        onNavigate?(navigateCalls.count)
        state = beginsNavigationInPlanning ? .planning : .driving
        let outcome = navigateOutcomes.isEmpty ? navigateOutcome : navigateOutcomes.removeFirst()
        Task { @MainActor in
            if beginsNavigationInPlanning {
                try? await Task.sleep(for: .seconds(planningDelay))
                guard state == .planning else { return }
                state = .driving
            }
            try? await Task.sleep(for: .milliseconds(20))
            guard state == .driving else { return }
            state = outcome
        }
    }

    func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) {
        navigateStopClearances.append(clearance)
        navigate(to: goal)
    }

    func rotate(by angle: Double) async {
        rotateCalls.append(angle)
        onRotate?(angle)
        if rotateDelay > 0 {
            try? await Task.sleep(for: .seconds(rotateDelay))
        }
        guard !rotateNeverCompletes else {
            state = .driving
            while state == .driving, !Task.isCancelled {
                try? await Task.sleep(for: .milliseconds(5))
            }
            return
        }
        state = .arrived
    }

    func rotateForScan(by angle: Double) async {
        scanRotateCalls.append(angle)
        onScanRotate?(angle)
        if !scanRotateOutcomes.isEmpty {
            state = scanRotateOutcomes.removeFirst()
            return
        }
        if scanRotationUsesIndependentTask {
            state = .driving
            let task = Task { @MainActor in
                while !Task.isCancelled {
                    try? await Task.sleep(for: .milliseconds(5))
                }
            }
            independentScanTask = task
            await task.value
            independentScanTask = nil
            return
        }
        await rotate(by: angle)
    }

    func cancel() {
        cancelCallCount += 1
        independentScanTask?.cancel()
        independentScanTask = nil
        state = .idle
        let cancellationObserver = onCancel
        onCancel = nil
        cancellationObserver?()
    }

    func stopAndWait() async {
        stopAndWaitCallCount += 1
        lifecycleEvents.append("stopStarted")
        if stopAndWaitDelay > 0 {
            try? await Task.sleep(for: .seconds(stopAndWaitDelay))
        }
        state = .idle
        lifecycleEvents.append("stopFinished")
    }

    func assessGoal(_ goal: Vec2) -> NavigationGoalAssessment {
        goalAssessment?(goal)
            ?? NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: 0)
    }

    func preparePlanningContext(
        requiring requirement: PlanningContextRequirement
    ) async -> PlanningRecoveryOutcome {
        planningRecoveryCallCount += 1
        planningContextRequirements.append(requirement)
        onPlanningRecovery?()
        return planningContextOutcomes[requirement] ?? .ready
    }

    func recoverPlanningContext() async -> PlanningRecoveryOutcome {
        await preparePlanningContext(requiring: .refreshedTrustedMesh)
    }
}

@MainActor
private final class FakePerception: RoverPerception {
    var pose: Pose2D? = Pose2D(position: .zero, yaw: 0)
    var objects: [PerceivedObject] = []
    var frontiers: [Frontier] = []
    var unprojectResult: Vec2? = Vec2(1, 2)
    var frameSequence: UInt64? = nil
    var observations: [PoseObservation] = []
    var frontiersForFrameSequence: ((UInt64?) -> [Frontier])?
    private(set) var unprojectFrameSequences: [UInt64?] = []
    private(set) var frontierFrameSequences: [UInt64?] = []
    var onDetectObjects: (() -> Void)?

    var latestObservation: PoseObservation? {
        guard !observations.isEmpty else { return nil }
        let observation = observations.removeFirst()
        pose = observation.pose
        frameSequence = observation.frameSequence
        return observation
    }

    func detectObjects() -> [PerceivedObject] {
        onDetectObjects?()
        return objects
    }
    func unproject(normalizedPoint: CGPoint) -> Vec2? {
        unprojectFrameSequences.append(frameSequence)
        return unprojectResult
    }
    func capturedFrameJPEG() -> Data? { nil }
    func explorationFrontiers() -> [Frontier] {
        frontierFrameSequences.append(frameSequence)
        return frontiersForFrameSequence?(frameSequence) ?? frontiers
    }
}

@MainActor
private final class FakeVoice: RoverVoice {
    private(set) var spoken: [String] = []
    var scriptedReplies: [String?] = []

    func speak(_ text: String) { spoken.append(text) }

    func ask(_ question: String, timeout: TimeInterval) async -> String? {
        spoken.append(question)
        return scriptedReplies.isEmpty ? nil : scriptedReplies.removeFirst()
    }
}
