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

    func testLooksAroundWhenNothingVisible() async {
        let motion = FakeMotion()
        let perception = FakePerception()
        let voice = FakeVoice()
        let brain = FakeBrain(script: [.lookAround(angle: .pi), .done])
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }

        await agent.handle("go to the chair")

        XCTAssertEqual(motion.rotateCalls, [.pi])
    }

    func testDoesNotDriveToOpeningWhenVisualTargetIsNotCurrentlyDetected() async {
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
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(voice.spoken, ["I couldn't quite figure out where that is."])
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

        XCTAssertEqual(motion.rotateCalls, [.pi / 6, -.pi / 3])
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(brain.seenContexts.count, 2)
        XCTAssertEqual(voice.spoken, ["I couldn't quite figure out where that is."])
        XCTAssertEqual(agent.phase, .idle)
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
                                 visualTargetScanDelay: 0.05,
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

        XCTAssertEqual(motion.cancelCallCount, 1)
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
        XCTAssertEqual(motion.rotateCalls, [.pi / 6, -.pi / 3])
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
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { nil }

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
        let perception = FakePerception()
        perception.pose = nil
        let voice = FakeVoice()
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { nil }

        await agent.handle("stop")

        XCTAssertEqual(motion.cancelCallCount, 1)
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

        XCTAssertEqual(motion.cancelCallCount, 1)
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

        XCTAssertEqual(motion.cancelCallCount, 1)
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
                                 currentBrain: { brain })

        await agent.handle("go to the refrigerator")

        XCTAssertEqual(brain.seenContexts.map(\.utterance), ["go to the refrigerator"])
        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertEqual(voice.spoken, ["Sorry, I'm having trouble thinking right now."])
        XCTAssertEqual(agent.phase, .idle)
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

    init(error: Error) { self.error = error }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        throw error
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
    /// What `state` settles to shortly after `navigate(to:)` — simulates the real drive
    /// loop reaching `.arrived` asynchronously.
    var navigateOutcome: NavigationController.State = .arrived
    var navigateOutcomes: [NavigationController.State] = []
    var rotateNeverCompletes = false
    var rotateDelay: TimeInterval = 0
    var onNavigate: ((Int) -> Void)?
    var onRotate: ((Double) -> Void)?
    var onScanRotate: ((Double) -> Void)?

    func navigate(to goal: Vec2) {
        navigateCalls.append(goal)
        onNavigate?(navigateCalls.count)
        state = .driving
        let outcome = navigateOutcomes.isEmpty ? navigateOutcome : navigateOutcomes.removeFirst()
        Task { @MainActor in
            try? await Task.sleep(for: .milliseconds(20))
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
        await rotate(by: angle)
    }

    func cancel() {
        cancelCallCount += 1
        state = .idle
    }
}

@MainActor
private final class FakePerception: RoverPerception {
    var pose: Pose2D? = Pose2D(position: .zero, yaw: 0)
    var objects: [PerceivedObject] = []
    var frontiers: [Frontier] = []
    var unprojectResult: Vec2? = Vec2(1, 2)

    func detectObjects() -> [PerceivedObject] { objects }
    func unproject(normalizedPoint: CGPoint) -> Vec2? { unprojectResult }
    func capturedFrameJPEG() -> Data? { nil }
    func explorationFrontiers() -> [Frontier] { frontiers }
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
