import Foundation
import RoverNav
import XCTest
@testable import PhroverKit

@MainActor
final class FollowTargetTrackerTests: XCTestCase {
    private let tracker = FollowTargetTracker()
    private let now: TimeInterval = 10

    func testInitialSelectionRequiresConfidenceAndFreshnessAtInclusiveBoundaries() {
        let lowConfidence = person(1, confidence: 0.499, box: box(0.5, 0.5))
        let stale = person(2, timestamp: 9.499, box: box(0.5, 0.5))
        let future = person(3, timestamp: 10.001, box: box(0.5, 0.5))
        let boundary = person(4, timestamp: 9.5, confidence: 0.5, box: box(0.7, 0.5))

        XCTAssertNil(tracker.selectInitial([lowConfidence, stale, future], now: now))
        XCTAssertEqual(tracker.selectInitial([lowConfidence, stale, future, boundary], now: now)?.frameID, boundary.frameID)
    }

    func testInitialSelectionChoosesEligibleBoxNearestImageCenter() {
        let outer = person(1, box: box(0.1, 0.1))
        let near = person(2, box: box(0.55, 0.5))
        let ineligibleCenter = person(3, confidence: 0.4, box: box(0.5, 0.5))

        XCTAssertEqual(tracker.selectInitial([outer, near, ineligibleCenter], now: now)?.frameID, near.frameID)
    }

    func testContinuationRetainsOffCenterLockInsteadOfSelectingNewCentralPerson() {
        let previous = person(1, box: box(0.7, 0.5), position: Vec2(2, 0))
        let newcomer = person(2, box: box(0.5, 0.5), position: Vec2(5, 0))
        let locked = person(3, box: box(0.72, 0.5), position: Vec2(2.2, 0))

        assertMatched(tracker.continueTrack([newcomer, locked], previous: previous,
                                            predictedPosition: Vec2(2, 0), now: now), frameID: locked.frameID)
    }

    func testContinuationRequiresWorldAndScreenGateAndReportsLossOrAmbiguity() {
        let previous = person(1, box: box(0.2, 0.5), position: Vec2(0, 0))
        let overlappingJump = person(2, box: box(0.2, 0.5), position: Vec2(0.751, 0))
        let nearbyOffscreen = person(3, box: box(0.8, 0.5), position: Vec2(0.1, 0))
        let matchA = person(4, box: box(0.2, 0.5), position: Vec2(0.75, 0))
        let matchB = person(5, box: box(0.22, 0.5), position: Vec2(0.2, 0))

        assertLost(tracker.continueTrack([overlappingJump, nearbyOffscreen], previous: previous,
                                         predictedPosition: .zero, now: now))
        assertMatched(tracker.continueTrack([matchA], previous: previous,
                                            predictedPosition: .zero, now: now), frameID: matchA.frameID)
        assertAmbiguous(tracker.continueTrack([matchA, matchB], previous: previous,
                                              predictedPosition: .zero, now: now))
    }

    func testContinuationAcceptsScreenDisplacementWithoutOverlapButRejectsStaleAndLowConfidence() {
        let previous = person(1, box: box(0.2, 0.5))
        let closeOnScreen = person(2, box: box(0.45, 0.5), position: Vec2(0.1, 0))
        let stale = person(3, timestamp: 9.499, box: box(0.2, 0.5))
        let weak = person(4, confidence: 0.499, box: box(0.2, 0.5))

        assertMatched(tracker.continueTrack([closeOnScreen, stale, weak], previous: previous,
                                            predictedPosition: .zero, now: now), frameID: closeOnScreen.frameID)
    }

    func testReacquisitionAcceptsExactlyOneFreshNearbyCandidateOnly() {
        let centralDistant = person(1, box: box(0.5, 0.5), position: Vec2(1.501, 0))
        let nearby = person(2, timestamp: 9.5, confidence: 0.5,
                            box: box(0.9, 0.5), position: Vec2(1.5, 0))
        let second = person(3, box: box(0.1, 0.5), position: Vec2(0.3, 0))
        let stale = person(4, timestamp: 9.499, box: box(0.5, 0.5))
        let weak = person(5, confidence: 0.499, box: box(0.5, 0.5))

        assertLost(tracker.reacquire([centralDistant, stale, weak], lastPosition: .zero, now: now))
        assertMatched(tracker.reacquire([centralDistant, nearby], lastPosition: .zero, now: now),
                      frameID: nearby.frameID)
        assertAmbiguous(tracker.reacquire([nearby, second], lastPosition: .zero, now: now))
    }

    func testStandOffGoalStopsShortOfPerson() {
        guard let goal = tracker.standOffGoal(rover: Vec2(0, 0), person: Vec2(0, 4)) else {
            return XCTFail("Expected a stand-off goal")
        }
        XCTAssertEqual(goal.x, 0, accuracy: 0.001)
        XCTAssertEqual(goal.y, 2.5, accuracy: 0.001)
        XCTAssertNil(tracker.standOffGoal(rover: Vec2(0, 0), person: Vec2(0, 1)))
        XCTAssertNil(tracker.standOffGoal(rover: .zero, person: Vec2(1.75, 0)))
    }

    private func box(_ x: CGFloat, _ y: CGFloat) -> CGRect {
        CGRect(x: x - 0.05, y: y - 0.1, width: 0.1, height: 0.2)
    }

    private func person(_ sequence: UInt64, timestamp: TimeInterval = 10, confidence: Float = 0.9,
                        box: CGRect, position: Vec2 = .zero) -> FollowPersonObservation {
        FollowPersonObservation(frameID: ARFrameID(generation: 1, sequence: sequence), timestamp: timestamp,
                                confidence: confidence, boundingBox: box, position: position,
                                pose: Pose2D(position: .zero, yaw: 0))
    }

    private func assertMatched(_ result: FollowTrackMatch, frameID: ARFrameID,
                               file: StaticString = #filePath, line: UInt = #line) {
        guard case .matched(let observation) = result else {
            return XCTFail("Expected matched", file: file, line: line)
        }
        XCTAssertEqual(observation.frameID, frameID, file: file, line: line)
    }

    private func assertLost(_ result: FollowTrackMatch, file: StaticString = #filePath, line: UInt = #line) {
        guard case .lost = result else { return XCTFail("Expected lost", file: file, line: line) }
    }

    private func assertAmbiguous(_ result: FollowTrackMatch, file: StaticString = #filePath, line: UInt = #line) {
        guard case .ambiguous = result else { return XCTFail("Expected ambiguous", file: file, line: line) }
    }
}
