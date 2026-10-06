import XCTest
import RoverNav
@testable import PhroverKit

final class SharedMissionFrameTests: XCTestCase {
    func testIdentityFrameConvertsMissionPointAndPoseToLocalCoordinates() throws {
        let frame = try XCTUnwrap(SharedMissionFrame(
            localOrigin: Vec2(0, 0),
            localNorthHeading: .pi / 2,
            sessionGeneration: 7
        ))
        let point = try XCTUnwrap(MissionPoint(x: 2, y: 3))
        let pose = try XCTUnwrap(MissionPose(position: point, heading: 0))

        XCTAssertEqual(frame.localPoint(from: point).x, 2, accuracy: 1e-12)
        XCTAssertEqual(frame.localPoint(from: point).y, 3, accuracy: 1e-12)
        XCTAssertEqual(frame.localPose(from: pose).position.x, 2, accuracy: 1e-12)
        XCTAssertEqual(frame.localPose(from: pose).position.y, 3, accuracy: 1e-12)
        XCTAssertEqual(frame.localPose(from: pose).yaw, .pi / 2, accuracy: 1e-12)
    }

    func testTranslatedAndRotatedFrameConvertsInBothDirections() throws {
        let frame = try XCTUnwrap(SharedMissionFrame(
            localOrigin: Vec2(10, -4),
            localNorthHeading: 0,
            sessionGeneration: 3
        ))
        let mission = try XCTUnwrap(MissionPoint(x: 2, y: 3))

        let local = frame.localPoint(from: mission)
        XCTAssertEqual(local.x, 13, accuracy: 1e-12)
        XCTAssertEqual(local.y, -6, accuracy: 1e-12)
        XCTAssertEqual(frame.missionPoint(from: local), mission)
    }

    func testMissionHeadingIsCounterclockwiseFromNorthNotPoseYawFromLocalX() throws {
        let frame = try XCTUnwrap(SharedMissionFrame(
            localOrigin: .zero,
            localNorthHeading: -.pi / 4,
            sessionGeneration: 1
        ))
        let missionPose = try XCTUnwrap(MissionPose(
            position: XCTUnwrap(MissionPoint(x: 0, y: 0)),
            heading: .pi / 2
        ))

        let localPose = frame.localPose(from: missionPose)
        XCTAssertEqual(localPose.yaw, .pi / 4, accuracy: 1e-12)
        XCTAssertEqual(frame.missionPose(from: localPose), missionPose)
    }

    func testHeadingsNormalizeAtWraparound() throws {
        let pose = try XCTUnwrap(MissionPose(
            position: XCTUnwrap(MissionPoint(x: 0, y: 0)),
            heading: 3 * .pi
        ))
        let frame = try XCTUnwrap(SharedMissionFrame(
            localOrigin: .zero,
            localNorthHeading: -3 * .pi,
            sessionGeneration: 1
        ))

        XCTAssertEqual(pose.heading, .pi, accuracy: 1e-12)
        XCTAssertEqual(frame.localNorthHeading, .pi, accuracy: 1e-12)
    }

    func testPointAndPoseRoundTrip() throws {
        let frame = try XCTUnwrap(SharedMissionFrame(
            localOrigin: Vec2(-1.25, 8.5),
            localNorthHeading: 2.2,
            sessionGeneration: 99
        ))
        let point = try XCTUnwrap(MissionPoint(x: -3.1, y: 0.7))
        let pose = try XCTUnwrap(MissionPose(position: point, heading: -2.8))

        let pointRoundTrip = try XCTUnwrap(frame.missionPoint(from: frame.localPoint(from: point)))
        let poseRoundTrip = try XCTUnwrap(frame.missionPose(from: frame.localPose(from: pose)))

        XCTAssertEqual(pointRoundTrip.x, point.x, accuracy: 1e-12)
        XCTAssertEqual(pointRoundTrip.y, point.y, accuracy: 1e-12)
        XCTAssertEqual(poseRoundTrip.position.x, pose.position.x, accuracy: 1e-12)
        XCTAssertEqual(poseRoundTrip.position.y, pose.position.y, accuracy: 1e-12)
        XCTAssertEqual(poseRoundTrip.heading, pose.heading, accuracy: 1e-12)
    }

    func testRejectsNonFiniteCoordinatesAndHeadings() {
        XCTAssertNil(MissionPoint(x: .infinity, y: 0))
        XCTAssertNil(MissionPoint(x: 0, y: .nan))
        XCTAssertNil(MissionPose(position: MissionPoint(x: 0, y: 0)!, heading: .infinity))
        XCTAssertNil(SharedMissionFrame(localOrigin: Vec2(.nan, 0), localNorthHeading: 0, sessionGeneration: 1))
        XCTAssertNil(SharedMissionFrame(localOrigin: .zero, localNorthHeading: .infinity, sessionGeneration: 1))
    }

    func testFrameValidityIsScopedToSessionGeneration() throws {
        let frame = try XCTUnwrap(SharedMissionFrame(
            localOrigin: .zero,
            localNorthHeading: 0,
            sessionGeneration: 42
        ))

        XCTAssertTrue(frame.isValid(forSessionGeneration: 42))
        XCTAssertFalse(frame.isValid(forSessionGeneration: 43))
    }

    func testRolesSectorsAndFixedGeometry() throws {
        XCTAssertEqual(RoverRole.a.searchSector, .west)
        XCTAssertEqual(RoverRole.b.searchSector, .east)
        XCTAssertEqual(SilentSearchGeometry.centerBandHalfWidth, 0.25, accuracy: 1e-12)
        XCTAssertEqual(SilentSearchGeometry.rendezvousPoint(for: .a), MissionPoint(x: -0.60, y: -0.80))
        XCTAssertEqual(SilentSearchGeometry.rendezvousPoint(for: .b), MissionPoint(x: 0.60, y: -0.80))
        XCTAssertEqual(SilentSearchGeometry.targetOffset, 0.60, accuracy: 1e-12)
        XCTAssertEqual(SilentSearchGeometry.positionTolerance, 0.20, accuracy: 1e-12)
        XCTAssertEqual(SilentSearchGeometry.headingTolerance, 10 * .pi / 180, accuracy: 1e-12)
    }
}
