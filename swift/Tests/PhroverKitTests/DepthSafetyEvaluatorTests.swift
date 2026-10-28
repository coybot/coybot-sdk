import CoreVideo
import simd
import XCTest
import RoverNav
@testable import PhroverKit

final class DepthSafetyEvaluatorTests: XCTestCase {
    func testEnteringSideInteriorIsCheckedEvenWhenItsCornerAlsoBelongsToFront() throws {
        for mirrored in [false, true] {
            let shape = RoverCollisionGeometry(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04,
                maximumCollisionHeight: 0.34, sections: [
                    .init(name: "body", length: 0.30, width: 0.25, minimumHeight: 0.04, maximumHeight: 0.34)])
            let snapshot = DepthSafetyEvaluator.ingest(
                rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
                cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
                calibration: RoverConfig.cameraMountCalibration, geometry: shape)
            let command = WheelCommand(left: mirrored ? 0.25 : 0.20, right: mirrored ? 0.20 : 0.25)
            let observation = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05)
            let coverage = try XCTUnwrap(observation.coverage)
            let side = mirrored ? "right_side" : "left_side"
            let interior = try XCTUnwrap(coverage.points.first {
                $0.boundary == side && $0.travel == 0 && abs($0.roverPosition.z - 0.075) < 1e-12
            })
            XCTAssertEqual(interior.support, .outsideView)
            XCTAssertTrue(coverage.boundaries.contains { $0.boundary == side && $0.required > 0 })
            XCTAssertEqual(observation.state, .unavailable(.blindSweptVolume))
        }
    }

    func testTightMirroredArcsCheckEnteringRearSubsegment() throws {
        let shape = RoverCollisionGeometry(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04,
            maximumCollisionHeight: 0.34, sections: [
                .init(name: "body", length: 0.30, width: 0.25, minimumHeight: 0.04, maximumHeight: 0.34)])
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: shape)
        for mirrored in [false, true] {
            let result = DepthSafetyEvaluator.evaluate(snapshot,
                command: .init(left: mirrored ? 0.25 : 0, right: mirrored ? 0 : 0.25), now: 10.05)
            let coverage = try XCTUnwrap(result.coverage)
            // Instantaneous center is +/-6.5cm; the entering rear subsegment
            // spans that point to the +/-20.5cm safety edge, midpoint +/-13.5cm.
            XCTAssertTrue(coverage.points.contains {
                $0.boundary == "rear" && $0.travel == 0 &&
                    abs($0.roverPosition.x - (mirrored ? 0.135 : -0.135)) < 1e-12
            })
            XCTAssertEqual(result.state, .unavailable(.blindSweptVolume))
        }
    }

    func testOffsetUpperSectionCannotAuthorizeArcWithUnseenEnteringSide() throws {
        let intrinsics = simd_float3x3(columns: (SIMD3<Float>(25, 0, 0), SIMD3<Float>(0, 8, 0), SIMD3<Float>(40, 30, 1)))
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: intrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: offsetSectionGeometry())
        let observation = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: 0.20, right: 0.25), now: 10.05)
        // Front-edge-only coverage was 100% chassis and 98.1% upper. Yet the
        // upper rear-right boundary enters space outside the image during the arc.
        XCTAssertEqual(observation.state, .unavailable(.blindSweptVolume))
        let coverage = try XCTUnwrap(observation.coverage)
        XCTAssertTrue(coverage.points.contains {
            $0.section == "upper" && $0.boundary == "right_side" && $0.support == .outsideView
        })
    }

    func testConfirmedRotationUsesHeightSectionEnvelopeAndVoxelMargin() {
        func observation(height: Int) -> DepthSafetyObservation {
            let cells = Set((0...2).map { DepthSafetyCell(lateralIndex: $0, heightIndex: height, forwardIndex: 13) })
            return DepthSafetyEvaluator.evaluate(.init(timestamp: 10, cells: cells, cellResolution: 0.04,
                validSampleRatio: 1, geometry: offsetSectionGeometry(), projection: nil, failureReason: nil),
                command: .init(left: -0.25, right: 0.25), now: 10.05, operatorClearedRotation: true)
        }
        XCTAssertEqual(observation(height: 2).state, .stop)
        XCTAssertEqual(observation(height: 5).state, .caution)
        XCTAssertGreaterThan(observation(height: 5).clearance, 0.30)
        XCTAssertEqual(observation(height: 5).coverageProvenance, "operator_confirmed_rotation")
    }

    func testMirroredCurvesReachInitiallySeparatedOffsetUpperObstacle() {
        for mirrored in [false, true] {
            let command = WheelCommand(left: mirrored ? 0.25 : 0.20, right: mirrored ? 0.20 : 0.25)
            let shape = offsetSectionGeometry(lateralOffset: mirrored ? -0.03 : 0.03)
            func observation(lateral: Int, forward: Int) -> DepthSafetyObservation {
                let cells = Set((5...7).map { DepthSafetyCell(lateralIndex: lateral, heightIndex: $0, forwardIndex: forward) })
                return DepthSafetyEvaluator.evaluate(.init(timestamp: 10, cells: cells, cellResolution: 0.04,
                    validSampleRatio: 1, geometry: shape, projection: nil, failureReason: nil), command: command, now: 10.05)
            }
            let hit = observation(lateral: mirrored ? 3 : -4, forward: 5)
            XCTAssertEqual(hit.state, .stop)
            XCTAssertEqual(hit.supportCount, 3)
            // Independently worked first contact: continuous travel .256813m;
            // the 33-interval sweep first intersects at step14 (.275758m).
            XCTAssertEqual(hit.clearance, 14 * 0.65 / 33, accuracy: 1e-10)
            let miss = observation(lateral: mirrored ? 5 : -6, forward: 6)
            XCTAssertEqual(miss.supportCount, 0)
            XCTAssertEqual(miss.clearance, .infinity)
        }
    }

    @MainActor
    func testRealSenderLogsCoveragePointsWithoutSendingBlindMotion() async throws {
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        let sink = FollowDiagnosticRecordingSink()
        var sends = 0
        let pose = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
            frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 10, trackingQuality: .normal)
        let controller = FollowNavigationController(currentPose: { pose.pose }, forwardClearance: { 2 },
            depthSafetyObservation: { DepthSafetyEvaluator.evaluate(snapshot, command: $0, now: 10.05, boundedReadyMotion: true) },
            plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { _ in },
            diagnosticEmitter: .init(streamID: "coverage-projection", monotonic: { 10.05 }, utc: { Date() }, sink: sink.append),
            poseSample: { pose }, sourceNow: { 10.05 }, sourceStopSnapshot: { pose })
        _ = await controller.sendFollowTurnBurst(.init(left: 0.25, right: 0.25), requestedBudget: 0.02, purpose: .followReady)
        XCTAssertEqual(sends, 0)
        let payload = try XCTUnwrap(sink.records.first { $0.event == "follow_depth.candidate" }?.fields["payload"])
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any])
        let points = try XCTUnwrap(json["depth_coverage_points"] as? [[String: Any]])
        XCTAssertEqual(points.count, 81)
        XCTAssertEqual(json["depth_coverage_points_truncated"] as? Bool, false)
        XCTAssertEqual(json["depth_geometry_model"] as? String, "conservative_bounding_box")
        XCTAssertEqual((json["depth_projection_camera_from_world_column_major"] as? [Double])?.count, 16)
        XCTAssertTrue(points.contains { ($0["outside_edges"] as? [String])?.isEmpty == false })
        let projection = try XCTUnwrap(json["depth_projection"] as? [String: Any])
        XCTAssertEqual(projection["camera_forward_offset_m"] as? Double ?? 0, -0.085, accuracy: 1e-6)
    }

    func testUnseenThinComponentCannotBeAveragedAwayOrHiddenByDiagnosticCap() throws {
        var sections: [RoverCollisionGeometry.Section] = []
        for index in 0..<7 {
            sections.append(.init(name: "lower_\(index)", length: 0.30, width: 0.25,
                minimumHeight: index == 0 ? 0.04 : 0.04 + 0.44 * Double(index) / 7,
                maximumHeight: index == 6 ? 0.48 : 0.04 + 0.44 * Double(index + 1) / 7))
        }
        sections.append(.init(name: "top", length: 0.30, width: 0.25, minimumHeight: 0.48, maximumHeight: 0.50))
        let shape = RoverCollisionGeometry(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04,
            maximumCollisionHeight: 0.50, sections: sections)
        XCTAssertEqual(shape.validation, .valid)
        // Deliberately synthetic camera sees everything below its own height,
        // but none of the thin top section. This is not an iPhone FOV claim.
        let intrinsics = simd_float3x3(columns: (SIMD3<Float>(10, 0, 0), SIMD3<Float>(0, 100, 0), SIMD3<Float>(40, 0, 1)))
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 200, constantDepth: 3), intrinsics: intrinsics,
            cameraTransform: cameraTransform(height: 0.48), timestamp: 10,
            calibration: .init(cameraHeight: 0.48, forwardOffset: -0.085), geometry: shape)
        let observation = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: 0.25, right: 0.25), now: 10.05, boundedReadyMotion: true)
        let coverage = try XCTUnwrap(observation.coverage)
        XCTAssertGreaterThan(coverage.fraction, 0.90)
        XCTAssertEqual(coverage.limitingFraction, 0)
        XCTAssertEqual(observation.state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(coverage.points.count, 256)
        XCTAssertGreaterThan(coverage.requiredPoints, coverage.points.count)
        XCTAssertEqual(coverage.components.last?.name, "top")
        XCTAssertEqual(coverage.components.last?.covered, 0)
        XCTAssertTrue(coverage.points.contains { $0.section == "top" && $0.support == .outsideView })
        let fields = coverage.diagnosticFields(includePoints: true)
        XCTAssertEqual(fields["depth_coverage_points_truncated"], .bool(true))
        XCTAssertEqual(fields["depth_limiting_component_fraction"], .number(0))
        XCTAssertNil(coverage.diagnosticFields(includePoints: false)["depth_coverage_points"])
    }

    func testHeightDependentFootprintsKeepChassisAndUpperAssemblyHazards() {
        let shape = RoverCollisionGeometry(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04,
            maximumCollisionHeight: 0.34, sections: [
                .init(name: "chassis", length: 0.30, width: 0.25, minimumHeight: 0.04, maximumHeight: 0.14),
                .init(name: "upper", length: 0.08, width: 0.10, minimumHeight: 0.14, maximumHeight: 0.34, forwardOffset: -0.085)])
        let projection = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: shape).projection
        func observation(height: Int, lateral: Int, command: WheelCommand = .init(left: 0.25, right: 0.25)) -> DepthSafetyObservation {
            let cells = Set((2...4).map { DepthSafetyCell(lateralIndex: lateral, heightIndex: height, forwardIndex: $0) })
            let snapshot = DepthSafetySnapshot(timestamp: 10, cells: cells, cellResolution: 0.04,
                validSampleRatio: 1, geometry: shape, projection: projection, failureReason: nil)
            return DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05, boundedReadyMotion: true)
        }
        XCTAssertEqual(observation(height: 2, lateral: 4).state, .stop, "Wide chassis edge must still stop")
        XCTAssertEqual(observation(height: 5, lateral: 0).state, .stop, "Narrow upper assembly still collides head-on")
        XCTAssertEqual(observation(height: 3, lateral: 4).state, .stop, "Boundary voxel must check both sections")
        XCTAssertNotEqual(observation(height: 5, lateral: 4).state, .stop, "Empty upper corner is not a chassis-height obstacle")
        XCTAssertEqual(observation(height: 2, lateral: 4, command: .init(left: 0.20, right: 0.25)).state, .stop)
        XCTAssertEqual(observation(height: 5, lateral: 0, command: .init(left: -0.25, right: 0.25)).state, .stop)
    }

    func testCoverageReportsExactSamplePositionsAndOutOfViewEdges() throws {
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        let result = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: 0.25, right: 0.25),
            now: 10.05, boundedReadyMotion: true)
        let coverage = try XCTUnwrap(result.coverage)
        XCTAssertEqual(coverage.points.count, 81)
        XCTAssertEqual(coverage.points.filter { $0.support == .covered }.count, coverage.coveredPoints)
        XCTAssertEqual(coverage.points.filter { $0.support == .outsideView }.count, coverage.outsideViewPoints)
        XCTAssertEqual(Set(coverage.points.map(\.boundary)), ["left", "center", "right"])
        XCTAssertEqual(Set(coverage.points.map { $0.roverPosition.y }), [0.06, 0.27, 0.48])
        XCTAssertEqual(coverage.points.first?.roverPosition.z, 0.15)
        XCTAssertTrue(coverage.points.filter { $0.support == .outsideView }.allSatisfy { !$0.outsideEdges.isEmpty })
        XCTAssertEqual(coverage.projectionSummary["image_width_px"], 80)
        XCTAssertEqual(coverage.projectionSummary["camera_forward_offset_m"] ?? 0, -0.085, accuracy: 1e-6)
        XCTAssertEqual(coverage.cameraFromWorldColumnMajor.count, 16)
        XCTAssertEqual(coverage.components.first?.required, 81)
    }

    func testRearwardMeasuredMountProjectsChassisAheadOfCameraAndKeepsObstacleStop() throws {
        let command = WheelCommand(left: 0.25, right: 0.25)
        func snapshot(offset: Double, wallDepth: Float = 3) -> DepthSafetySnapshot {
            DepthSafetyEvaluator.ingest(rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: wallDepth),
                intrinsics: ordinaryIntrinsics, cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.30, forwardOffset: offset), geometry: geometry)
        }
        let centered = DepthSafetyEvaluator.evaluate(snapshot(offset: 0), command: command,
            now: 10.05, boundedReadyMotion: true)
        for offset in [-0.07, -0.085, -0.10] {
            let captured = snapshot(offset: offset)
            let projection = try XCTUnwrap(captured.projection)
            // Identity camera looks toward world -Z. A camera behind the chassis
            // puts the chassis origin and leading boundary farther down -Z.
            XCTAssertEqual(projection.roverOrigin.z, offset, accuracy: 1e-9)
            let front = projection.roverOrigin + projection.roverForward * 0.15
            XCTAssertEqual(-front.z, 0.15 - offset, accuracy: 1e-9)
            let measured = DepthSafetyEvaluator.evaluate(captured, command: command, now: 10.05, boundedReadyMotion: true)
            XCTAssertGreaterThan(measured.coverage?.fraction ?? 0, centered.coverage?.fraction ?? 0)
            XCTAssertEqual(measured.state, .unavailable(.blindSweptVolume),
                "Synthetic ordinary-FOV visibility still cannot prove 90% coverage merely by correcting the mount")
            let obstacle = DepthSafetyEvaluator.evaluate(snapshot(offset: offset, wallDepth: 0.35),
                command: command, now: 10.05, boundedReadyMotion: true)
            XCTAssertEqual(obstacle.state, .stop)
            XCTAssertGreaterThanOrEqual(obstacle.supportCount, 3)
            XCTAssertLessThanOrEqual(obstacle.clearance, 0.30)
        }
    }

    func testProductionDepthProjectionUsesMeasuredRearwardLensPosition() throws {
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: RoverConfig.collisionGeometry)
        let projection = try XCTUnwrap(snapshot.projection)
        XCTAssertEqual(projection.roverOrigin.z, -0.085, accuracy: 1e-9)
        XCTAssertEqual(projection.roverOrigin.y, 0, accuracy: 1e-6)
    }

    func testBoundedProbeUsesObservedStoppingCorridorWithoutCruiseLookahead() {
        // Real projection/evaluator, deliberately incomplete depth outside the
        // stopping corridor. This is a synthetic regression, not device imagery.
        let map = stoppingCorridorDepth()
        let snapshot = DepthSafetyEvaluator.ingest(rawDepthMap: map, intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let normal = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05)
        XCTAssertEqual(normal.state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(normal.coverage?.horizon ?? 0, 0.65, accuracy: 1e-12)
        XCTAssertGreaterThan(normal.coverage?.missingDepthPoints ?? 0, 0)
        let probe = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(probe.state, .clear)
        XCTAssertEqual(probe.requiredStoppingDistance, 0.30)
        XCTAssertEqual(probe.coverage?.horizon, 0.30)
        XCTAssertGreaterThanOrEqual(probe.coverage?.fraction ?? 0, 0.90)
        XCTAssertEqual(probe.coverageProvenance, "bounded_ready_motion_stopping_corridor")
    }

    @MainActor
    func testARDepthProviderUsesProbeScopeOnlyDuringExplicitAssessment() {
        let ar = ARSessionManager()
        ar.ingestDepthSafety(rawDepthMap: stoppingCorridorDepth(), intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let probe = ReadyMotionDepthScope.$active.withValue(true) {
            ar.depthSafetyObservation(for: command, at: 10.05)
        }
        XCTAssertEqual(probe.state, .clear)
        XCTAssertEqual(ar.depthSafetyObservation(for: command, at: 10.05).state, .unavailable(.blindSweptVolume))
        let stale = ReadyMotionDepthScope.$active.withValue(true) {
            ar.depthSafetyObservation(for: command, at: 10.251)
        }
        XCTAssertEqual(stale.state, .unavailable(.staleRawDepth))
    }

    func testProbeCannotClearNearBlindSpaceOrKnownStoppingCorridorObstacle() {
        let nearBlind = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let blind = DepthSafetyEvaluator.evaluate(nearBlind, command: command, now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(blind.state, .unavailable(.blindSweptVolume))
        XCTAssertGreaterThan(blind.coverage?.outsideViewPoints ?? 0, 0)
        XCTAssertEqual(blind.coverage?.missingDepthPoints, 0)
        let cells = Set((1...3).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 10) })
        let obstacle = DepthSafetySnapshot(timestamp: 10, cells: cells, cellResolution: 0.04,
            validSampleRatio: 1, geometry: geometry, projection: nearBlind.projection, failureReason: nil)
        let blocked = DepthSafetyEvaluator.evaluate(obstacle, command: command, now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(blocked.state, .stop)
        XCTAssertEqual(blocked.requiredStoppingDistance, 0.30)
        for other in [WheelCommand(left: 0.5, right: 0.5), .init(left: 0.2, right: 0.25),
                      .init(left: -0.25, right: -0.25), .init(left: -0.25, right: 0.25)] {
            let result = DepthSafetyEvaluator.evaluate(nearBlind, command: other, now: 10.05, boundedReadyMotion: true)
            XCTAssertNotEqual(result.coverageProvenance, "bounded_ready_motion_stopping_corridor")
        }
    }

    private func stoppingCorridorDepth() -> CVPixelBuffer {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: .nan)
        for index in 0...8 {
            let z = 0.15 + 0.30 * Double(index) / 8 - RoverConfig.cameraMountCalibration.forwardOffset
            for x in [-0.205, 0, 0.205] {
                for height in [0.06, 0.27, 0.48] {
                    let px = Int((40 + 20 * x / z).rounded())
                    let py = Int((30 - 8 * (height - 0.30) / z).rounded())
                    fill(depth: 3, x: px...px, y: py...py, in: map)
                }
            }
        }
        return map
    }

    private let geometry = RoverCollisionGeometry(
        length: 0.30,
        width: 0.25,
        minimumCollisionHeight: 0.04,
        maximumCollisionHeight: 0.50
    )

    func testConfirmedRotationStopsAtThirtyCentimeterRadialGap() {
        // Centers at (+/-2 cm, 54 cm), with a 25.4 cm swept chassis radius
        // and 2.83 cm voxel uncertainty: conservative gap is about 25.8 cm.
        let cells = Set((2...4).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 13) })
        let snapshot = DepthSafetySnapshot(timestamp: 10, cells: cells, cellResolution: 0.04,
            validSampleRatio: 1, geometry: geometry, projection: nil, failureReason: nil)
        let result = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true)
        XCTAssertEqual(result.state, .stop)
        XCTAssertLessThanOrEqual(result.clearance, 0.30)
    }

    func testConfirmedRotationRadialBoundaryCautionAndDynamicStopping() {
        func evaluate(_ index: Int, speed: Double = 0.25, authorized: Bool = true,
                      command: WheelCommand? = nil) -> DepthSafetyObservation {
            let cells = Set((2...4).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: index) })
            let snapshot = DepthSafetySnapshot(timestamp: 10, cells: cells, cellResolution: 0.04,
                validSampleRatio: 1, geometry: geometry, projection: nil, failureReason: nil)
            return DepthSafetyEvaluator.evaluate(snapshot, command: command ?? .init(left: -speed, right: speed),
                now: 10.05, operatorClearedRotation: authorized)
        }
        // 58 cm center minus 25.402 cm chassis sweep and 2.828 cm cell
        // uncertainty gives 29.804 cm: including uncertainty is decisive.
        let inside = evaluate(14)
        XCTAssertEqual(inside.state, .stop)
        XCTAssertEqual(inside.clearance, 0.29804, accuracy: 0.0001)
        let outside = evaluate(15)
        XCTAssertEqual(outside.state, .caution)
        XCTAssertEqual(outside.clearance, 0.33801, accuracy: 0.0001)
        XCTAssertEqual(outside.coverageProvenance, "operator_confirmed_rotation")
        XCTAssertEqual(outside.speedLimit, 0.12)
        XCTAssertEqual(evaluate(15, authorized: false).state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(evaluate(15, speed: 0.8).state, .stop, "Dynamic distance must exceed the 30 cm minimum")
        XCTAssertGreaterThan(evaluate(15, speed: 0.8).requiredStoppingDistance, 0.30)
        XCTAssertEqual(evaluate(24).state, .clear)
        XCTAssertEqual(evaluate(24, command: .init(left: 0.25, right: 0.25)).state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(evaluate(24, command: .init(left: -0.25, right: -0.25)).state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(evaluate(24, command: .init(left: -0.24, right: 0.25)).state, .unavailable(.blindSweptVolume))
    }

    func testOperatorClearedRotationDoesNotAuthorizeForwardObstacleCorridor() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 1.0, x: 35...44, y: 33...37, in: map)
        let snapshot = DepthSafetyEvaluator.ingest(rawDepthMap: map, intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.55), timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55), geometry: geometry)
        let rotating = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true)
        XCTAssertEqual(rotating.state, .clear)
        XCTAssertEqual(rotating.coverageProvenance, "operator_confirmed_rotation")
        XCTAssertEqual(rotating.sampleAge, 0.05, accuracy: 0.00001)
        let forward = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: 1.2, right: 1.2),
            now: 10.05, operatorClearedRotation: true)
        XCTAssertEqual(forward.state, .stop)
        XCTAssertGreaterThan(forward.requiredStoppingDistance, 0.30)
    }

    func testOperatorClearedRotationRetainsVisibleHazardAndRawDepthVetoes() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.20, x: 30...49, y: 30...59, in: map)
        let snapshot = DepthSafetyEvaluator.ingest(rawDepthMap: map, intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.55), timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55), geometry: geometry)
        XCTAssertEqual(DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true).state, .stop)
        XCTAssertEqual(DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.251, operatorClearedRotation: true).state, .unavailable(.staleRawDepth))
        let malformed = DepthSafetyEvaluator.ingest(rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: .nan),
            intrinsics: ordinaryIntrinsics, cameraTransform: cameraTransform(height: 0.55), timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55), geometry: geometry)
        XCTAssertEqual(DepthSafetyEvaluator.evaluate(malformed, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true).state, .unavailable(.malformedDepth))
    }

    func testFollowSpeedUsesThirtyCentimeterTotalStoppingDistance() {
        let observation = stoppingMarginObservation(speed: 0.25, age: 0.15, obstacleDepth: 0.50)

        XCTAssertEqual(observation.requiredStoppingDistance, 0.30, accuracy: 0.0001)
        XCTAssertGreaterThan(observation.clearance, 0.30)
        XCTAssertLessThan(observation.clearance, 0.38)
        XCTAssertEqual(observation.state, .caution)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testObstacleInsideThirtyCentimetersStillStops() {
        let observation = stoppingMarginObservation(speed: 0.25, age: 0.15, obstacleDepth: 0.43)

        XCTAssertEqual(observation.requiredStoppingDistance, 0.30, accuracy: 0.0001)
        XCTAssertLessThanOrEqual(observation.clearance, 0.30)
        XCTAssertEqual(observation.state, .stop)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testHigherSpeedRetainsLatencyAndBrakingDistanceBeyondThirtyCentimeters() {
        let observation = stoppingMarginObservation(speed: 0.50, age: 0.20, obstacleDepth: 0.50)

        // 0.50 m/s * (0.20 s sample age + 0.25 s latency), plus 0.15625 m braking.
        XCTAssertEqual(observation.requiredStoppingDistance, 0.38125, accuracy: 0.0001)
        XCTAssertGreaterThan(observation.clearance, 0.30)
        XCTAssertLessThan(observation.clearance, observation.requiredStoppingDistance)
        XCTAssertEqual(observation.state, .stop)
    }

    private func stoppingMarginObservation(speed: Double, age: TimeInterval, obstacleDepth: Float) -> DepthSafetyObservation {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: obstacleDepth, x: 35...44, y: 33...37, in: map)
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.55),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )
        return DepthSafetyEvaluator.evaluate(
            snapshot,
            command: WheelCommand(left: speed, right: speed),
            now: 10 + age
        )
    }

    func testVisibleNarrowLowCrossbarProducesCautionInsteadOfLookingPastIt() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        // The wide synthetic frustum sees the full near swept volume. These rows are a low crossbar.
        fill(depth: 0.70, x: 35...44, y: 33...37, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.35, right: 0.35),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .caution)
        XCTAssertEqual(observation.clearance, 0.55, accuracy: 0.08)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testBlindNearSweptVolumeFailsClosedInsteadOfClearing() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: ordinaryIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.20, right: 0.20),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .unavailable(.blindSweptVolume))
    }

    func testSingleNearDepthOutlierDoesNotCreateHazardWhenSweepIsVisible() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.70, x: 40...40, y: 35...35, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.20, right: 0.20),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .clear)
    }

    func testFloorPlaneDoesNotBecomeObstacleWhenConfiguredMountHeightIsTooHigh() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fillFloorPlane(cameraHeight: 0.40, in: map)

        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.40),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )
        let commands = [
            WheelCommand(left: 0.20, right: 0.20),
            WheelCommand(left: 0.15, right: 0.20),
        ]

        for command in commands {
            let observation = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05)
            XCTAssertEqual(observation.state, .clear, "command=\(command)")
        }
        XCTAssertEqual(
            DepthSafetyEvaluator.evaluate(
                snapshot,
                command: WheelCommand(left: 0.10, right: 0.30),
                now: 10.05
            ).state,
            .unavailable(.blindSweptVolume)
        )
    }

    func testThirtyCentimeterProductionMountKeepsVisibleFloorClear() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fillFloorPlane(cameraHeight: 0.30, in: map)

        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30),
            timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration,
            geometry: geometry
        )
        let observation = DepthSafetyEvaluator.evaluate(
            snapshot,
            command: WheelCommand(left: 0.20, right: 0.20),
            now: 10.05
        )

        XCTAssertEqual(RoverConfig.cameraMountCalibration.cameraHeight, 0.30)
        XCTAssertEqual(observation.state, .clear)
    }

    func testThirtyCentimeterProductionMountStillDetectsLowCrossbar() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.70, x: 35...44, y: 31...32, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.30),
                timestamp: 10,
                calibration: RoverConfig.cameraMountCalibration,
                geometry: geometry
            ),
            command: WheelCommand(left: 0.35, right: 0.35),
            now: 10.05
        )

        XCTAssertTrue(observation.state == .caution || observation.state == .stop)
        XCTAssertLessThan(observation.clearance, 0.80)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testIntrinsicsAreScaledFromCameraImageToDepthResolution() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.70, x: 35...44, y: 33...37, in: map)
        let cameraIntrinsics = simd_float3x3(columns: (
            SIMD3<Float>(40, 0, 0),
            SIMD3<Float>(0, 16, 0),
            SIMD3<Float>(80, 60, 1)
        ))

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: cameraIntrinsics,
                intrinsicsImageSize: CGSize(width: 160, height: 120),
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.35, right: 0.35),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .caution)
    }

    func testCurvedCommandUsesArcContactDistance() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.35, x: 27...31, y: 36...40, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.10, right: 0.30),
            now: 10.05
        )

        XCTAssertEqual(observation.motionClass, .curved)
        XCTAssertEqual(observation.state, .stop)
        XCTAssertTrue(observation.clearance.isFinite)
    }

    func testForwardOnlyDepthCannotAuthorizeInPlaceRotation() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: -0.15, right: 0.15),
            now: 10.05
        )

        XCTAssertEqual(observation.motionClass, .rotating)
        XCTAssertEqual(observation.state, .unavailable(.blindSweptVolume))
    }

    func testStaleRawDepthFailsClosed() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.55),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )

        XCTAssertEqual(
            DepthSafetyEvaluator.evaluate(
                snapshot,
                command: WheelCommand(left: 0.20, right: 0.20),
                now: 10.31
            ).state,
            .unavailable(.staleRawDepth)
        )
    }

    func testReverseFailsClosedWithForwardFacingDepth() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.55),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )

        XCTAssertEqual(
            DepthSafetyEvaluator.evaluate(
                snapshot,
                command: WheelCommand(left: -0.20, right: -0.20),
                now: 10.05
            ).state,
            .unavailable(.blindSweptVolume)
        )
    }

    private var ordinaryIntrinsics: simd_float3x3 {
        simd_float3x3(columns: (
            SIMD3<Float>(50, 0, 0),
            SIMD3<Float>(0, 40, 0),
            SIMD3<Float>(40, 30, 1)
        ))
    }

    // MARK: - Remembered visibility (docs/remembered-depth-visibility.md)

    /// The identity camera looks along world -Z; `back` moves it away from the
    /// space ahead of the live rover, as on the approach before a stop.
    private func rememberedFrame(back: Float, age: TimeInterval, depth: Float = 3,
                                 cells: Set<DepthSafetyCell>? = nil) throws -> DepthVisibilityMemory.Frame {
        var transform = cameraTransform(height: 0.30)
        transform.columns.3.z = back
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: depth), intrinsics: ordinaryIntrinsics,
            cameraTransform: transform, timestamp: 10 - age,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        return .init(timestamp: snapshot.timestamp, projection: try XCTUnwrap(snapshot.projection),
                     cells: cells ?? snapshot.cells, cellResolution: snapshot.cellResolution)
    }

    private func liveNearBlindSnapshot(history: [DepthVisibilityMemory.Frame] = [],
                                       depth: CVPixelBuffer? = nil) -> DepthSafetySnapshot {
        var snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: depth ?? makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        snapshot.history = history
        return snapshot
    }

    func testOutOfViewSamplesAreAnsweredByARecentFrameThatSawThem() throws {
        let command = WheelCommand(left: 0.25, right: 0.25)
        let live = DepthSafetyEvaluator.evaluate(liveNearBlindSnapshot(), command: command,
            now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(live.state, .unavailable(.blindSweptVolume))
        XCTAssertGreaterThan(live.coverage?.outsideViewPoints ?? 0, 0)

        let remembered = DepthSafetyEvaluator.evaluate(
            liveNearBlindSnapshot(history: [try rememberedFrame(back: 0.8, age: 0.5)]),
            command: command, now: 10.05, boundedReadyMotion: true)
        let coverage = try XCTUnwrap(remembered.coverage)
        XCTAssertEqual(remembered.state, .clear)
        XCTAssertEqual(coverage.rememberedPoints, live.coverage?.outsideViewPoints)
        XCTAssertEqual(coverage.outsideViewPoints, 0)
        XCTAssertEqual(coverage.historyFrames, 1)
        XCTAssertEqual(coverage.oldestHistoryAge ?? 0, 0.5, accuracy: 1e-9)
        let rememberedPoints = coverage.points.filter { $0.support == .rememberedCovered }
        XCTAssertEqual(rememberedPoints.count, coverage.rememberedPoints)
        XCTAssertTrue(rememberedPoints.allSatisfy { ($0.rememberedAge ?? 0) == 0.5 && !$0.outsideEdges.isEmpty },
            "Remembered points keep their live out-of-view edges for diagnosis")
        let fields = coverage.diagnosticFields(includePoints: false)
        XCTAssertEqual(fields["depth_remembered_points"], .number(Double(coverage.rememberedPoints)))
        XCTAssertEqual(fields["depth_history_frames"], .number(1))
    }

    func testMemoryOlderThanTheMaximumAgeIsIgnored() throws {
        let stale = try rememberedFrame(back: 0.8, age: DepthVisibilityMemory.maximumAge + 0.01)
        let result = DepthSafetyEvaluator.evaluate(liveNearBlindSnapshot(history: [stale]),
            command: .init(left: 0.25, right: 0.25), now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(result.state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(result.coverage?.rememberedPoints, 0)
    }

    func testLiveViewStaysAuthoritativeWhereverItCanSee() throws {
        // Live depth is missing in the lower image half; those samples are in view.
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3)
        fill(depth: .nan, x: 0...79, y: 40...59, in: map)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let liveOnly = try XCTUnwrap(DepthSafetyEvaluator.evaluate(liveNearBlindSnapshot(depth: map),
            command: command, now: 10.05, boundedReadyMotion: true).coverage)
        XCTAssertGreaterThan(liveOnly.missingDepthPoints, 0)
        let withMemory = try XCTUnwrap(DepthSafetyEvaluator.evaluate(
            liveNearBlindSnapshot(history: [try rememberedFrame(back: 0.8, age: 0.5)], depth: map),
            command: command, now: 10.05, boundedReadyMotion: true).coverage)
        XCTAssertEqual(withMemory.missingDepthPoints, liveOnly.missingDepthPoints)
        for (before, after) in zip(liveOnly.points, withMemory.points) where before.support != .outsideView {
            XCTAssertEqual(after.support, before.support)
            XCTAssertNil(after.rememberedAge)
        }
    }

    func testRememberedOcclusionIsNotCoverageAndNewestViewpointDecides() throws {
        let command = WheelCommand(left: 0.25, right: 0.25)
        // Newest retained frame saw a surface in front of the samples; an older
        // frame saw past them. The newer observation wins.
        let blocked = try rememberedFrame(back: 0.8, age: 0.3, depth: 0.4)
        let olderClear = try rememberedFrame(back: 0.8, age: 1.0)
        let result = DepthSafetyEvaluator.evaluate(liveNearBlindSnapshot(history: [blocked, olderClear]),
            command: command, now: 10.05, boundedReadyMotion: true)
        let coverage = try XCTUnwrap(result.coverage)
        XCTAssertEqual(result.state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(coverage.rememberedPoints, 0)
        XCTAssertGreaterThan(coverage.occludedPoints, 0)
        XCTAssertTrue(coverage.points.filter { $0.support == .occluded }.allSatisfy {
            abs(($0.rememberedAge ?? 0) - 0.3) < 1e-9
        })
    }

    func testRememberedObstacleTheLiveCameraCannotSeeStillStops() throws {
        // A low box just past the bumper, seen from 0.8 m back, now below the live view.
        let box = Set((-1...1).map { DepthSafetyCell(lateralIndex: $0, heightIndex: 1, forwardIndex: 25) })
        let frame = try rememberedFrame(back: 0.8, age: 0.5, cells: box)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let result = DepthSafetyEvaluator.evaluate(liveNearBlindSnapshot(history: [frame]),
            command: command, now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(result.state, .stop)
        XCTAssertEqual(result.rememberedHazardCellCount, 3)
        XCTAssertLessThanOrEqual(result.clearance, 0.30)
    }

    func testRememberedObstacleIsDroppedWhereLiveDepthNowSeesPastIt() throws {
        // Same box higher up, where the live camera sees through to the 3 m wall.
        let moved = Set((-1...1).map { DepthSafetyCell(lateralIndex: $0, heightIndex: 6, forwardIndex: 25) })
        let frame = try rememberedFrame(back: 0.8, age: 0.5, cells: moved)
        let result = DepthSafetyEvaluator.evaluate(liveNearBlindSnapshot(history: [frame]),
            command: .init(left: 0.25, right: 0.25), now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(result.rememberedHazardCellCount, 0)
        XCTAssertNotEqual(result.state, .stop)
    }

    // MARK: - Follow depth policy (docs/follow-depth-policy.md)

    func testAstralCompatibleDropsOnlyTheVisibilityRequirementAndStillReportsIt() throws {
        let snapshot = liveNearBlindSnapshot()
        let cases: [(String, WheelCommand, Bool)] = [("ready", .init(left: 0.25, right: 0.25), true),
                                                     ("pursuit", .init(left: 0.25, right: 0.25), false),
                                                     ("curve", .init(left: 0.20, right: 0.25), false),
                                                     ("turn", .init(left: -0.25, right: 0.25), false)]
        var blindUnderFullVolume: [String] = []
        for (name, command, bounded) in cases {
            let full = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05, boundedReadyMotion: bounded)
            let astral = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05,
                boundedReadyMotion: bounded, visibilityPolicy: .astralCompatible)
            XCTAssertEqual(full.visibilityPolicy, .fullVolume)
            XCTAssertEqual(astral.visibilityPolicy, .astralCompatible)
            XCTAssertEqual(astral.coverageProvenance, "astral_compatible", name)
            // Same measurement under both policies; only enforcement differs.
            XCTAssertEqual(astral.fullVolumeWouldBlock, full.fullVolumeWouldBlock, name)
            XCTAssertEqual(astral.coverage?.coveredPoints, full.coverage?.coveredPoints, name)
            XCTAssertEqual(astral.requiredStoppingDistance, full.requiredStoppingDistance, name)
            XCTAssertEqual(astral.state, .clear, name)
            if full.state == .unavailable(.blindSweptVolume) {
                blindUnderFullVolume.append(name)
                XCTAssertEqual(full.fullVolumeWouldBlock, true, name)
            } else {
                XCTAssertEqual(full.state, .clear, name)
                XCTAssertEqual(full.fullVolumeWouldBlock, false, name)
            }
        }
        // The 30 cm ready-signal horizon is the near-field case the mount cannot see;
        // pursuit's longer horizon dilutes the blind near samples in this synthetic scene.
        XCTAssertTrue(blindUnderFullVolume.contains("ready"), "\(blindUnderFullVolume)")
        XCTAssertTrue(blindUnderFullVolume.contains("turn"), "\(blindUnderFullVolume)")
    }

    func testAstralCompatibleKeepsEveryOtherDepthVeto() throws {
        let command = WheelCommand(left: 0.25, right: 0.25)
        func astral(_ snapshot: DepthSafetySnapshot, _ command: WheelCommand = .init(left: 0.25, right: 0.25),
                    now: TimeInterval = 10.05) -> DepthSafetyObservation {
            DepthSafetyEvaluator.evaluate(snapshot, command: command, now: now, visibilityPolicy: .astralCompatible)
        }
        let live = liveNearBlindSnapshot()
        func withCells(_ cells: Set<DepthSafetyCell>) -> DepthSafetySnapshot {
            .init(timestamp: 10, cells: cells, cellResolution: 0.04, validSampleRatio: 1,
                  geometry: geometry, projection: live.projection, failureReason: nil)
        }
        XCTAssertEqual(astral(live, now: 10.30).state, .unavailable(.staleRawDepth))
        for reason: DepthSafetyUnavailableReason in [.missingRawDepth, .malformedDepth, .invalidCalibration] {
            let failed = DepthSafetySnapshot(timestamp: 10, cells: [], cellResolution: 0.04, validSampleRatio: 0,
                geometry: geometry, projection: nil, failureReason: reason)
            XCTAssertEqual(astral(failed).state, .unavailable(reason))
        }
        XCTAssertEqual(astral(live, .init(left: -0.25, right: -0.25)).state, .unavailable(.blindSweptVolume),
            "Reverse has no rear depth under either policy")

        let near = Set((1...3).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 10) })
        let stop = astral(withCells(near))
        XCTAssertEqual(stop.state, .stop)
        XCTAssertLessThanOrEqual(stop.clearance, 0.30)

        let ahead = Set((1...3).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 14) })
        let caution = astral(withCells(ahead))
        XCTAssertEqual(caution.state, .caution)
        XCTAssertEqual(caution.speedLimit, 0.12)

        let beside = Set((1...3).map { DepthSafetyCell(lateralIndex: 6, heightIndex: $0, forwardIndex: 0) })
        XCTAssertEqual(astral(withCells(beside), .init(left: -0.25, right: 0.25)).state, .stop,
            "A visible obstacle inside the turning radius still stops a turn")

        let box = Set((-1...1).map { DepthSafetyCell(lateralIndex: $0, heightIndex: 1, forwardIndex: 25) })
        let remembered = astral(liveNearBlindSnapshot(history: [try rememberedFrame(back: 0.8, age: 0.5, cells: box)]), command)
        XCTAssertEqual(remembered.state, .stop, "A remembered obstacle the camera cannot see still stops")
        XCTAssertEqual(remembered.rememberedHazardCellCount, 3)
    }

    @MainActor
    func testDepthPolicyReachesOnlyFollowOwnedDepthReads() {
        let ar = ARSessionManager()
        ar.ingestDepthSafety(rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3),
            intrinsics: ordinaryIntrinsics, cameraTransform: cameraTransform(height: 0.30), timestamp: 10)
        let command = WheelCommand(left: 0.25, right: 0.25)
        func ready() -> DepthSafetyObservation {
            ReadyMotionDepthScope.$active.withValue(true) { ar.depthSafetyObservation(for: command, at: 10.05) }
        }
        // Ordinary navigation and Silent Search call the provider outside any policy scope.
        XCTAssertEqual(ready().state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(ready().visibilityPolicy, .fullVolume)
        let scoped = FollowDepthPolicyScope.$current.withValue(.astralCompatible) { ready() }
        XCTAssertEqual(scoped.state, .clear)
        XCTAssertEqual(scoped.visibilityPolicy, .astralCompatible)
        XCTAssertEqual(FollowDepthPolicyScope.current, .fullVolume, "The scope never leaks past its read")
        XCTAssertEqual(RoverConfig.followDepthPolicy, .astralCompatible,
            "Default for the current WAVE ROVER mount; change deliberately with docs/follow-depth-policy.md")
    }

    @MainActor
    func testFollowControllerAppliesItsPolicyToEveryDepthReadAndLogsIt() async throws {
        let snapshot = liveNearBlindSnapshot()
        for policy: FollowDepthPolicy in [.astralCompatible, .fullVolume] {
            let sink = FollowDiagnosticRecordingSink()
            var scopes: [FollowDepthPolicy] = []
            let pose = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
                frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 10, trackingQuality: .normal)
            let controller = FollowNavigationController(currentPose: { pose.pose }, forwardClearance: { 2 },
                depthSafetyObservation: { command in
                    // Mirrors ARSessionManager's provider, which reads the scope the same way.
                    scopes.append(FollowDepthPolicyScope.current)
                    return DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05,
                        boundedReadyMotion: true, visibilityPolicy: FollowDepthPolicyScope.current)
                },
                plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in }, stopRover: {}, sleep: { _ in },
                diagnosticEmitter: .init(streamID: "depth-policy", monotonic: { 10.05 }, utc: { Date() }, sink: sink.append),
                poseSample: { pose }, sourceNow: { 10.05 }, sourceStopSnapshot: { pose }, depthPolicy: policy)
            XCTAssertEqual(controller.depthPolicy, policy)
            _ = await controller.sendFollowTurnBurst(.init(left: 0.25, right: 0.25), requestedBudget: 0.02, purpose: .followReady)
            XCTAssertFalse(scopes.isEmpty)
            XCTAssertTrue(scopes.allSatisfy { $0 == policy }, "\(scopes)")
            let payload = try XCTUnwrap(sink.records.first { $0.event == "follow_depth.candidate" }?.fields["payload"])
            let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any])
            XCTAssertEqual(json["depth_visibility_policy"] as? String, policy.rawValue)
            XCTAssertEqual(json["depth_full_volume_would_block"] as? Bool, true)
            XCTAssertEqual(json["depth_state"] as? String, policy == .fullVolume ? "blind_swept_volume" : "clear")
        }
    }

    private func offsetSectionGeometry(lateralOffset: Double = 0.03) -> RoverCollisionGeometry {
        .init(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04, maximumCollisionHeight: 0.34, sections: [
            .init(name: "chassis", length: 0.30, width: 0.25, minimumHeight: 0.04, maximumHeight: 0.14),
            .init(name: "upper", length: 0.08, width: 0.10, minimumHeight: 0.14, maximumHeight: 0.34,
                forwardOffset: -0.05, lateralOffset: lateralOffset)])
    }

    private var wideIntrinsics: simd_float3x3 {
        simd_float3x3(columns: (
            SIMD3<Float>(20, 0, 0),
            SIMD3<Float>(0, 8, 0),
            SIMD3<Float>(40, 30, 1)
        ))
    }

    private func cameraTransform(height: Float) -> simd_float4x4 {
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(0, height, 0, 1)
        return transform
    }

    private func makeDepthBuffer(width: Int, height: Int, constantDepth: Float) -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_DepthFloat32,
            nil,
            &pixelBuffer
        )
        let buffer = pixelBuffer!
        fill(depth: constantDepth, x: 0...(width - 1), y: 0...(height - 1), in: buffer)
        return buffer
    }

    private func fill(depth: Float,
                      x: ClosedRange<Int>,
                      y: ClosedRange<Int>,
                      in buffer: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let stride = CVPixelBufferGetBytesPerRow(buffer) / MemoryLayout<Float32>.size
        let base = CVPixelBufferGetBaseAddress(buffer)!.assumingMemoryBound(to: Float32.self)
        for row in y {
            for column in x {
                base[row * stride + column] = depth
            }
        }
    }

    private func fillFloorPlane(cameraHeight: Float, in buffer: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let width = CVPixelBufferGetWidth(buffer)
        let height = CVPixelBufferGetHeight(buffer)
        let stride = CVPixelBufferGetBytesPerRow(buffer) / MemoryLayout<Float32>.size
        let base = CVPixelBufferGetBaseAddress(buffer)!.assumingMemoryBound(to: Float32.self)
        let fy = wideIntrinsics[1][1]
        let cy = wideIntrinsics[2][1]
        for row in Int(cy + 1)..<height {
            let depth = cameraHeight * fy / (Float(row) - cy)
            for column in 0..<width {
                base[row * stride + column] = depth
            }
        }
    }
}

@MainActor
final class FollowSearchClearanceTests: XCTestCase {
    private func sample(x: Double = 0, generation: UInt64 = 7, now: Double = 10,
                        tracking: ARTrackingQuality = .normal) -> NavigationPoseSample {
        .init(pose: .init(position: .init(x, 0), yaw: 0), frameID: .init(generation: generation, sequence: 1),
              sourceTimestamp: now, trackingQuality: tracking)
    }

    func testHealthyRotationRetainsGrantThroughTwentyCentimeterObservedShift() throws {
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        XCTAssertTrue(grant.validate(sample: sample(x: 0.12), session: 3, now: 10))
        XCTAssertTrue(grant.validate(sample: sample(x: 0.20), session: 3, now: 10))
    }

    func testConfirmationIsSingleSessionAndNeverRenewsAbsoluteExpiry() throws {
        let confirmation = FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10)
        let grant = try XCTUnwrap(confirmation.consume(session: 3))
        XCTAssertNil(confirmation.consume(session: 4))
        XCTAssertTrue(grant.validate(sample: sample(now: 69.99), session: 3, now: 69.99))
        XCTAssertFalse(grant.validate(sample: sample(now: 70), session: 3, now: 70))
        XCTAssertTrue(grant.revoked)
    }

    func testSourceResetDriftAndTrackingLossPermanentlyRevoke() throws {
        for invalid in [sample(x: 0.201), sample(generation: 8), sample(tracking: .limited), sample(now: 9), NavigationPoseSample.unavailable] {
            let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
            XCTAssertFalse(grant.validate(sample: invalid, session: 3, now: 10))
            XCTAssertFalse(grant.validate(sample: sample(), session: 3, now: 10), "Returning to origin cannot restore assertion")
        }
    }

    func testExcursionReasonAndFrameLatchAcrossReturnAndCleanup() throws {
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        XCTAssertFalse(grant.validate(sample: sample(x: 0.20001, now: 11), session: 3, now: 11))
        XCTAssertFalse(grant.validate(sample: sample(now: 12), session: 3, now: 12))
        grant.revoke(now: 13)
        XCTAssertEqual(grant.revocationCause, .searchClearanceSpaceExceeded)
        XCTAssertEqual(grant.diagnosticFields["search_clearance_observed_displacement_m"], .number(0.20001))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_observed_frame"], .string("7:1"))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_source_timestamp_s"], .number(11))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_revoked_at_s"], .number(11))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_displacement_limit_m"], .number(0.20))
    }

    func testRadialBudgetDoesNotAccumulatePathOrRebaseOrigin() throws {
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        for x in [0.19, -0.19, 0.19, -0.19] {
            XCTAssertTrue(grant.validate(sample: sample(x: x), session: 3, now: 10))
        }
        XCTAssertFalse(grant.validate(sample: sample(x: -0.20001), session: 3, now: 10))
    }

    func testFutureNonfiniteAndDifferentSessionAssertionsFailClosed() throws {
        for timestamp in [11.0, .infinity, .nan] {
            let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: timestamp).consume(session: 3))
            XCTAssertFalse(grant.validate(sample: sample(), session: 3, now: 10))
        }
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        XCTAssertFalse(grant.validate(sample: sample(), session: 4, now: 10))
    }
}
