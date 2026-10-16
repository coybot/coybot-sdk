import CoreVideo
import RoverNav
import simd
import XCTest
@testable import PhroverKit

@MainActor
final class ARSharedMissionFrameCalibratorTests: XCTestCase {
    func testIgnoresSnapshotsUntilTrackingIsNormal() async throws {
        let manager = ARSessionManager()
        let scans = ScanCounter()
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { _ in
            scans.increment()
            return []
        }
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await _ in stream {}
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 100, height: 100),
            depthMap: nil, trackingQuality: .limited
        )
        manager.ingestForTesting(
            image: makeImage(), timestamp: 2, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 100, height: 100),
            depthMap: nil, trackingQuality: .unavailable
        )
        await taskTurn()
        XCTAssertEqual(scans.value, 0)

        manager.ingestForTesting(
            image: makeImage(), timestamp: 3, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 100, height: 100),
            depthMap: nil, trackingQuality: .normal
        )
        await eventually { scans.value == 1 }
        consumer.cancel()
    }

    func testNonNormalTrackingFeedbackIsDeduplicatedUntilTheIssueChanges() async {
        enum ScannerFailure: Error { case failed }
        let manager = ARSessionManager()
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager, scanner: { _ in
            throw ScannerFailure.failed
        })
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let received = Task { () -> [SilentSearchCalibrationEvent] in
            var events: [SilentSearchCalibrationEvent] = []
            for await event in stream {
                events.append(event)
                if events.count == 2 { break }
            }
            return events
        }
        await Task.yield()

        for (timestamp, quality) in [(1.0, ARTrackingQuality.limited), (2.0, .unavailable), (3.0, .normal)] {
            manager.ingestForTesting(
                image: makeImage(), timestamp: timestamp, cameraTransform: matrix_identity_float4x4,
                intrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
                trackingQuality: quality
            )
            await Task.yield()
        }

        let events = await received.value
        XCTAssertEqual(events, [
            .feedback(.trackingNotNormal(
                context: .init(frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1)
            )),
            .feedback(.scannerFailed(
                context: .init(frameID: ARFrameID(generation: 0, sequence: 3), monotonicTimestamp: 3)
            )),
        ])
    }

    func testGroundsOrientedCornersWithTheObservationSnapshot() throws {
        let snapshot = makeSnapshot(generation: 7, sequence: 3)
        let observation = OpticalObservation(
            payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8), frameID: 3,
            monotonicTimestamp: 4.5,
            corners: OrientedMarkerCorners(topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
                                           bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4))
        )

        let result = ARSharedMissionFrameCalibrator.ground(
            observation: observation, in: snapshot, expectedMarkerID: "SILENT_SEARCH_01",
            sessionGeneration: 7
        )
        guard case let .success(grounded) = result else {
            return XCTFail("Expected grounded observation, got \(result)")
        }

        XCTAssertEqual(grounded.sessionGeneration, 7)
        XCTAssertEqual(grounded.frameID, 3)
        XCTAssertEqual(grounded.monotonicTimestamp, 4.5)
        XCTAssertGreaterThan(grounded.corners.topLeft.y, grounded.corners.bottomLeft.y)
        XCTAssertEqual(grounded.corners.topLeft.x, -0.1, accuracy: 0.001)
        XCTAssertEqual(grounded.corners.topRight.x, 0.1, accuracy: 0.001)
    }

    func testGroundingFailuresHaveDeterministicValidationPrecedence() {
        let snapshot = makeSnapshot(generation: 7, sequence: 3, missingCorner: .topLeft)
        let corners = OrientedMarkerCorners(topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
                                             bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4))
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(observation: OpticalObservation(
            payload: Data([0xFF]), frameID: 2,
            monotonicTimestamp: 1, corners: corners
        ), in: snapshot, expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7), .failure(.frameMismatch))
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(observation: OpticalObservation(
            payload: Data([0xFF]), frameID: 3,
            monotonicTimestamp: 1, corners: corners
        ), in: snapshot, expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7), .failure(.timestampMismatch))
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(observation: OpticalObservation(
            payload: Data([0xFF]), frameID: 3,
            monotonicTimestamp: 4.5, corners: corners
        ), in: snapshot, expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7), .failure(.invalidPayload))
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(observation: OpticalObservation(
            payload: Data("PHROVER-CAL|1|OTHER".utf8), frameID: 3,
            monotonicTimestamp: 4.5, corners: corners
        ), in: snapshot, expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7), .failure(.wrongMarkerID))
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(observation: OpticalObservation(
            payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8), frameID: 3,
            monotonicTimestamp: 4.5, corners: corners
        ), in: snapshot, expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7),
                       .failure(.cornerUnavailable(.topLeft)))

        let validObservation = OpticalObservation(
            payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8), frameID: 3,
            monotonicTimestamp: 4.5, corners: corners
        )
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(
            observation: validObservation,
            in: makeSnapshot(generation: 7, sequence: 3, trackingQuality: .limited),
            expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7
        ), .failure(.trackingNotNormal))
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(
            observation: validObservation, in: makeSnapshot(generation: 7, sequence: 3),
            expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 8
        ), .failure(.generationMismatch))
        XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(
            observation: validObservation,
            in: makeSnapshot(generation: 7, sequence: 3, includesDepth: false),
            expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7
        ), .failure(.missingDepthMap))
        for corner in [SilentSearchCalibrationCorner.topRight, .bottomLeft, .bottomRight] {
            XCTAssertEqual(ARSharedMissionFrameCalibrator.ground(
                observation: validObservation,
                in: makeSnapshot(generation: 7, sequence: 3, missingCorner: corner),
                expectedMarkerID: "SILENT_SEARCH_01", sessionGeneration: 7
            ), .failure(.cornerUnavailable(corner)))
        }
    }

    func testExpectedMarkerFeedbackPrecedesGroundingFailureForTheSameFrame() async {
        let manager = ARSessionManager()
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { frame in
            [OpticalObservation(
                payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                corners: corners
            )]
        }
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let received = Task { () -> [SilentSearchCalibrationEvent] in
            var events: [SilentSearchCalibrationEvent] = []
            for await event in stream {
                events.append(event)
                if events.count == 2 { break }
            }
            return events
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 2, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 100, height: 100),
            depthMap: nil, trackingQuality: .normal
        )

        let frameID = ARFrameID(generation: 0, sequence: 1)
        let events = await received.value
        XCTAssertEqual(events, [
            .feedback(.expectedMarkerDetected(
                context: .init(frameID: frameID, monotonicTimestamp: 2),
                markerID: "SILENT_SEARCH_01", corners: corners
            )),
            .feedback(.groundingFailed(
                context: .init(frameID: frameID, monotonicTimestamp: 2), reason: .missingDepthMap
            )),
        ])
    }

    func testExpectedMarkerDetectionIsObservedBeforeGroundingStarts() async {
        let manager = ARSessionManager()
        let collector = CalibrationEventCollector()
        let probe = GroundingOrderProbe()
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let context = SilentSearchCalibrationFrameContext(
            frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 2
        )
        let detection = SilentSearchCalibrationEvent.feedback(.expectedMarkerDetected(
            context: context, markerID: "SILENT_SEARCH_01", corners: corners
        ))
        let calibrator = ARSharedMissionFrameCalibrator(
            sessionManager: manager,
            grounder: { _, _, _, _ in
                probe.record(eventsObserved: collector.events, expectedDetection: detection)
                return .failure(.missingDepthMap)
            }
        ) { frame in
            [OpticalObservation(
                payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                corners: corners
            )]
        }
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 2, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 100, height: 100),
            depthMap: nil, trackingQuality: .normal
        )
        await eventually { probe.wasInvoked }
        await eventually { collector.events.count == 2 }

        XCTAssertTrue(probe.detectionWasObserved)
        XCTAssertEqual(Array(collector.events.prefix(2)), [
            detection,
            .feedback(.groundingFailed(context: context, reason: .missingDepthMap)),
        ])
        consumer.cancel()
    }

    func testFallbackSuccessEmitsBackendDiagnosticBeforeQRDetectedFeedback() async {
        let manager = ARSessionManager()
        let collector = CalibrationEventCollector()
        let diagnostic = OpticalScannerBackendDiagnostic(
            backend: .vision, orientation: .right,
            errorDomain: "VisionStableDomain", errorCode: 17
        )
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(
            sessionManager: manager,
            grounder: { _, _, _, _ in .failure(.missingDepthMap) },
            detailedScanner: { frame in
                OpticalScanOutcome(
                    observations: [OpticalObservation(
                        payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                        frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                        corners: corners
                    )],
                    diagnostics: [diagnostic]
                )
            }
        )
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 2, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal
        )
        await eventually { collector.events.count == 3 }

        let context = SilentSearchCalibrationFrameContext(
            frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 2
        )
        XCTAssertEqual(collector.events, [
            .feedback(.scannerBackendFailed(context: context, diagnostic: diagnostic)),
            .feedback(.expectedMarkerDetected(
                context: context, markerID: "SILENT_SEARCH_01", corners: corners
            )),
            .feedback(.groundingFailed(context: context, reason: .missingDepthMap)),
        ])
        XCTAssertFalse(collector.events.contains(.feedback(.scannerFailed(context: context))))
        consumer.cancel()
    }

    func testAllCornersGroundedFeedbackPrecedesCalibrationProgress() async {
        let manager = ARSessionManager()
        let snapshot = makeSnapshot(generation: 0, sequence: 1)
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { frame in
            [OpticalObservation(
                payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                corners: corners
            )]
        }
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let received = Task { () -> [SilentSearchCalibrationEvent] in
            var events: [SilentSearchCalibrationEvent] = []
            for await event in stream {
                events.append(event)
                if events.count == 3 { break }
            }
            return events
        }
        await Task.yield()

        manager.ingestForTesting(
            image: snapshot.image, timestamp: 4.5, cameraTransform: snapshot.cameraTransform,
            intrinsics: snapshot.cameraIntrinsics, imageResolution: snapshot.imageResolution,
            depthMap: snapshot.depthMap, trackingQuality: .normal
        )

        let events = await received.value
        XCTAssertEqual(events[0], .feedback(.expectedMarkerDetected(
            context: .init(
                frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 4.5
            ),
            markerID: "SILENT_SEARCH_01", corners: corners
        )))
        XCTAssertEqual(events[1], .feedback(.allCornersGrounded(
            context: .init(
                frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 4.5
            )
        )))
        XCTAssertEqual(events[2], .progress(
            context: .init(
                frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 4.5
            ),
            acceptedFrameCount: 1
        ))
    }

    func testThirdAcceptedSampleEmitsContextualProgressBeforeAcceptance() async {
        let manager = ARSessionManager()
        let snapshot = makeSnapshot(generation: 0, sequence: 1)
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { frame in
            [OpticalObservation(
                payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                corners: corners
            )]
        }
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        var iterator = stream.makeAsyncIterator()
        // Let the calibrator subscribe to snapshots before the first frame arrives.
        for _ in 0..<10 { await Task.yield() }

        // `snapshots()` buffers only the newest frame, so drain each frame's events before
        // ingesting the next; otherwise a slow consumer drops a sample and this never ends.
        var events: [SilentSearchCalibrationEvent] = []
        for (index, timestamp) in [1.0, 1.1, 1.2].enumerated() {
            manager.ingestForTesting(
                image: snapshot.image, timestamp: timestamp,
                cameraTransform: snapshot.cameraTransform,
                intrinsics: snapshot.cameraIntrinsics, imageResolution: snapshot.imageResolution,
                depthMap: snapshot.depthMap, trackingQuality: .normal
            )
            while let event = await iterator.next() {
                events.append(event)
                if case .progress(_, let count) = event, count == index + 1 { break }
            }
        }
        while let event = await iterator.next() {
            events.append(event)
            if case .accepted = event { break }
        }
        let context = SilentSearchCalibrationFrameContext(
            frameID: ARFrameID(generation: 0, sequence: 3), monotonicTimestamp: 1.2
        )
        guard case let .accepted(frame) = events.last else {
            return XCTFail("Expected acceptance after third-sample progress, got \(events)")
        }
        XCTAssertEqual(Array(events.suffix(4)), [
            .feedback(.expectedMarkerDetected(
                context: context, markerID: "SILENT_SEARCH_01", corners: corners
            )),
            .feedback(.allCornersGrounded(context: context)),
            .progress(context: context, acceptedFrameCount: 3),
            .accepted(frame),
        ])
    }

    func testScannerFailuresAreExplicitAndConsecutiveFailuresAreDeduplicated() async {
        enum ScannerFailure: Error { case failed }
        let manager = ARSessionManager()
        let scans = ScanCounter()
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager, scanner: { _ in
            scans.increment()
            throw ScannerFailure.failed
        })
        let collector = CalibrationEventCollector()
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal
        )
        await eventually { collector.events.count == 1 }
        manager.ingestForTesting(image: makeImage(), timestamp: 2,
            cameraTransform: matrix_identity_float4x4, intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal)
        await eventually { scans.value == 2 }
        manager.ingestForTesting(image: makeImage(), timestamp: 3,
            cameraTransform: matrix_identity_float4x4, intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .limited)

        await eventually { collector.events.count == 2 }
        XCTAssertEqual(collector.events, [
            .feedback(.scannerFailed(context: .init(
                frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1
            ))),
            .feedback(.trackingNotNormal(context: .init(
                frameID: ARFrameID(generation: 0, sequence: 3), monotonicTimestamp: 3
            ))),
        ])
        consumer.cancel()
    }

    func testSuccessfulEmptyScanClearsScannerFailureOnce() async {
        enum ScannerFailure: Error { case failed }
        let manager = ARSessionManager()
        let scans = ScanCounter()
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { _ in
            scans.increment()
            if scans.value == 1 { throw ScannerFailure.failed }
            return []
        }
        let collector = CalibrationEventCollector()
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        for timestamp in [1.0, 2.0, 3.0] {
            manager.ingestForTesting(
                image: makeImage(), timestamp: timestamp,
                cameraTransform: matrix_identity_float4x4,
                intrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
                trackingQuality: .normal
            )
            await Task.yield()
        }

        await eventually { scans.value == 3 }
        await eventually { collector.events.count == 2 }
        XCTAssertEqual(collector.events, [
            .feedback(.scannerFailed(context: .init(
                frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1
            ))),
            .feedback(.waitingForMarker(context: .init(
                frameID: ARFrameID(generation: 0, sequence: 2), monotonicTimestamp: 2
            ))),
        ])
        consumer.cancel()
    }

    func testEmptyDetailedFallbackRestoresWaitingGuidanceAfterScannerFailure() async {
        enum ScannerFailure: Error { case failed }
        let manager = ARSessionManager()
        let scans = ScanCounter()
        let diagnostic = OpticalScannerBackendDiagnostic(
            backend: .vision, orientation: .right,
            errorDomain: "VisionStableDomain", errorCode: 17
        )
        let calibrator = ARSharedMissionFrameCalibrator(
            sessionManager: manager,
            detailedScanner: { _ in
                scans.increment()
                if scans.value == 1 { throw ScannerFailure.failed }
                return OpticalScanOutcome(observations: [], diagnostics: [diagnostic])
            }
        )
        let collector = CalibrationEventCollector()
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        for timestamp in [1.0, 2.0] {
            manager.ingestForTesting(
                image: makeImage(), timestamp: timestamp,
                cameraTransform: matrix_identity_float4x4,
                intrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
                trackingQuality: .normal
            )
            await Task.yield()
        }

        await eventually { collector.events.count == 3 }
        let first = SilentSearchCalibrationFrameContext(
            frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1
        )
        let second = SilentSearchCalibrationFrameContext(
            frameID: ARFrameID(generation: 0, sequence: 2), monotonicTimestamp: 2
        )
        XCTAssertEqual(collector.events, [
            .feedback(.scannerFailed(context: first)),
            .feedback(.scannerBackendFailed(context: second, diagnostic: diagnostic)),
            .feedback(.waitingForMarker(context: second)),
        ])
        consumer.cancel()
    }

    func testSuccessfulEmptyScanClearsWrongMarkerFailureOnce() async {
        let manager = ARSessionManager()
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { frame in
            guard frame.monotonicTimestamp == 1 else { return [] }
            return [OpticalObservation(
                payload: Data("PHROVER-CAL|1|OTHER".utf8), frameID: frame.frameID,
                monotonicTimestamp: frame.monotonicTimestamp, corners: corners
            )]
        }
        let collector = CalibrationEventCollector()
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        for timestamp in [1.0, 2.0, 3.0] {
            manager.ingestForTesting(
                image: makeImage(), timestamp: timestamp,
                cameraTransform: matrix_identity_float4x4,
                intrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
                trackingQuality: .normal
            )
            await Task.yield()
        }

        await eventually { collector.events.count == 2 }
        XCTAssertEqual(collector.events, [
            .feedback(.groundingFailed(context: .init(
                frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1
            ), reason: .wrongMarkerID)),
            .feedback(.waitingForMarker(context: .init(
                frameID: ARFrameID(generation: 0, sequence: 2), monotonicTimestamp: 2
            ))),
        ])
        consumer.cancel()
    }

    func testQRLossEmitsAfterHalfASecondWithoutAnotherSnapshotAndDetectionCanResume() async {
        let manager = ARSessionManager()
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { frame in
            guard frame.monotonicTimestamp == 1 || frame.monotonicTimestamp == 2 else { return [] }
            return [OpticalObservation(
                payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                corners: corners
            )]
        }
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let received = Task { () -> [SilentSearchCalibrationEvent] in
            var events: [SilentSearchCalibrationEvent] = []
            for await event in stream {
                events.append(event)
                if events.count == 4 { break }
            }
            return events
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal
        )
        try? await Task.sleep(for: .milliseconds(700))
        manager.ingestForTesting(
            image: makeImage(), timestamp: 2, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal
        )

        let events = await received.value
        XCTAssertEqual(events, [
            .feedback(.expectedMarkerDetected(
                context: .init(
                    frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1
                ),
                markerID: "SILENT_SEARCH_01", corners: corners
            )),
            .feedback(.groundingFailed(
                context: .init(
                    frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1
                ),
                reason: .missingDepthMap
            )),
            .feedback(.qrLost(
                context: .init(
                    frameID: ARFrameID(generation: 0, sequence: 1), monotonicTimestamp: 1
                )
            )),
            .feedback(.expectedMarkerDetected(
                context: .init(
                    frameID: ARFrameID(generation: 0, sequence: 2), monotonicTimestamp: 2
                ),
                markerID: "SILENT_SEARCH_01", corners: corners
            )),
        ])
    }

    func testQRLossAllowsSameGroundingFailureAfterSnapshotsResume() async {
        let manager = ARSessionManager()
        let expiry = MarkerExpiryController()
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(
            sessionManager: manager,
            markerExpirySleep: { await expiry.wait() }
        ) { frame in
            [OpticalObservation(
                payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                corners: corners
            )]
        }
        let collector = CalibrationEventCollector()
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal
        )
        await eventually { collector.events.count == 2 && expiry.isWaiting }
        expiry.expire()
        await eventually { collector.events.count == 3 }

        manager.ingestForTesting(
            image: makeImage(), timestamp: 2, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal
        )
        await eventually { collector.events.count == 5 }

        XCTAssertEqual(collector.events.map { event in
            switch event {
            case .feedback(.expectedMarkerDetected): "detected"
            case .feedback(.groundingFailed(_, .missingDepthMap)): "missing_depth"
            case .feedback(.qrLost): "lost"
            default: "unexpected"
            }
        }, ["detected", "missing_depth", "lost", "detected", "missing_depth"])
        await eventually { expiry.isWaiting }
        calibrator.cancel()
        expiry.expire()
        consumer.cancel()
    }

    func testCancellingCalibrationCancelsPendingQRExpiry() async {
        let manager = ARSessionManager()
        let corners = OrientedMarkerCorners(
            topLeft: Vec2(0.6, 0.6), topRight: Vec2(0.6, 0.4),
            bottomLeft: Vec2(0.4, 0.6), bottomRight: Vec2(0.4, 0.4)
        )
        let calibrator = ARSharedMissionFrameCalibrator(sessionManager: manager) { frame in
            [OpticalObservation(
                payload: Data("PHROVER-CAL|1|SILENT_SEARCH_01".utf8),
                frameID: frame.frameID, monotonicTimestamp: frame.monotonicTimestamp,
                corners: corners
            )]
        }
        let collector = CalibrationEventCollector()
        let stream = calibrator.events(markerID: "SILENT_SEARCH_01", sessionGeneration: 0)
        let consumer = Task {
            for await event in stream { collector.append(event) }
        }
        await Task.yield()

        manager.ingestForTesting(
            image: makeImage(), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 100, height: 100), depthMap: nil,
            trackingQuality: .normal
        )
        await eventually { collector.events.count == 2 }
        calibrator.cancel()
        try? await Task.sleep(for: .milliseconds(600))

        XCTAssertEqual(collector.events.count, 2)
        consumer.cancel()
    }

    private func makeSnapshot(
        generation: UInt64,
        sequence: UInt64,
        missingCorner: SilentSearchCalibrationCorner? = nil,
        includesDepth: Bool = true,
        trackingQuality: ARTrackingQuality = .normal
    ) -> ARFrameSnapshot {
        let image = makeImage()
        var depth: CVPixelBuffer?
        if includesDepth {
            CVPixelBufferCreate(kCFAllocatorDefault, 10, 10, kCVPixelFormatType_DepthFloat32, nil, &depth)
            CVPixelBufferLockBaseAddress(depth!, [])
            let stride = CVPixelBufferGetBytesPerRow(depth!) / MemoryLayout<Float>.size
            let values = CVPixelBufferGetBaseAddress(depth!)!.assumingMemoryBound(to: Float.self)
            for y in 0..<10 {
                for x in 0..<10 { values[y * stride + x] = y < 5 ? 2 : 2.2 }
            }
            // Knock out a 3x3 patch, not one pixel: `sampleDepth` reads a small window
            // around each corner and bridges a lone invalid pixel by design.
            let hole: (y: Int, x: Int)? = switch missingCorner {
            case .topLeft: (4, 4)
            case .topRight: (4, 6)
            case .bottomLeft: (6, 4)
            case .bottomRight: (6, 6)
            case nil: nil
            }
            if let hole {
                for y in (hole.y - 1)...(hole.y + 1) {
                    for x in (hole.x - 1)...(hole.x + 1) { values[y * stride + x] = 0 }
                }
            }
            CVPixelBufferUnlockBaseAddress(depth!, [])
        }
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(200, 0, 0), SIMD3<Float>(0, 200, 0), SIMD3<Float>(50, 50, 1)
        ))
        return ARFrameSnapshot(id: ARFrameID(generation: generation, sequence: sequence), timestamp: 4.5,
            image: image, cameraTransform: matrix_identity_float4x4, cameraIntrinsics: intrinsics,
            imageResolution: CGSize(width: 100, height: 100), depthMap: depth,
            pose: Pose2D(position: Vec2(0, 0), yaw: 0), trackingQuality: trackingQuality)
    }

    private func makeImage() -> CVPixelBuffer {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 100, 100, kCVPixelFormatType_32BGRA, nil, &image)
        return image!
    }

    private func taskTurn() async {
        await Task.yield()
        await Task.yield()
    }

    private func eventually(_ condition: @escaping () -> Bool) async {
        // Bounded by wall time, not a yield count: under a loaded CI runner
        // 100 yields can run out before the work under test is scheduled.
        let deadline = ContinuousClock.now + .seconds(2)
        while !condition(), ContinuousClock.now < deadline { await Task.yield() }
    }
}

private final class ScanCounter: @unchecked Sendable {
    private let lock = NSLock()
    private var count = 0

    var value: Int { lock.withLock { count } }
    func increment() { lock.withLock { count += 1 } }
}

@MainActor
private final class CalibrationEventCollector {
    private(set) var events: [SilentSearchCalibrationEvent] = []

    func append(_ event: SilentSearchCalibrationEvent) {
        events.append(event)
    }
}

@MainActor
private final class GroundingOrderProbe {
    private(set) var wasInvoked = false
    private(set) var detectionWasObserved = false

    func record(
        eventsObserved: [SilentSearchCalibrationEvent],
        expectedDetection: SilentSearchCalibrationEvent
    ) {
        wasInvoked = true
        detectionWasObserved = eventsObserved.last == expectedDetection
    }
}

@MainActor
private final class MarkerExpiryController {
    private var continuation: CheckedContinuation<Void, Never>?
    var isWaiting: Bool { continuation != nil }

    func wait() async {
        await withCheckedContinuation { continuation = $0 }
    }

    func expire() {
        continuation?.resume()
        continuation = nil
    }
}
