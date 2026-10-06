import XCTest
import RoverNav
@testable import PhroverKit

final class SilentSearchTargetTrackerTests: XCTestCase {
    func testExactLabelConfidenceAndGroundingAreRequired() {
        let sink = RecordingTargetSink()
        let tracker = makeTracker(sink: sink)

        XCTAssertEqual(tracker.process(frame(1, 0, [detection("Chair", 1, point: p(0, 0))])), .collecting(sampleCount: 0))
        XCTAssertEqual(tracker.process(frame(2, 0.1, [detection("chair", 0.8999, point: p(0, 0))])), .collecting(sampleCount: 0))
        XCTAssertEqual(tracker.process(frame(3, 0.2, [detection("chair", 0.90, point: nil)])), .collecting(sampleCount: 0))
        XCTAssertEqual(sink.rejections, [.noMatchingDetection, .confidenceBelowThreshold, .invalidGrounding])
    }

    func testHighestConfidenceMatchingBoxDoesNotFallThroughWhenUngrounded() {
        let sink = RecordingTargetSink()
        let tracker = makeTracker(sink: sink)
        let detections = [
            detection("chair", 0.91, point: p(0, 0)),
            detection("chair", 0.99, point: nil),
        ]

        XCTAssertEqual(tracker.process(frame(1, 0, detections)), .collecting(sampleCount: 0))
        XCTAssertEqual(sink.rejections, [.invalidGrounding])
    }

    func testDuplicateFrameCountsOnce() {
        let sink = RecordingTargetSink()
        let tracker = makeTracker(sink: sink)

        XCTAssertEqual(tracker.process(frame(7, 0, [detection("chair", 0.9, point: p(0, 0))])), .collecting(sampleCount: 1))
        XCTAssertEqual(tracker.process(frame(7, 0.1, [detection("chair", 1, point: p(0, 0))])), .collecting(sampleCount: 1))
        XCTAssertEqual(sink.rejections, [.duplicateFrame])
    }

    func testGroundedLocalPointsAreTransformedAndMedianConfirmsAtExactWindowBoundary() {
        let tracker = makeTracker()
        XCTAssertEqual(tracker.process(frame(1, 0, [detection("chair", 0.9, point: p(0.8, 2))])), .collecting(sampleCount: 1))
        XCTAssertEqual(tracker.process(frame(2, 1, [detection("chair", 1, point: p(1, 2.2))])), .collecting(sampleCount: 2))

        guard case .confirmed(let confirmation) = tracker.process(
            frame(3, 2, [detection("chair", 0.95, point: p(1.2, 1.8))])
        ) else { return XCTFail("expected confirmation") }

        XCTAssertEqual(confirmation.label, "chair")
        XCTAssertEqual(confirmation.coordinate.x, 1, accuracy: 0.0001)
        XCTAssertEqual(confirmation.coordinate.y, 2, accuracy: 0.0001)
        XCTAssertEqual(confirmation.sampleCount, 3)
        XCTAssertEqual(confirmation.meanConfidence, 0.95, accuracy: 0.0001)
    }

    func testEvidenceOlderThanTwoSecondsIsTrimmed() {
        let tracker = makeTracker()
        _ = tracker.process(frame(1, 0, [detection("chair", 1, point: p(1, 2))]))
        _ = tracker.process(frame(2, 1, [detection("chair", 1, point: p(1, 2))]))

        XCTAssertEqual(
            tracker.process(frame(3, 2.0001, [detection("chair", 1, point: p(1, 2))])),
            .collecting(sampleCount: 2)
        )
    }

    func testClusterAcceptsExactThirtyFiveCentimeterBoundaryAndRejectsBeyondIt() {
        let accepted = makeTracker()
        _ = accepted.process(frame(1, 0, [detection("chair", 1, point: p(0, 0))]))
        _ = accepted.process(frame(2, 0.1, [detection("chair", 1, point: p(0.35, 0))]))
        XCTAssertNotNil(confirmation(accepted.process(frame(3, 0.2, [detection("chair", 1, point: p(-0.35, 0))]))))

        let rejected = makeTracker()
        _ = rejected.process(frame(1, 0, [detection("chair", 1, point: p(0, 0))]))
        _ = rejected.process(frame(2, 0.1, [detection("chair", 1, point: p(0.3501, 0))]))
        XCTAssertEqual(
            rejected.process(frame(3, 0.2, [detection("chair", 1, point: p(-0.3501, 0))])),
            .collecting(sampleCount: 3)
        )
    }

    func testOutlierExpiresBeforeLaterClusterConfirms() {
        let tracker = makeTracker()
        _ = tracker.process(frame(1, 0, [detection("chair", 1, point: p(4, 4))]))
        _ = tracker.process(frame(2, 0.7, [detection("chair", 1, point: p(0, 0))]))
        XCTAssertEqual(
            tracker.process(frame(3, 1, [detection("chair", 1, point: p(0.1, 0))])),
            .collecting(sampleCount: 3)
        )

        XCTAssertNotNil(confirmation(
            tracker.process(frame(4, 2.6, [detection("chair", 1, point: p(0, 0.1))]))
        ))
    }

    func testConfirmationLatchesWithoutChangingSummary() {
        let tracker = makeTracker()
        _ = tracker.process(frame(1, 0, [detection("chair", 0.9, point: p(0, 0))]))
        _ = tracker.process(frame(2, 0.1, [detection("chair", 0.9, point: p(0, 0))]))
        let first = tracker.process(frame(3, 0.2, [detection("chair", 0.9, point: p(0, 0))]))

        XCTAssertEqual(tracker.process(frame(4, 10, [detection("chair", 1, point: p(9, 9))])), first)
        XCTAssertEqual(tracker.confirmation, confirmation(first))
    }

    func testStructuredTelemetryContainsEvidenceButNoImageOrDetectionContents() {
        let sink = RecordingStructuredSink()
        let tracker = SilentSearchTargetTracker(
            canonicalLabel: "chair",
            frame: SharedMissionFrame(localOrigin: .zero, localNorthHeading: 0, sessionGeneration: 1)!,
            events: sink
        )

        _ = tracker.process(frame(1, 0, [detection("chair", 0.95, point: p(-2, -1))]))
        _ = tracker.process(frame(2, 0.1, [detection("chair", 0.96, point: p(-2, -1))]))
        _ = tracker.process(frame(3, 0.2, [detection("chair", 0.97, point: p(-2, -1))]))

        XCTAssertEqual(sink.entries.map(\.event), [
            "silent_search_target_evidence",
            "silent_search_target_evidence",
            "silent_search_target_evidence",
            "silent_search_target_confirmed",
        ])
        XCTAssertEqual(sink.entries.last?.fields["sample_count"], "3")
        XCTAssertEqual(sink.entries.last?.fields["confidence_basis_points"], "9600")
        XCTAssertTrue(sink.entries.allSatisfy { entry in
            entry.fields.keys.allSatisfy { !["payload", "image", "detections", "pixel_buffer"].contains($0) }
        })
    }

    private func makeTracker(sink: TargetTrackerEventSink? = nil) -> SilentSearchTargetTracker {
        SilentSearchTargetTracker(
            canonicalLabel: "chair",
            frame: SharedMissionFrame(localOrigin: Vec2(1, 2), localNorthHeading: 0, sessionGeneration: 1)!,
            eventSink: sink
        )
    }

    private func frame(_ id: UInt64, _ time: TimeInterval, _ detections: [TargetDetectionObservation]) -> TargetFrameObservation {
        TargetFrameObservation(frameID: id, monotonicTimestamp: time, detections: detections)
    }

    private func detection(_ label: String, _ confidence: Double, point: Vec2?) -> TargetDetectionObservation {
        TargetDetectionObservation(label: label, confidence: confidence, normalizedCenter: Vec2(0.5, 0.5), localGroundedPoint: point)
    }

    private func p(_ missionX: Double, _ missionY: Double) -> Vec2 {
        Vec2(1 + missionY, 2 - missionX)
    }

    private func confirmation(_ result: TargetTrackingResult) -> TargetConfirmation? {
        guard case .confirmed(let value) = result else { return nil }
        return value
    }
}

private final class RecordingStructuredSink: SilentSearchEventSink, @unchecked Sendable {
    struct Entry { let event: String; let fields: [String: String] }
    private(set) var entries: [Entry] = []

    func record(event: String, fields: [String: String]) {
        entries.append(Entry(event: event, fields: fields))
    }
}

private final class RecordingTargetSink: TargetTrackerEventSink {
    var rejections: [TargetRejectionReason] = []

    func record(_ event: TargetTrackerEvent) {
        if case .rejected(_, let reason) = event { rejections.append(reason) }
    }
}
