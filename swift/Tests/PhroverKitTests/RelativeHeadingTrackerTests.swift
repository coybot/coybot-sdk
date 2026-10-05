import XCTest
@testable import PhroverKit

final class RelativeHeadingTrackerTests: XCTestCase {
    func testIntegratesPositiveRotationAroundGravity() {
        var tracker = RelativeHeadingTracker()
        tracker.reset()

        XCTAssertTrue(tracker.ingest(sample(at: 0, rate: SIMD3(0, -1, 0))))
        XCTAssertTrue(tracker.ingest(sample(at: 0.1, rate: SIMD3(0, -1, 0))))

        let measurement = tracker.measurement(at: 0.1)
        XCTAssertEqual(measurement.accumulatedAngle, 0.1, accuracy: 0.000_001)
        XCTAssertEqual(measurement.reliability, .reliable)
    }

    func testIntegratesNegativeRotationAndIsIndependentOfMountAxis() {
        var upright = RelativeHeadingTracker()
        XCTAssertTrue(upright.ingest(sample(at: 0, rate: SIMD3(0, 2, 0))))
        XCTAssertTrue(upright.ingest(sample(at: 0.05, rate: SIMD3(0, 2, 0))))

        var landscape = RelativeHeadingTracker()
        let landscapeGravity = SIMD3<Double>(-1, 0, 0)
        XCTAssertTrue(landscape.ingest(RelativeHeadingSample(
            timestamp: 0,
            rotationRate: SIMD3(2, 0, 0),
            gravity: landscapeGravity
        )))
        XCTAssertTrue(landscape.ingest(RelativeHeadingSample(
            timestamp: 0.05,
            rotationRate: SIMD3(2, 0, 0),
            gravity: landscapeGravity
        )))

        XCTAssertEqual(upright.measurement(at: 0.05).accumulatedAngle, -0.1, accuracy: 0.000_001)
        XCTAssertEqual(landscape.measurement(at: 0.05).accumulatedAngle, -0.1, accuracy: 0.000_001)
    }

    func testTiltedAndNearVerticalMountsProduceSameRelativeTurn() {
        let inverseRootTwo = 1 / sqrt(2.0)
        let fixtures: [(gravity: SIMD3<Double>, rate: SIMD3<Double>)] = [
            (SIMD3(0, -1, 0), SIMD3(0, -1.5, 0)),
            (SIMD3(inverseRootTwo, -inverseRootTwo, 0),
             SIMD3(1.5 * inverseRootTwo, -1.5 * inverseRootTwo, 0)),
            (SIMD3(0.999, -0.0447101778, 0),
             SIMD3(1.4985, -0.0670652667, 0))
        ]

        for fixture in fixtures {
            var tracker = RelativeHeadingTracker()
            XCTAssertTrue(tracker.ingest(RelativeHeadingSample(
                timestamp: 1,
                rotationRate: fixture.rate,
                gravity: fixture.gravity
            )))
            XCTAssertTrue(tracker.ingest(RelativeHeadingSample(
                timestamp: 1.1,
                rotationRate: fixture.rate,
                gravity: fixture.gravity
            )))
            XCTAssertEqual(tracker.measurement(at: 1.1).accumulatedAngle, 0.15, accuracy: 0.000_01)
        }
    }

    func testSixtyHertzSamplesUseTrapezoidalIntegration() {
        var tracker = RelativeHeadingTracker()

        for index in 0...60 {
            let timestamp = Double(index) / 60
            XCTAssertTrue(tracker.ingest(sample(
                at: timestamp,
                rate: SIMD3(0, -timestamp, 0)
            )))
        }

        XCTAssertEqual(
            tracker.measurement(at: 1).accumulatedAngle,
            0.5,
            accuracy: 0.000_001
        )
    }

    func testResetStartsIndependentZeroBasedMeasurement() {
        var tracker = RelativeHeadingTracker()
        XCTAssertTrue(tracker.ingest(sample(at: 0, rate: SIMD3(0, -1, 0))))
        XCTAssertTrue(tracker.ingest(sample(at: 0.1, rate: SIMD3(0, -1, 0))))
        XCTAssertEqual(tracker.measurement(at: 0.1).accumulatedAngle, 0.1, accuracy: 0.000_001)

        tracker.reset()

        XCTAssertEqual(tracker.measurement(at: 0.1).accumulatedAngle, 0, accuracy: 0.000_001)
        XCTAssertEqual(tracker.measurement(at: 0.1).reliability, .unreliable(.notStarted))
    }

    func testRejectsNonMonotonicTimestampAndLatchesFailureUntilReset() {
        var tracker = RelativeHeadingTracker()
        XCTAssertTrue(tracker.ingest(sample(at: 1, rate: SIMD3(0, -1, 0))))
        XCTAssertFalse(tracker.ingest(sample(at: 1, rate: SIMD3(0, -1, 0))))
        XCTAssertEqual(tracker.measurement(at: 1).reliability, .unreliable(.nonMonotonicTimestamp))
        XCTAssertFalse(tracker.ingest(sample(at: 1.05, rate: SIMD3(0, -1, 0))))
        XCTAssertEqual(tracker.measurement(at: 1.05).accumulatedAngle, 0, accuracy: 0.000_001)
    }

    func testRejectsExcessiveSampleGap() {
        var tracker = RelativeHeadingTracker()
        XCTAssertTrue(tracker.ingest(sample(at: 0, rate: SIMD3(0, -1, 0))))
        XCTAssertFalse(tracker.ingest(sample(at: 0.101, rate: SIMD3(0, -1, 0))))
        XCTAssertEqual(tracker.measurement(at: 0.101).reliability, .unreliable(.sampleGap))
        XCTAssertEqual(tracker.measurement(at: 0.101).accumulatedAngle, 0, accuracy: 0.000_001)
    }

    func testRejectsInvalidGravityAndNonFiniteInput() {
        var tracker = RelativeHeadingTracker()
        XCTAssertFalse(tracker.ingest(RelativeHeadingSample(
            timestamp: 0,
            rotationRate: .zero,
            gravity: SIMD3(0, -0.5, 0)
        )))
        XCTAssertEqual(tracker.measurement(at: 0).reliability, .unreliable(.invalidGravity))

        tracker.reset()
        XCTAssertFalse(tracker.ingest(RelativeHeadingSample(
            timestamp: 0,
            rotationRate: SIMD3(.nan, 0, 0),
            gravity: SIMD3(0, -1, 0)
        )))
        XCTAssertEqual(tracker.measurement(at: 0).reliability, .unreliable(.nonFiniteSample))
    }

    func testMeasurementBecomesStaleAfterFreshnessWindow() {
        var tracker = RelativeHeadingTracker()
        XCTAssertTrue(tracker.ingest(sample(at: 2, rate: SIMD3(0, -1, 0))))
        XCTAssertEqual(tracker.measurement(at: 2.15).reliability, .reliable)
        XCTAssertEqual(tracker.measurement(at: 2.151).reliability, .unreliable(.staleSample))
    }

    func testStoreAcceptsSensorSamplesFromNonMainProducer() async {
        let store = RelativeHeadingTrackerStore()
        store.beginMeasurement()
        let samplesAccepted = expectation(description: "background samples accepted")
        let first = sample(at: 3, rate: SIMD3(0, -2, 0))
        let second = sample(at: 3.05, rate: SIMD3(0, -2, 0))

        DispatchQueue.global(qos: .userInteractive).async {
            XCTAssertTrue(store.ingest(first)?.accepted == true)
            XCTAssertTrue(store.ingest(second)?.accepted == true)
            samplesAccepted.fulfill()
        }

        await fulfillment(of: [samplesAccepted], timeout: 1)
        let measurement = store.measurement(at: 3.05)
        XCTAssertEqual(measurement.accumulatedAngle, 0.1, accuracy: 0.000_001)
        XCTAssertEqual(measurement.reliability, .reliable)
    }

    private func sample(at timestamp: TimeInterval,
                        rate: SIMD3<Double>) -> RelativeHeadingSample {
        RelativeHeadingSample(
            timestamp: timestamp,
            rotationRate: rate,
            gravity: SIMD3(0, -1, 0)
        )
    }
}
