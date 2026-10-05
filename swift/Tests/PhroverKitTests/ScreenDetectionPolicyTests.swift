import CoreGraphics
import XCTest
@testable import PhroverKit

final class ScreenDetectionPolicyTests: XCTestCase {
    func testFallbackRunsOnlyWhenPrimaryHasNoScreenLikeObject() {
        XCTAssertTrue(ScreenDetectionPolicy.shouldRunFallback(for: [detection("chair")]))
        XCTAssertFalse(ScreenDetectionPolicy.shouldRunFallback(for: [detection("tv")]))
        XCTAssertFalse(ScreenDetectionPolicy.shouldRunFallback(for: [detection("laptop")]))
        XCTAssertFalse(ScreenDetectionPolicy.shouldRunFallback(for: [detection("screen")]))
    }

    func testMergeAcceptsLocalizedScreenAtThresholdAndPreservesPrimaryObjects() {
        let chair = detection("chair", confidence: 0.98, box: CGRect(x: 0.05, y: 0.1, width: 0.2, height: 0.4))
        let screen = detection("screen", confidence: 0.90, box: CGRect(x: 0.45, y: 0.2, width: 0.4, height: 0.5))

        let merged = ScreenDetectionPolicy.merge(primary: [chair], fallback: [screen])

        XCTAssertEqual(merged.map(\.label), ["chair", "screen"])
        XCTAssertEqual(merged.last?.confidence, 0.90)
        XCTAssertEqual(merged.last?.boundingBox, screen.boundingBox)
    }

    func testMergeRejectsWrongLabelWeakConfidenceAndInvalidBoxes() {
        let invalidBoxes = [
            CGRect(x: 0.1, y: 0.1, width: 0, height: 0.3),
            CGRect(x: -0.1, y: 0.1, width: 0.3, height: 0.3),
            CGRect(x: 0.8, y: 0.1, width: 0.3, height: 0.3),
        ]
        let fallback = [
            detection("poster", confidence: 0.99),
            detection("screen", confidence: 0.89),
        ] + invalidBoxes.map { detection("screen", confidence: 0.99, box: $0) }

        XCTAssertTrue(ScreenDetectionPolicy.merge(primary: [], fallback: fallback).isEmpty)
    }

    func testMergeCanonicalizesAcceptedAliasAndDeduplicatesOverlappingPrimaryScreen() {
        let primary = detection("tv", confidence: 0.95, box: CGRect(x: 0.2, y: 0.2, width: 0.5, height: 0.5))
        let fallback = detection("television", confidence: 0.99, box: CGRect(x: 0.22, y: 0.22, width: 0.48, height: 0.48))

        let merged = ScreenDetectionPolicy.merge(primary: [primary], fallback: [fallback])

        XCTAssertEqual(merged.count, 1)
        XCTAssertEqual(merged[0].label, "tv")
    }

    private func detection(_ label: String,
                           confidence: Float = 0.95,
                           box: CGRect = CGRect(x: 0.2, y: 0.2, width: 0.4, height: 0.4)) -> Detector.Detection {
        Detector.Detection(label: label, confidence: confidence, boundingBox: box)
    }
}
