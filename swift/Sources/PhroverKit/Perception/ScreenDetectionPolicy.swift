import CoreGraphics
import Foundation

enum ScreenDetectionPolicy {
    private static let screenLikeLabels: Set<String> = [
        "display", "laptop", "monitor", "screen", "television", "tv",
    ]

    static func shouldRunFallback(for primary: [Detector.Detection]) -> Bool {
        !primary.contains { isScreenLike($0.label) }
    }

    static func merge(primary: [Detector.Detection],
                      fallback: [Detector.Detection],
                      minimumConfidence: Float = 0.90) -> [Detector.Detection] {
        var merged = primary
        for candidate in fallback where isAccepted(candidate, minimumConfidence: minimumConfidence) {
            let canonical = Detector.Detection(
                label: "screen",
                confidence: min(max(candidate.confidence, 0), 1),
                boundingBox: candidate.boundingBox
            )
            guard !merged.contains(where: {
                isScreenLike($0.label) && intersectionOverUnion($0.boundingBox, canonical.boundingBox) >= 0.50
            }) else { continue }
            merged.append(canonical)
        }
        return merged
    }

    static func isScreenLike(_ label: String) -> Bool {
        screenLikeLabels.contains(label.lowercased().trimmingCharacters(in: .whitespacesAndNewlines))
    }

    private static func isAccepted(_ detection: Detector.Detection,
                                   minimumConfidence: Float) -> Bool {
        isScreenLike(detection.label)
            && detection.confidence.isFinite
            && detection.confidence >= minimumConfidence
            && isValid(detection.boundingBox)
    }

    private static func isValid(_ box: CGRect) -> Bool {
        let values = [box.minX, box.minY, box.width, box.height, box.maxX, box.maxY]
        return values.allSatisfy(\.isFinite)
            && box.width > 0
            && box.height > 0
            && box.minX >= 0
            && box.minY >= 0
            && box.maxX <= 1
            && box.maxY <= 1
    }

    private static func intersectionOverUnion(_ lhs: CGRect, _ rhs: CGRect) -> CGFloat {
        let intersection = lhs.intersection(rhs)
        guard !intersection.isNull, intersection.width > 0, intersection.height > 0 else { return 0 }
        let intersectionArea = intersection.width * intersection.height
        let unionArea = lhs.width * lhs.height + rhs.width * rhs.height - intersectionArea
        return unionArea > 0 ? intersectionArea / unionArea : 0
    }
}
