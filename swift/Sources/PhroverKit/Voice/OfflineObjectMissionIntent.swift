import Foundation

public enum LocalObjectColor: String, CaseIterable, Hashable, Sendable {
    case black, white, gray, red, orange, yellow, green, blue, purple, brown
}

public struct OfflineObjectMissionIntent: Equatable, Sendable {
    public let objectQuery: String
    public let targetLabel: String
    public let requestedColors: Set<LocalObjectColor>
    public let searchOtherRooms: Bool
    public let shouldReturn: Bool

    public init(objectQuery: String,
                targetLabel: String,
                requestedColors: Set<LocalObjectColor>,
                searchOtherRooms: Bool,
                shouldReturn: Bool) {
        self.objectQuery = objectQuery
        self.targetLabel = targetLabel
        self.requestedColors = requestedColors
        self.searchOtherRooms = searchOtherRooms
        self.shouldReturn = shouldReturn
    }
}

public enum OfflineObjectMissionIntentParser {
    private static let prefixes = ["navigate to", "drive to", "look for", "go to", "find"]
    private static let articles: Set<String> = ["a", "an", "the"]

    public static func parse(_ utterance: String) -> OfflineObjectMissionIntent? {
        var remainder = normalize(utterance)
        guard let prefix = prefixes.first(where: { remainder == $0 || remainder.hasPrefix($0 + " ") }) else {
            return nil
        }
        remainder = String(remainder.dropFirst(prefix.count)).trimmingCharacters(in: .whitespaces)

        var shouldReturn = false
        for phrase in [
            "and come back", "then come back", "come back",
            "and go back", "then go back", "go back",
            "and return", "then return", "return",
        ] {
            if remainder == phrase {
                shouldReturn = true
                remainder = ""
                break
            }
            if remainder.hasSuffix(" " + phrase) {
                shouldReturn = true
                remainder.removeLast(phrase.count + 1)
                remainder = remainder.trimmingCharacters(in: .whitespaces)
                break
            }
        }

        var searchOtherRooms = false
        for phrase in [
            "in the other room", "in another room", "in our room", "in my room", "other room",
        ] {
            if remainder == phrase {
                searchOtherRooms = true
                remainder = ""
                break
            }
            if remainder.contains(" " + phrase) {
                searchOtherRooms = true
                remainder = remainder.replacingOccurrences(of: " " + phrase, with: "")
                break
            }
        }

        var words = remainder.split(separator: " ").map(String.init)
        while let first = words.first, articles.contains(first) {
            words.removeFirst()
        }

        var requestedColors = Set<LocalObjectColor>()
        var colorWords: [String] = []
        while let first = words.first,
              let color = LocalObjectColor(rawValue: first) {
            requestedColors.insert(color)
            colorWords.append(color.rawValue)
            words.removeFirst()
            if words.first == "and" {
                words.removeFirst()
            }
        }

        guard !words.isEmpty, !words.contains("and") else { return nil }
        guard !words.contains(where: { ["bring", "take", "get", "tell", "say"].contains($0) }) else {
            return nil
        }

        let targetLabel = canonicalize(words.joined(separator: " "))
        guard !targetLabel.isEmpty else { return nil }
        let objectQuery = (colorWords + [targetLabel]).joined(separator: " ")
        return OfflineObjectMissionIntent(objectQuery: objectQuery,
                                          targetLabel: targetLabel,
                                          requestedColors: requestedColors,
                                          searchOtherRooms: searchOtherRooms,
                                          shouldReturn: shouldReturn)
    }

    private static func normalize(_ utterance: String) -> String {
        utterance.lowercased().map { $0.isLetter || $0.isNumber || $0 == " " ? $0 : " " }
            .reduce(into: "") { result, character in
                if character == " " && result.last == " " { return }
                result.append(character)
            }
            .trimmingCharacters(in: .whitespaces)
    }

    private static func canonicalize(_ label: String) -> String {
        switch label {
        case "fridge": return "refrigerator"
        case "couch": return "sofa"
        case "monitor", "computer monitor", "display", "computer display",
             "screen", "computer screen", "television", "tv":
            return "screen"
        default: return label
        }
    }
}
