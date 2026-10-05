import Foundation

public enum RoomTransitionIntent {
    public static func matches(_ utterance: String) -> Bool {
        let words = utterance.lowercased().split { !$0.isLetter }.map(String.init)
        guard let verb = words.first, ["go", "move", "enter"].contains(verb) else {
            return false
        }

        var remainder = Array(words.dropFirst())
        if verb != "enter", let first = remainder.first, first == "to" || first == "into" {
            remainder.removeFirst()
        }
        if remainder.first == "the" {
            remainder.removeFirst()
        }

        guard remainder.count == 2,
              ["other", "another", "next"].contains(remainder[0]),
              remainder[1] == "room" else {
            return false
        }
        return true
    }
}
