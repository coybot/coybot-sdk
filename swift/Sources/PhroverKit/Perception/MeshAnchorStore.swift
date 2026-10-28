import ARKit

/// Mesh anchors keyed by ARKit identity, in first-seen order. Generic over the
/// anchor class so the bookkeeping is testable with plain `ARAnchor`s; ARKit
/// offers no supported way to build an `ARMeshAnchor` off-device.
struct MeshAnchorStore<Anchor: ARAnchor> {
    private(set) var anchors: [Anchor] = []

    /// An update replaces the anchor with the same identifier in place.
    mutating func upsert(_ incoming: [Anchor]) {
        var index = Dictionary(anchors.enumerated().map { ($1.identifier, $0) }, uniquingKeysWith: { first, _ in first })
        for anchor in incoming {
            if let existing = index[anchor.identifier] {
                anchors[existing] = anchor
            } else {
                index[anchor.identifier] = anchors.count
                anchors.append(anchor)
            }
        }
    }

    /// Returns how many anchors were retired.
    @discardableResult
    mutating func remove(identifiers: Set<UUID>) -> Int {
        let before = anchors.count
        anchors.removeAll { identifiers.contains($0.identifier) }
        return before - anchors.count
    }

    mutating func removeAll() { anchors.removeAll() }
}
