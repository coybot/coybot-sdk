import Foundation

/// Compact, bounded evidence captured at AR ingress, not at a coalescing
/// consumer. This is never a source of motion authority or image retention.
struct FollowTurnPoseEvidenceArchive {
    static let capacity = 128
    private var samples: [FollowTurnBurstPlanner.Sample] = []

    mutating func record(_ source: NavigationPoseSample, at uptime: Double) {
        guard let id = source.frameID else { clear(); return }
        if samples.last?.generation != id.generation { samples.removeAll(keepingCapacity: true) }
        let captured = FollowTurnBurstPlanner.Sample(yaw: source.pose?.yaw ?? .nan,
            sequence: id.sequence, generation: id.generation, sourceTimestamp: source.sourceTimestamp,
            collectedUptime: uptime, clockDomain: "ar_system_uptime",
            healthy: source.rejection(at: uptime, expectedGeneration: id.generation, requireEnriched: true) == nil,
            sourceIdentity: source.source, trackingState: source.trackingQuality.map { String(describing: $0) })
        if samples.count == Self.capacity { samples.removeFirst() }
        samples.append(captured)
    }

    func evidence(from first: ARFrameID, through last: ARFrameID) -> [FollowTurnBurstPlanner.Sample]? {
        guard first.generation == last.generation, last.sequence >= first.sequence,
              last.sequence - first.sequence < UInt64(Self.capacity),
              let start = samples.firstIndex(where: { $0.generation == first.generation && $0.sequence == first.sequence }),
              let end = samples.lastIndex(where: { $0.generation == last.generation && $0.sequence == last.sequence }),
              start <= end else { return nil }
        let result = Array(samples[start...end])
        guard result.count == Int(last.sequence - first.sequence) + 1,
              result.enumerated().allSatisfy({ index, sample in
                  sample.generation == first.generation && sample.sequence == first.sequence + UInt64(index)
              }) else { return nil }
        return result
    }

    mutating func clear() { samples.removeAll(keepingCapacity: true) }
}
