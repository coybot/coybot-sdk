import Foundation
import RoverNav
import PhroverKit

/// `RoverDrive` backed by the Godot Depot sim, so the real `FollowController` — the same
/// tracker, the same `FollowPolicy`, the same safety gate — runs unmodified against a
/// walking person actor. `GodotMotion` is the equivalent seam for whole-trip navigation;
/// this is the per-tick servo one following needs.
@MainActor
final class GodotDrive: RoverDrive {
    private let link: GodotLink
    private let rid: String
    /// Every `{v, w}` actually sent, for tests that assert on what was commanded.
    private(set) var commands: [(v: Double, w: Double)] = []

    init(link: GodotLink, rid: String) {
        self.link = link
        self.rid = rid
    }

    var forwardClearance: Double {
        let state = link.call(["op": "phrover_state", "id": rid])
        guard let clearance = godotDouble(state["clearance"]) else { return .infinity }
        // phrover_manager's probe only casts 0.7 m ahead and reports a sentinel 999 when
        // nothing is in that range — not a real long-range measurement, so anything at or
        // above it means "nothing near", not "something 999 m away".
        return clearance >= 900 ? .infinity : clearance
    }

    func send(v: Double, w: Double) async throws {
        commands.append((v: v, w: w))
        _ = link.call(["op": "phrover_drive", "id": rid, "v": v, "w": w])
    }

    func stop() async throws {
        _ = link.call(["op": "phrover_stop", "id": rid])
    }

    /// The sim acknowledges every call synchronously, so the comms watchdog never has a
    /// stale ack to fire on — matching `GodotMotion`, which likewise has no link to lose.
    func lastAck() async -> Date? { Date() }
}
