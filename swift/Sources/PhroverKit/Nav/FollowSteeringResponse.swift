import Foundation
import RoverNav

/// Detect persistent heading error during acknowledged arcs across speed changes.
/// This requests a stopped alignment; it never authorizes a wheel command.
struct FollowSteeringResponse {
    private var window: (time: Double, yaw: Double, bearing: Double, direction: Double)?
    private var lastTime: Double?

    mutating func reset() { window = nil; lastTime = nil }

    mutating func observe(command: WheelCommand, yaw: Double, bearing: Double,
                          now: Double, eligible: Bool) -> Double? {
        let direction = command.right > command.left ? 1.0 : -1.0
        guard eligible, now.isFinite, yaw.isFinite, bearing.isFinite,
              command.left > 0, command.right > 0,
              abs(command.left - command.right) / max(command.left, command.right) >= 0.1,
              bearing * direction > Double.pi / 15 else { reset(); return nil }
        if let lastTime, now < lastTime || now - lastTime > 0.3 { reset() }
        lastTime = now
        guard let window, window.direction == direction else {
            window = (now, yaw, abs(bearing), direction)
            return nil
        }
        guard now - window.time >= 2 else { return nil }
        let delta = atan2(sin(yaw - window.yaw), cos(yaw - window.yaw)) * direction
        let improvement = window.bearing - abs(bearing)
        self.window = (now, yaw, abs(bearing), direction)
        return delta < Double.pi / 60 || improvement < Double.pi / 60 ? direction : nil
    }
}

/// Near-target low commands with no measured translation should settle into
/// the coordinator's existing hysteretic hold, not accumulate a stall.
struct FollowCreepResponse {
    private var window: (time: Double, position: Vec2)?
    private var lastTime: Double?

    mutating func observe(command: WheelCommand, position: Vec2, now: Double, eligible: Bool) -> Bool {
        guard eligible, command.left > 0, command.right > 0,
              max(command.left, command.right) <= 0.1 else {
            window = nil; lastTime = nil
            return false
        }
        if let lastTime, now < lastTime || now - lastTime > 0.3 { window = nil }
        lastTime = now
        guard let window, position.distance(to: window.position) <= 0.01 else {
            window = (now, position)
            return false
        }
        return now - window.time >= 1
    }
}
