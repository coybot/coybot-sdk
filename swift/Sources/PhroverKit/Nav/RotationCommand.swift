import Foundation
import RoverNav

enum RotationCommand {
    static let gain = 2.0
    static let maxAngular = 1.5
    private static let depthVisibleArcInnerSpeedRatio = 0.5

    static func command(forYawError error: Double) -> WheelCommand {
        let w = min(max(error * gain, -maxAngular), maxAngular)
        var command = DifferentialDrive.wheels(v: 0,
                                               w: w,
                                               wheelBase: RoverConfig.wheelBase,
                                               maxWheelSpeed: RoverConfig.maxWheelSpeed)
        let peak = max(abs(command.left), abs(command.right))
        guard peak > 0, peak < RoverConfig.minimumRotateWheelSpeed else { return command }

        let scale = RoverConfig.minimumRotateWheelSpeed / peak
        command.left *= scale
        command.right *= scale
        return command
    }

    static func depthVisibleArc(forYawError error: Double) -> WheelCommand {
        let outer = RoverConfig.minimumRotateWheelSpeed
        let inner = outer * depthVisibleArcInnerSpeedRatio
        if error >= 0 {
            return WheelCommand(left: inner, right: outer)
        }
        return WheelCommand(left: outer, right: inner)
    }

    static func depthVisibleArc(matching rotation: WheelCommand) -> WheelCommand {
        depthVisibleArc(forYawError: rotation.right - rotation.left)
    }
}
