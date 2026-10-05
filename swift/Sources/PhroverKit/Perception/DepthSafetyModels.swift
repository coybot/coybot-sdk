import Foundation

import RoverNav
import simd

struct CameraMountCalibration: Equatable, Sendable {
    let cameraHeight: Double
    let forwardOffset: Double
    let lateralOffset: Double
    let headingAlignment: Double

    init(cameraHeight: Double,
         forwardOffset: Double = 0,
         lateralOffset: Double = 0,
         headingAlignment: Double = 0) {
        self.cameraHeight = cameraHeight
        self.forwardOffset = forwardOffset
        self.lateralOffset = lateralOffset
        self.headingAlignment = headingAlignment
    }

    var validation: DepthSafetyValidation {
        guard cameraHeight.isFinite,
              forwardOffset.isFinite,
              lateralOffset.isFinite,
              headingAlignment.isFinite else {
            return .invalid(.nonFiniteValue)
        }
        guard (0.10...1.50).contains(cameraHeight) else {
            return .invalid(.cameraHeightOutOfRange)
        }
        guard abs(forwardOffset) <= 1.0, abs(lateralOffset) <= 1.0 else {
            return .invalid(.mountOffsetOutOfRange)
        }
        return .valid
    }
}

struct RoverCollisionGeometry: Equatable, Sendable {
    let length: Double
    let width: Double
    let minimumCollisionHeight: Double
    let maximumCollisionHeight: Double
    let lateralSafetyMargin: Double

    init(length: Double,
         width: Double,
         minimumCollisionHeight: Double,
         maximumCollisionHeight: Double,
         lateralSafetyMargin: Double = 0.08) {
        self.length = length
        self.width = width
        self.minimumCollisionHeight = minimumCollisionHeight
        self.maximumCollisionHeight = maximumCollisionHeight
        self.lateralSafetyMargin = lateralSafetyMargin
    }

    var validation: DepthSafetyValidation {
        let values = [length, width, minimumCollisionHeight, maximumCollisionHeight, lateralSafetyMargin]
        guard values.allSatisfy(\.isFinite) else { return .invalid(.nonFiniteValue) }
        guard length > 0, width > 0, lateralSafetyMargin >= 0 else {
            return .invalid(.nonPositiveChassisDimension)
        }
        guard minimumCollisionHeight >= 0,
              maximumCollisionHeight > minimumCollisionHeight else {
            return .invalid(.invalidCollisionHeightBand)
        }
        return .valid
    }
}

enum DepthSafetyValidation: Equatable, Sendable {
    case valid
    case invalid(DepthSafetyInvalidReason)
}

enum DepthSafetyInvalidReason: String, Equatable, Sendable {
    case nonFiniteValue = "non_finite_value"
    case cameraHeightOutOfRange = "camera_height_out_of_range"
    case mountOffsetOutOfRange = "mount_offset_out_of_range"
    case nonPositiveChassisDimension = "non_positive_chassis_dimension"
    case invalidCollisionHeightBand = "invalid_collision_height_band"
}

enum DepthSafetyUnavailableReason: String, Equatable, Sendable {
    case missingRawDepth = "missing_raw_depth"
    case staleRawDepth = "stale_raw_depth"
    case invalidCalibration = "invalid_calibration"
    case invalidCollisionGeometry = "invalid_collision_geometry"
    case malformedDepth = "malformed_depth"
    case blindSweptVolume = "blind_swept_volume"
}

enum DepthSafetyState: Equatable, Sendable {
    case clear
    case caution
    case stop
    case unavailable(DepthSafetyUnavailableReason)

    var telemetryReason: String {
        switch self {
        case .clear: "clear"
        case .caution: "caution"
        case .stop: "stop"
        case .unavailable(let reason): reason.rawValue
        }
    }
}

enum DepthSafetyMotionClass: String, Equatable, Sendable {
    case stopped
    case forward
    case reverse
    case curved
    case rotating

    static func classify(_ command: WheelCommand, epsilon: Double = 0.01) -> Self {
        let average = (command.left + command.right) / 2
        if abs(command.left) <= epsilon, abs(command.right) <= epsilon { return .stopped }
        if abs(average) <= epsilon { return .rotating }
        if average < 0 { return .reverse }
        if abs(command.left - command.right) > epsilon { return .curved }
        return .forward
    }
}

struct DepthSafetyObservation: Equatable, Sendable {
    let state: DepthSafetyState
    let clearance: Double
    let supportCount: Int
    let sampleAge: TimeInterval
    let requiredStoppingDistance: Double
    let motionClass: DepthSafetyMotionClass
    let speedLimit: Double?

    static func unavailable(_ reason: DepthSafetyUnavailableReason,
                            sampleAge: TimeInterval,
                            motionClass: DepthSafetyMotionClass) -> DepthSafetyObservation {
        DepthSafetyObservation(
            state: .unavailable(reason),
            clearance: .infinity,
            supportCount: 0,
            sampleAge: sampleAge,
            requiredStoppingDistance: 0,
            motionClass: motionClass,
            speedLimit: nil
        )
    }
}


struct DepthSafetyCell: Hashable, Sendable {
    let lateralIndex: Int
    let heightIndex: Int
    let forwardIndex: Int
}

struct DepthSafetyProjection: Sendable {
    let width: Int
    let height: Int
    let fx: Double
    let fy: Double
    let cx: Double
    let cy: Double
    let depthValues: [Float]
    let cameraFromWorld: simd_float4x4
    let roverOrigin: SIMD3<Double>
    let roverRight: SIMD3<Double>
    let roverForward: SIMD3<Double>
}

struct DepthSafetySnapshot: Sendable {
    let timestamp: TimeInterval
    let cells: Set<DepthSafetyCell>
    let cellResolution: Double
    let validSampleRatio: Double
    let geometry: RoverCollisionGeometry
    let projection: DepthSafetyProjection?
    let failureReason: DepthSafetyUnavailableReason?

    var occupiedCellCount: Int { cells.count }
}
