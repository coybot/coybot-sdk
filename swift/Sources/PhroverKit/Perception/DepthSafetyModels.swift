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
    /// Measured, vertically stacked parts of the assembly. Offsets are relative
    /// to the chassis center, not the camera. Empty sections retain the box model.
    struct Section: Equatable, Sendable {
        let name: String
        let length: Double
        let width: Double
        let minimumHeight: Double
        let maximumHeight: Double
        var forwardOffset: Double = 0
        var lateralOffset: Double = 0
    }
    let length: Double
    let width: Double
    let minimumCollisionHeight: Double
    let maximumCollisionHeight: Double
    let lateralSafetyMargin: Double
    let sections: [Section]

    init(length: Double,
         width: Double,
         minimumCollisionHeight: Double,
         maximumCollisionHeight: Double,
          lateralSafetyMargin: Double = 0.08,
          sections: [Section] = []) {
        self.length = length
        self.width = width
        self.minimumCollisionHeight = minimumCollisionHeight
        self.maximumCollisionHeight = maximumCollisionHeight
        self.lateralSafetyMargin = lateralSafetyMargin
        self.sections = sections
    }

    var collisionSections: [Section] {
        sections.isEmpty ? [.init(name: "bounding_box", length: length, width: width,
            minimumHeight: minimumCollisionHeight, maximumHeight: maximumCollisionHeight)] : sections
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
        if !sections.isEmpty {
            guard sections.count <= 8, Set(sections.map(\.name)).count == sections.count else {
                return .invalid(.invalidCollisionSections)
            }
            var lower = minimumCollisionHeight
            for section in sections {
                let dimensions = [section.length, section.width, section.minimumHeight, section.maximumHeight,
                                  section.forwardOffset, section.lateralOffset]
                guard !section.name.isEmpty, section.name.count <= 64, dimensions.allSatisfy(\.isFinite),
                      section.length > 0, section.width > 0,
                      section.maximumHeight > section.minimumHeight,
                      section.minimumHeight == lower,
                      abs(section.forwardOffset) + section.length / 2 <= length / 2,
                      abs(section.lateralOffset) + section.width / 2 <= width / 2 else {
                    return .invalid(.invalidCollisionSections)
                }
                lower = section.maximumHeight
            }
            guard lower == maximumCollisionHeight else { return .invalid(.invalidCollisionSections) }
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
    case invalidCollisionSections = "invalid_collision_sections"
}

public enum DepthSafetyUnavailableReason: String, Equatable, Sendable {
    case missingRawDepth = "missing_raw_depth"
    case staleRawDepth = "stale_raw_depth"
    case invalidCalibration = "invalid_calibration"
    case invalidCollisionGeometry = "invalid_collision_geometry"
    case malformedDepth = "malformed_depth"
    case blindSweptVolume = "blind_swept_volume"
}

/// Which depth evidence a follow-me motion needs. See docs/follow-depth-policy.md.
enum FollowDepthPolicy: String, Sendable {
    /// At least 90% of the motion-volume samples must be visible (live or remembered).
    case fullVolume = "full_volume"
    /// No visibility fraction is required, as in Astral. Astral's own forward gate
    /// (`ObstacleGuard`, central-region clearance) runs in the follow controller under
    /// both policies; depth freshness, missing depth and visible obstacles still stop.
    case astralCompatible = "astral_compatible"
}

/// Set only by the follow-me controller around its own depth reads. Ordinary
/// navigation and Silent Search never see anything but `fullVolume`.
enum FollowDepthPolicyScope {
    @TaskLocal static var current: FollowDepthPolicy = .fullVolume
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
    var coverageProvenance: String = "depth"
    var coverage: DepthSafetyCoverage?
    /// Occupied cells contributed by retained frames where the live camera is blind.
    var rememberedHazardCellCount = 0
    var visibilityPolicy: FollowDepthPolicy = .fullVolume
    /// Whether the 90% visibility requirement fails for this command, under either policy.
    var fullVolumeWouldBlock: Bool?
    /// Work performed by the evaluator, not evidence of physical clearance.
    var collisionCandidateCount: Int?
    var rememberedProjectionCount: Int?
    var evaluationDuration: TimeInterval?

    static func unavailable(_ reason: DepthSafetyUnavailableReason,
                             sampleAge: TimeInterval,
                            motionClass: DepthSafetyMotionClass,
                            visibilityPolicy: FollowDepthPolicy = .fullVolume) -> DepthSafetyObservation {
        DepthSafetyObservation(
            state: .unavailable(reason),
            clearance: .infinity,
            supportCount: 0,
            sampleAge: sampleAge,
            requiredStoppingDistance: 0,
            motionClass: motionClass,
            speedLimit: nil,
            visibilityPolicy: visibilityPolicy
        )
    }
}

struct DepthSafetyCoverage: Equatable, Sendable {
    struct Point: Equatable, Sendable {
        enum Support: String, Sendable {
            case covered, rememberedCovered = "remembered_covered"
            case outsideView = "outside_view", missingDepth = "missing_depth", occluded
            var isCovered: Bool { self == .covered || self == .rememberedCovered }
        }
        let section: String
        let boundary: String
        let travel: Double
        let roverPosition: SIMD3<Double>
        let support: Support
        let outsideEdges: [String]
        let pixelX: Double?
        let pixelY: Double?
        let cameraDepth: Double?
        let measuredDepth: Double?
        /// Age of the retained frame that decided an out-of-view point; nil when live.
        var rememberedAge: Double? = nil
    }
    struct Component: Equatable, Sendable {
        let name: String
        let required: Int
        let covered: Int
        var boundary: String? = nil
        var fraction: Double { required > 0 ? Double(covered) / Double(required) : 0 }
    }
    let horizon: Double
    let requiredPoints: Int
    let coveredPoints: Int
    let outsideViewPoints: Int
    let missingDepthPoints: Int
    let occludedPoints: Int
    let projectionAvailable: Bool
    /// Included in `coveredPoints`: out of live view, seen free by a retained frame.
    var rememberedPoints = 0
    var historyFrames = 0
    var oldestHistoryAge: Double?
    var points: [Point] = []
    var components: [Component] = []
    var boundaries: [Component] = []
    var enforceBoundaryCoverage = false
    var projectionSummary: [String: Double] = [:]
    var cameraFromWorldColumnMajor: [Double] = []
    var geometry: RoverCollisionGeometry?
    var fraction: Double { requiredPoints > 0 ? Double(coveredPoints) / Double(requiredPoints) : 0 }
    var limitingComponent: Component? {
        (components + (enforceBoundaryCoverage ? boundaries : [])).min { $0.fraction < $1.fraction }
    }
    var limitingFraction: Double { limitingComponent?.fraction ?? fraction }
    func isSufficient(_ required: Double) -> Bool { fraction >= required && limitingFraction >= required }

    /// Values are captured during evaluation; serialization never reads a newer
    /// camera frame or substitutes display-preview coordinates for depth pixels.
    func diagnosticFields(includePoints: Bool) -> [String: FollowDiagnosticValue] {
        func number(_ value: Double?) -> FollowDiagnosticValue {
            value.flatMap { $0.isFinite ? .number($0) : nil } ?? .null
        }
        let limiting = limitingComponent
        func componentFields(_ component: Component) -> FollowDiagnosticValue {
            .object(["name": .string(component.name), "boundary": component.boundary.map { .string($0) } ?? .null,
                "required": .number(Double(component.required)), "covered": .number(Double(component.covered)),
                "fraction": number(component.fraction)])
        }
        var fields: [String: FollowDiagnosticValue] = [
            "depth_limiting_component": limiting.map { .string($0.name + ($0.boundary.map { ": " + $0 } ?? "")) } ?? .null,
            "depth_limiting_component_fraction": number(limitingFraction),
            "depth_boundary_coverage_enforced": .bool(enforceBoundaryCoverage),
            "depth_remembered_points": .number(Double(rememberedPoints)),
            "depth_history_frames": .number(Double(historyFrames)),
            "depth_history_oldest_age_s": number(oldestHistoryAge),
            "depth_coverage_components": .array(components.map(componentFields)),
            "depth_coverage_boundaries": .array(boundaries.map(componentFields))]
        guard includePoints else { return fields }
        fields["depth_projection_coordinates"] = .string("raw_depth_pixels_x_right_y_down;rover_metres_x_right_y_up_z_forward")
        fields["depth_projection"] = .object(projectionSummary.mapValues { number($0) })
        fields["depth_projection_camera_from_world_column_major"] = .array(cameraFromWorldColumnMajor.map { number($0) })
        fields["depth_coverage_points_truncated"] = .bool(points.count < requiredPoints)
        fields["depth_coverage_points_limit"] = .number(256)
        fields["depth_coverage_points_retention"] = .string("prefer_uncovered_when_truncated")
        fields["depth_coverage_points"] = .array(points.map { point in
            .object(["section": .string(point.section), "boundary": .string(point.boundary),
                "travel_m": number(point.travel), "lateral_m": number(point.roverPosition.x),
                "height_m": number(point.roverPosition.y), "forward_m": number(point.roverPosition.z),
                "support": .string(point.support.rawValue), "outside_edges": .array(point.outsideEdges.map { .string($0) }),
                "pixel_x": number(point.pixelX), "pixel_y": number(point.pixelY),
                "camera_depth_m": number(point.cameraDepth), "measured_depth_m": number(point.measuredDepth),
                "remembered_age_s": number(point.rememberedAge)])
        })
        if let geometry {
            fields["depth_geometry_model"] = .string(geometry.sections.isEmpty ? "conservative_bounding_box" : "height_sections")
            fields["depth_geometry_lateral_margin_m"] = number(geometry.lateralSafetyMargin)
            fields["depth_geometry_sections"] = .array(geometry.collisionSections.map { section in
                .object(["name": .string(section.name), "length_m": number(section.length), "width_m": number(section.width),
                    "minimum_height_m": number(section.minimumHeight), "maximum_height_m": number(section.maximumHeight),
                    "forward_offset_m": number(section.forwardOffset), "lateral_offset_m": number(section.lateralOffset)])
            })
        }
        return fields
    }
}

/// Set only for controller-owned bounded readiness or its recovery probe.
/// Ordinary navigation and pursuit never inherit a reduced visibility horizon.
enum ReadyMotionDepthScope {
    @TaskLocal static var active = false
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
    /// Retained earlier frames of the same tracked session, newest first.
    var history: [DepthVisibilityMemory.Frame] = []

    var occupiedCellCount: Int { cells.count }
}

/// Recent depth frames from one continuously tracked AR session. The live frame
/// stays authoritative wherever it can see; retained frames only answer for
/// space the live camera cannot see at all. See docs/remembered-depth-visibility.md.
struct DepthVisibilityMemory: Sendable {
    struct Frame: Sendable {
        let timestamp: TimeInterval
        let projection: DepthSafetyProjection
        let cells: Set<DepthSafetyCell>
        let cellResolution: Double
    }

    /// Space no live sensor can see is trusted for at most this long.
    static let maximumAge: TimeInterval = 2.0
    static let minimumSpacing: TimeInterval = 0.10
    static let maximumFrames = 20

    /// Oldest first.
    private(set) var frames: [Frame] = []

    mutating func clear() { frames.removeAll() }

    /// Hands `snapshot` the retained frames older than it, then retains it if
    /// its own evaluation succeeded and it is far enough from the newest frame.
    mutating func attach(to snapshot: inout DepthSafetySnapshot) {
        let now = snapshot.timestamp
        guard now.isFinite else { clear(); return }
        frames.removeAll { frame in
            let age = now - frame.timestamp
            return !(age > 0 && age <= Self.maximumAge)
        }
        snapshot.history = frames.reversed()
        guard snapshot.failureReason == nil, let projection = snapshot.projection else { return }
        if let newest = frames.last, now - newest.timestamp < Self.minimumSpacing { return }
        frames.append(.init(timestamp: now, projection: projection, cells: snapshot.cells,
                            cellResolution: snapshot.cellResolution))
        if frames.count > Self.maximumFrames { frames.removeFirst(frames.count - Self.maximumFrames) }
    }
}
