import CoreVideo
import Foundation
import RoverNav
import simd

enum DepthSafetyEvaluator {
    private static let cellResolution = 0.04
    static let maximumRawDepthAge: TimeInterval = 0.25
    private static let minimumStoppingDistance = 0.30
    private static let assumedLatency: TimeInterval = 0.25
    private static let assumedDeceleration = 0.80
    private static let cautionLeadDistance = 0.35
    private static let cautionSpeedLimit = 0.12
    private static let minimumConnectedCells = 3
    static let minimumSweptVolumeCoverage = 0.90
    private static let maximumGroundOffset = 0.20
    private static let minimumGroundPlaneSamples = 40
    private static let minimumGroundPlaneSampleRatio = 0.05
    private static let minimumGroundPlaneForwardSpan = 0.50
    private static let minimumGroundPlaneLateralSpan = 0.30

    private struct ProjectedSample {
        let lateral: Double
        let vertical: Double
        let forward: Double
    }

    static func ingest(rawDepthMap: CVPixelBuffer,
                       intrinsics: simd_float3x3,
                       intrinsicsImageSize: CGSize? = nil,
                       cameraTransform: simd_float4x4,
                       timestamp: TimeInterval,
                       calibration: CameraMountCalibration,
                       geometry: RoverCollisionGeometry) -> DepthSafetySnapshot {
        guard calibration.validation == .valid else {
            return unavailableSnapshot(timestamp: timestamp, geometry: geometry, reason: .invalidCalibration)
        }
        guard geometry.validation == .valid else {
            return unavailableSnapshot(timestamp: timestamp, geometry: geometry, reason: .invalidCollisionGeometry)
        }

        let width = CVPixelBufferGetWidth(rawDepthMap)
        let height = CVPixelBufferGetHeight(rawDepthMap)
        let referenceWidth = intrinsicsImageSize.map { Double($0.width) } ?? Double(width)
        let referenceHeight = intrinsicsImageSize.map { Double($0.height) } ?? Double(height)
        guard width > 0, height > 0, referenceWidth > 0, referenceHeight > 0 else {
            return unavailableSnapshot(timestamp: timestamp, geometry: geometry, reason: .malformedDepth)
        }
        let scaleX = Double(width) / referenceWidth
        let scaleY = Double(height) / referenceHeight
        let fx = Double(intrinsics[0][0]) * scaleX
        let fy = Double(intrinsics[1][1]) * scaleY
        let cx = Double(intrinsics[2][0]) * scaleX
        let cy = Double(intrinsics[2][1]) * scaleY
        guard fx.isFinite, fy.isFinite, fx > 0, fy > 0, cx.isFinite, cy.isFinite else {
            return unavailableSnapshot(timestamp: timestamp, geometry: geometry, reason: .malformedDepth)
        }

        let cameraPosition = SIMD3<Double>(
            Double(cameraTransform.columns.3.x),
            Double(cameraTransform.columns.3.y),
            Double(cameraTransform.columns.3.z)
        )
        var projectedForward = SIMD3<Double>(
            -Double(cameraTransform.columns.2.x),
            0,
            -Double(cameraTransform.columns.2.z)
        )
        let projectedLength = simd_length(projectedForward)
        guard projectedLength > 0.05, projectedLength.isFinite else {
            return unavailableSnapshot(timestamp: timestamp, geometry: geometry, reason: .blindSweptVolume)
        }
        projectedForward /= projectedLength
        let cameraRight = SIMD3<Double>(-projectedForward.z, 0, projectedForward.x)
        let c = cos(calibration.headingAlignment)
        let s = sin(calibration.headingAlignment)
        let roverForward = projectedForward * c + cameraRight * s
        let roverRight = SIMD3<Double>(-roverForward.z, 0, roverForward.x)
        var roverOrigin = cameraPosition
            - SIMD3<Double>(0, calibration.cameraHeight, 0)
            - roverForward * calibration.forwardOffset
            - roverRight * calibration.lateralOffset

        CVPixelBufferLockBaseAddress(rawDepthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(rawDepthMap, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(rawDepthMap) else {
            return unavailableSnapshot(timestamp: timestamp, geometry: geometry, reason: .malformedDepth)
        }
        let stride = CVPixelBufferGetBytesPerRow(rawDepthMap) / MemoryLayout<Float32>.size
        let values = base.assumingMemoryBound(to: Float32.self)
        var depthValues = [Float](repeating: .nan, count: width * height)
        var validSamples = 0
        var projectedSamples: [ProjectedSample] = []
        projectedSamples.reserveCapacity(width * height / 2)
        var cells = Set<DepthSafetyCell>()
        cells.reserveCapacity(width * height / 8)

        for row in 0..<height {
            for column in 0..<width {
                let raw = values[row * stride + column]
                depthValues[row * width + column] = raw
                let depth = Double(raw)
                guard depth > 0.05, depth.isFinite else { continue }
                validSamples += 1
                let xCamera = (Double(column) - cx) / fx * depth
                let yCamera = -(Double(row) - cy) / fy * depth
                let cameraPoint = SIMD4<Float>(Float(xCamera), Float(yCamera), Float(-depth), 1)
                let world4 = cameraTransform * cameraPoint
                let relative = SIMD3<Double>(
                    Double(world4.x) - roverOrigin.x,
                    Double(world4.y) - roverOrigin.y,
                    Double(world4.z) - roverOrigin.z
                )
                let lateral = simd_dot(relative, roverRight)
                let vertical = relative.y
                let forward = simd_dot(relative, roverForward)
                guard forward >= -geometry.length / 2,
                      forward <= 4.0,
                      abs(lateral) <= 3.0 else { continue }
                projectedSamples.append(ProjectedSample(
                    lateral: lateral,
                    vertical: vertical,
                    forward: forward
                ))
            }
        }

        let sampleRatio = Double(validSamples) / Double(width * height)
        guard validSamples > 0 else {
            return DepthSafetySnapshot(
                timestamp: timestamp,
                cells: cells,
                cellResolution: cellResolution,
                validSampleRatio: sampleRatio,
                geometry: geometry,
                projection: nil,
                failureReason: .malformedDepth
            )
        }
        let groundOffset = inferredGroundOffset(
            samples: projectedSamples,
            validSampleCount: validSamples,
            geometry: geometry
        )
        roverOrigin.y += groundOffset
        for sample in projectedSamples {
            let correctedVertical = sample.vertical - groundOffset
            guard correctedVertical >= geometry.minimumCollisionHeight,
                  correctedVertical <= geometry.maximumCollisionHeight else { continue }
            cells.insert(cell(
                lateral: sample.lateral,
                height: correctedVertical,
                forward: sample.forward
            ))
        }
        let projection = DepthSafetyProjection(
            width: width,
            height: height,
            fx: fx,
            fy: fy,
            cx: cx,
            cy: cy,
            depthValues: depthValues,
            cameraFromWorld: simd_inverse(cameraTransform),
            roverOrigin: roverOrigin,
            roverRight: roverRight,
            roverForward: roverForward
        )
        return DepthSafetySnapshot(
            timestamp: timestamp,
            cells: cells,
            cellResolution: cellResolution,
            validSampleRatio: sampleRatio,
            geometry: geometry,
            projection: projection,
            failureReason: nil
        )
    }

    private static func inferredGroundOffset(samples: [ProjectedSample],
                                             validSampleCount: Int,
                                             geometry: RoverCollisionGeometry) -> Double {
        let candidates = samples.filter {
            $0.vertical >= geometry.minimumCollisionHeight
                && $0.vertical <= maximumGroundOffset
                && $0.forward >= 0.25
                && abs($0.lateral) <= 1.50
        }
        guard !candidates.isEmpty else { return 0 }

        var bins: [Int: [ProjectedSample]] = [:]
        for sample in candidates {
            bins[Int(floor(sample.vertical / cellResolution)), default: []].append(sample)
        }

        let minimumSupport = max(
            minimumGroundPlaneSamples,
            Int(ceil(Double(validSampleCount) * minimumGroundPlaneSampleRatio))
        )
        var bestCluster: [ProjectedSample] = []
        for index in bins.keys {
            let cluster = (index - 1...index + 1).flatMap { bins[$0] ?? [] }
            guard cluster.count >= minimumSupport else { continue }
            let forwardValues = cluster.map(\.forward)
            let lateralValues = cluster.map(\.lateral)
            guard let minimumForward = forwardValues.min(),
                  let maximumForward = forwardValues.max(),
                  let minimumLateral = lateralValues.min(),
                  let maximumLateral = lateralValues.max(),
                  maximumForward - minimumForward >= minimumGroundPlaneForwardSpan,
                  maximumLateral - minimumLateral >= minimumGroundPlaneLateralSpan else { continue }
            if cluster.count > bestCluster.count {
                bestCluster = cluster
            }
        }
        guard !bestCluster.isEmpty else { return 0 }
        let heights = bestCluster.map(\.vertical).sorted()
        return heights[heights.count / 2]
    }

    static func evaluate(_ snapshot: DepthSafetySnapshot,
                         command: WheelCommand,
                          now: TimeInterval,
                          operatorClearedRotation: Bool = false,
                          boundedReadyMotion: Bool = false) -> DepthSafetyObservation {
        let motion = DepthSafetyMotionClass.classify(command)
        let age = max(0, now - snapshot.timestamp)
        guard now.isFinite, snapshot.timestamp.isFinite, now >= snapshot.timestamp,
              snapshot.cellResolution.isFinite, snapshot.cellResolution > 0 else {
            return .unavailable(.malformedDepth, sampleAge: age, motionClass: motion)
        }
        guard snapshot.geometry.validation == .valid else {
            return .unavailable(.invalidCollisionGeometry, sampleAge: age, motionClass: motion)
        }
        if let reason = snapshot.failureReason {
            return .unavailable(reason, sampleAge: age, motionClass: motion)
        }
        guard age <= maximumRawDepthAge else {
            return .unavailable(.staleRawDepth, sampleAge: age, motionClass: motion)
        }
        if motion == .stopped {
            return clearObservation(age: age, motion: motion, stoppingDistance: 0)
        }
        if motion == .reverse {
            return .unavailable(.blindSweptVolume, sampleAge: age, motionClass: motion)
        }

        let confirmedRotation = operatorClearedRotation && motion == .rotating &&
            command.left.isFinite && command.right.isFinite && command.left == -command.right
        let speed = max(abs(command.left), abs(command.right))
        let latencyAndBrakingDistance = speed * (age + assumedLatency)
            + speed * speed / (2 * assumedDeceleration)
        // Thirty centimeters is the total minimum, not an additive buffer.
        // Faster motion still requires the full latency and braking distance.
        let stoppingDistance = max(confirmedRotation ? FollowSearchClearancePolicy.retainedClearance : minimumStoppingDistance, latencyAndBrakingDistance)
        let cautionDistance = stoppingDistance + cautionLeadDistance
        let boundedForward = boundedReadyMotion && motion == .forward && command.left == command.right &&
            command.left > 0 && command.left <= RoverConfig.readySignalWheelMagnitude
        // Keep the full collision/stopping margin. Only the cruise look-ahead
        // beyond stopping distance is unnecessary for the bounded experiment.
        let visibilityHorizon = boundedForward ? stoppingDistance : cautionDistance
        let collisionSections = snapshot.geometry.collisionSections
        let reach = (collisionSections.map {
            hypot(abs($0.forwardOffset) + $0.length / 2, abs($0.lateralOffset) + $0.width / 2 + snapshot.geometry.lateralSafetyMargin)
        }.max() ?? 0) + cautionDistance + snapshot.cellResolution
        // Obstacles seen moments ago stay hazards wherever the live camera is now blind.
        let rememberedHazards = rememberedHazardCells(snapshot: snapshot, reach: reach)
        func finalized(_ observation: DepthSafetyObservation) -> DepthSafetyObservation {
            var result = observation
            result.rememberedHazardCellCount = rememberedHazards.count
            return result
        }
        func checkingCoverage(_ observation: DepthSafetyObservation, horizon: Double) -> DepthSafetyObservation {
            let coverage = sweptVolumeCoverage(snapshot: snapshot, command: command, motion: motion, horizon: horizon)
            var result = coverage.isSufficient(minimumSweptVolumeCoverage) ? observation : DepthSafetyObservation(
                state: .unavailable(.blindSweptVolume), clearance: observation.clearance,
                supportCount: observation.supportCount, sampleAge: age, requiredStoppingDistance: stoppingDistance,
                motionClass: motion, speedLimit: nil)
            result.coverage = coverage
            result.coverageProvenance = boundedForward ? "bounded_ready_motion_stopping_corridor" : "depth"
            return finalized(result)
        }
        var contactTravel: [DepthSafetyCell: Double] = [:]
        for occupiedCell in snapshot.cells.union(rememberedHazards) {
            if confirmedRotation {
                let lateral = (Double(occupiedCell.lateralIndex) + 0.5) * snapshot.cellResolution
                let forward = (Double(occupiedCell.forwardIndex) + 0.5) * snapshot.cellResolution
                let height = (Double(occupiedCell.heightIndex) + 0.5) * snapshot.cellResolution
                let radii = collisionSections.filter {
                    height + snapshot.cellResolution / 2 >= $0.minimumHeight &&
                        height - snapshot.cellResolution / 2 <= $0.maximumHeight
                }.map {
                    hypot(abs($0.forwardOffset) + $0.length / 2,
                          abs($0.lateralOffset) + $0.width / 2 + snapshot.geometry.lateralSafetyMargin)
                        + snapshot.cellResolution / sqrt(2)
                }
                guard let radius = radii.max() else { continue }
                let gap = max(0, hypot(lateral, forward) - radius)
                if gap <= cautionDistance { contactTravel[occupiedCell] = gap }
                continue
            }
            let point = center(of: occupiedCell)
            if let travel = firstContactTravel(
                lateral: point.x,
                height: point.y,
                forward: point.z,
                command: command,
                geometry: snapshot.geometry,
                sections: collisionSections,
                horizon: cautionDistance
            ) {
                contactTravel[occupiedCell] = travel
            }
        }

        let clusters = connectedClusters(in: Set(contactTravel.keys))
            .filter { $0.count >= minimumConnectedCells }
        let nearestCluster = clusters.min {
            nearestContact(in: $0, contactTravel: contactTravel)
                < nearestContact(in: $1, contactTravel: contactTravel)
        }
        if let nearestCluster {
            let clearance = nearestContact(in: nearestCluster, contactTravel: contactTravel)
            let state: DepthSafetyState = clearance <= stoppingDistance ? .stop : .caution
            if state == .stop || confirmedRotation {
                var observation = hazardObservation(
                    state: state,
                    clearance: clearance,
                    supportCount: nearestCluster.count,
                    age: age,
                    stoppingDistance: stoppingDistance,
                    motion: motion
                )
                if confirmedRotation { observation.coverageProvenance = "operator_confirmed_rotation" }
                return finalized(observation)
            }
            return checkingCoverage(hazardObservation(
                state: state,
                clearance: clearance,
                supportCount: nearestCluster.count,
                age: age,
                stoppingDistance: stoppingDistance,
                motion: motion
            ), horizon: min(clearance, visibilityHorizon))
        }

        // Raw depth validity and all observed collision clusters were checked
        // above. The assertion supplements only unobserved pure-rotation sectors.
        if confirmedRotation {
            var observation = clearObservation(age: age, motion: motion, stoppingDistance: stoppingDistance)
            observation.coverageProvenance = "operator_confirmed_rotation"
            return finalized(observation)
        }

        return checkingCoverage(clearObservation(age: age, motion: motion, stoppingDistance: stoppingDistance),
            horizon: visibilityHorizon)
    }

    private static func unavailableSnapshot(timestamp: TimeInterval,
                                            geometry: RoverCollisionGeometry,
                                            reason: DepthSafetyUnavailableReason) -> DepthSafetySnapshot {
        DepthSafetySnapshot(
            timestamp: timestamp,
            cells: [],
            cellResolution: cellResolution,
            validSampleRatio: 0,
            geometry: geometry,
            projection: nil,
            failureReason: reason
        )
    }

    private static func cell(lateral: Double, height: Double, forward: Double) -> DepthSafetyCell {
        DepthSafetyCell(
            lateralIndex: Int(floor(lateral / cellResolution)),
            heightIndex: Int(floor(height / cellResolution)),
            forwardIndex: Int(floor(forward / cellResolution))
        )
    }

    private static func center(of cell: DepthSafetyCell) -> SIMD3<Double> {
        SIMD3(
            (Double(cell.lateralIndex) + 0.5) * cellResolution,
            (Double(cell.heightIndex) + 0.5) * cellResolution,
            (Double(cell.forwardIndex) + 0.5) * cellResolution
        )
    }

    private static func firstContactTravel(lateral: Double,
                                           height: Double,
                                           forward: Double,
                                           command: WheelCommand,
                                           geometry: RoverCollisionGeometry,
                                           sections: [RoverCollisionGeometry.Section],
                                           horizon: Double) -> Double? {
        // A voxel can straddle two height bands. Check the union, not merely
        // the section containing its center, so a chassis-edge hazard survives.
        let intersecting = sections.filter {
            height + cellResolution / 2 >= $0.minimumHeight && height - cellResolution / 2 <= $0.maximumHeight
        }
        let motion = DepthSafetyMotionClass.classify(command)
        if motion == .rotating {
            return intersecting.contains {
                let halfWidth = $0.width / 2 + geometry.lateralSafetyMargin + cellResolution / 2
                let halfLength = $0.length / 2 + cellResolution / 2
                return hypot(lateral, forward) <= hypot(abs($0.forwardOffset) + halfLength, abs($0.lateralOffset) + halfWidth)
            } ? 0 : nil
        }
        let average = (command.left + command.right) / 2
        guard average > 0 else { return nil }
        let delta = command.right - command.left
        let curvature = abs(delta) < 0.001 ? 0 : delta / (RoverConfig.wheelBase * average)
        let steps = sweepStepCount(horizon: horizon, curvature: curvature)
        for index in 0...steps {
            let travel = horizon * Double(index) / Double(steps)
            let pose = pose(at: travel, curvature: curvature)
            let dx = lateral - pose.x
            let dz = forward - pose.z
            let localLateral = dx * cos(pose.heading) + dz * sin(pose.heading)
            let localForward = -dx * sin(pose.heading) + dz * cos(pose.heading)
            if intersecting.contains(where: {
                abs(localLateral - $0.lateralOffset) <= $0.width / 2 + geometry.lateralSafetyMargin + cellResolution / 2 &&
                    abs(localForward - $0.forwardOffset) <= $0.length / 2 + cellResolution / 2
            }) {
                return travel
            }
        }
        return nil
    }

    private static func sweptVolumeCoverage(snapshot: DepthSafetySnapshot,
                                            command: WheelCommand,
                                            motion: DepthSafetyMotionClass,
                                            horizon: Double) -> DepthSafetyCoverage {
        let points = coveragePoints(
            command: command,
            motion: motion,
            geometry: snapshot.geometry,
            horizon: horizon
        )
        var covered = 0, remembered = 0, outside = 0, missing = 0, occluded = 0
        var details: [DepthSafetyCoverage.Point] = []
        var componentCounts: [String: (required: Int, covered: Int)] = [:]
        var boundaryCounts: [CoverageGroup: (required: Int, covered: Int)] = [:]
        for point in points {
            let detail = depthSupport(at: point, snapshot: snapshot)
            var counts = componentCounts[point.section] ?? (0, 0)
            counts.required += 1
            if detail.support.isCovered { counts.covered += 1 }
            componentCounts[point.section] = counts
            let group = CoverageGroup(section: point.section, boundary: point.boundary)
            var edgeCounts = boundaryCounts[group] ?? (0, 0)
            edgeCounts.required += 1
            if detail.support.isCovered { edgeCounts.covered += 1 }
            boundaryCounts[group] = edgeCounts
            if details.count < 256 { details.append(detail) }
            else if !detail.support.isCovered, let replace = details.firstIndex(where: { $0.support.isCovered }) {
                details[replace] = detail // Retain failing points preferentially; decisions still count every sample.
            }
            switch detail.support {
            case .covered: covered += 1
            case .rememberedCovered: covered += 1; remembered += 1
            case .outsideView: outside += 1
            case .missingDepth: missing += 1
            case .occluded: occluded += 1
            }
        }
        var result = DepthSafetyCoverage(horizon: horizon, requiredPoints: points.count, coveredPoints: covered,
            outsideViewPoints: outside, missingDepthPoints: missing, occludedPoints: occluded, projectionAvailable: snapshot.projection != nil)
        result.rememberedPoints = remembered
        result.historyFrames = snapshot.history.count
        result.oldestHistoryAge = snapshot.history.last.map { snapshot.timestamp - $0.timestamp }
        result.points = details
        result.components = snapshot.geometry.collisionSections.compactMap { section in
            componentCounts[section.name].map { .init(name: section.name, required: $0.required, covered: $0.covered) }
        }
        result.geometry = snapshot.geometry
        result.boundaries = boundaryCounts.keys.sorted { ($0.section, $0.boundary) < ($1.section, $1.boundary) }.map { group in
            let counts = boundaryCounts[group]!
            return .init(name: group.section, required: counts.required, covered: counts.covered, boundary: group.boundary)
        }
        // A newly entering side cannot be hidden by a large visible front face.
        // Preserve the legacy box policy until measured sections are configured.
        result.enforceBoundaryCoverage = !snapshot.geometry.sections.isEmpty
        if let projection = snapshot.projection {
            let cameraWorld = simd_inverse(projection.cameraFromWorld).columns.3
            let relative = SIMD3<Double>(Double(cameraWorld.x), Double(cameraWorld.y), Double(cameraWorld.z)) - projection.roverOrigin
            result.projectionSummary = ["image_width_px": Double(projection.width), "image_height_px": Double(projection.height),
                "fx_px": projection.fx, "fy_px": projection.fy, "cx_px": projection.cx, "cy_px": projection.cy,
                "camera_height_above_rover_origin_m": relative.y,
                "camera_forward_offset_m": simd_dot(relative, projection.roverForward),
                "camera_lateral_offset_m": simd_dot(relative, projection.roverRight), "depth_capture_uptime_s": snapshot.timestamp]
            for (name, vector) in [("rover_origin_world", projection.roverOrigin),
                                   ("rover_right_world", projection.roverRight), ("rover_forward_world", projection.roverForward)] {
                for axis in 0..<3 { result.projectionSummary["\(name)_\(["x", "y", "z"][axis])"] = vector[axis] }
            }
            result.cameraFromWorldColumnMajor = (0..<4).flatMap { column in
                (0..<4).map { row in Double(projection.cameraFromWorld[column][row]) }
            }
        }
        return result
    }

    private struct CoverageLocation {
        let section: String
        let boundary: String
        let travel: Double
        let position: SIMD3<Double>
    }

    private struct CoverageGroup: Hashable {
        let section: String
        let boundary: String
    }

    private static func coveragePoints(command: WheelCommand,
                                       motion: DepthSafetyMotionClass,
                                       geometry: RoverCollisionGeometry,
                                        horizon: Double) -> [CoverageLocation] {
        var result: [CoverageLocation] = []
        for section in geometry.collisionSections {
            let halfWidth = section.width / 2 + geometry.lateralSafetyMargin
            let halfLength = section.length / 2
            let inset = min(cellResolution / 2, (section.maximumHeight - section.minimumHeight) / 2)
            let heights = Array(Set([section.minimumHeight + inset,
                (section.minimumHeight + section.maximumHeight) / 2, section.maximumHeight - inset])).sorted()
            if motion == .rotating {
                let radius = hypot(abs(section.forwardOffset) + halfLength, abs(section.lateralOffset) + halfWidth)
                for index in 0..<36 {
                    let angle = 2 * Double.pi * Double(index) / 36
                    for height in heights {
                        result.append(.init(section: section.name, boundary: "rotation_\(index)", travel: 0,
                            position: SIMD3(radius * sin(angle), height, radius * cos(angle))))
                    }
                }
                continue
            }
            let average = (command.left + command.right) / 2
            guard average > 0 else { return [] }
            let delta = command.right - command.left
            let curvature = abs(delta) < 0.001 ? 0 : delta / (RoverConfig.wheelBase * average)
            let boundary = enteringBoundary(section: section, margin: geometry.lateralSafetyMargin,
                curvature: curvature, sectioned: !geometry.sections.isEmpty)
            let steps = max(1, Int(ceil(horizon / cellResolution)))
            for index in 0...steps {
                let travel = horizon * Double(index) / Double(steps)
                let pose = pose(at: travel, curvature: curvature)
                for (label, localLateral, localForward) in boundary {
                    let lateral = pose.x + localLateral * cos(pose.heading) - localForward * sin(pose.heading)
                    let forward = pose.z + localLateral * sin(pose.heading) + localForward * cos(pose.heading)
                    for height in heights {
                        result.append(.init(section: section.name, boundary: label, travel: travel,
                            position: SIMD3(lateral, height, forward)))
                    }
                }
            }
        }
        return result
    }

    /// Offset bodies can enter space along their sides/rear during an arc.
    /// In rover coordinates a boundary point's velocity per metre travelled is
    /// (-curvature * forward, 1 + curvature * lateral). Sample outward-moving
    /// portions of faces, including a midpoint in each entering portion. Shared
    /// corners remain checks in BOTH face groups; they cannot erase a side gate.
    /// Legacy box sampling is unchanged.
    private static func enteringBoundary(section: RoverCollisionGeometry.Section, margin: Double,
                                         curvature: Double, sectioned: Bool) -> [(String, Double, Double)] {
        let halfWidth = section.width / 2 + margin
        let halfLength = section.length / 2
        let xs = [section.lateralOffset - halfWidth, section.lateralOffset, section.lateralOffset + halfWidth]
        let front = section.forwardOffset + halfLength
        if !sectioned {
            return zip(["left", "center", "right"], xs).map { ($0.0, $0.1, front) }
        }
        let back = section.forwardOffset - halfLength
        var result: [(String, Double, Double)] = []
        func face(_ label: String, from start: Double, to end: Double,
                  slope: Double, intercept: Double, position: (Double) -> (Double, Double)) {
            let startVelocity = slope * start + intercept
            let endVelocity = slope * end + intercept
            guard max(startVelocity, endVelocity) > 0 else { return }
            var lower = start, upper = end
            if startVelocity < 0 { lower = -intercept / slope }
            if endVelocity < 0 { upper = -intercept / slope }
            for coordinate in [lower, (lower + upper) / 2, upper] {
                let (x, z) = position(coordinate)
                result.append((label, x, z))
            }
        }
        face("front", from: xs[0], to: xs[2], slope: curvature, intercept: 1) { ($0, front) }
        face("rear", from: xs[0], to: xs[2], slope: -curvature, intercept: -1) { ($0, back) }
        face("left_side", from: back, to: front, slope: curvature, intercept: 0) { (xs[0], $0) }
        face("right_side", from: back, to: front, slope: -curvature, intercept: 0) { (xs[2], $0) }
        return result
    }

    private static func depthSupport(at location: CoverageLocation,
                                     snapshot: DepthSafetySnapshot) -> DepthSafetyCoverage.Point {
        func detail(_ sight: Sight, support: DepthSafetyCoverage.Point.Support? = nil,
                    rememberedAge: Double? = nil) -> DepthSafetyCoverage.Point {
            .init(section: location.section, boundary: location.boundary, travel: location.travel,
                roverPosition: location.position, support: support ?? sight.support, outsideEdges: sight.edges,
                pixelX: sight.pixelX, pixelY: sight.pixelY, cameraDepth: sight.cameraDepth,
                measuredDepth: sight.measuredDepth, rememberedAge: rememberedAge)
        }
        guard let projection = snapshot.projection else { return detail(Sight(support: .missingDepth)) }
        let world = worldPoint(location.position, in: projection)
        let live = sight(of: world, in: projection)
        guard live.support == .outsideView else { return detail(live) }
        // Only space the live camera cannot see at all is answered from memory,
        // newest viewpoint first. A remembered occlusion stays a failure.
        for frame in snapshot.history {
            let age = snapshot.timestamp - frame.timestamp
            guard age > 0, age <= DepthVisibilityMemory.maximumAge else { continue }
            switch sight(of: world, in: frame.projection).support {
            case .covered: return detail(live, support: .rememberedCovered, rememberedAge: age)
            case .occluded: return detail(live, support: .occluded, rememberedAge: age)
            case .outsideView, .missingDepth, .rememberedCovered: continue
            }
        }
        return detail(live)
    }

    /// One projection of a world point into one captured depth frame.
    private struct Sight {
        var support: DepthSafetyCoverage.Point.Support
        var edges: [String] = []
        var pixelX: Double?
        var pixelY: Double?
        var cameraDepth: Double?
        var measuredDepth: Double?
    }

    private static func worldPoint(_ roverPoint: SIMD3<Double>, in projection: DepthSafetyProjection) -> SIMD3<Double> {
        projection.roverOrigin
            + projection.roverRight * roverPoint.x
            + SIMD3<Double>(0, roverPoint.y, 0)
            + projection.roverForward * roverPoint.z
    }

    private static func sight(of world: SIMD3<Double>, in projection: DepthSafetyProjection) -> Sight {
        let camera = projection.cameraFromWorld * SIMD4<Float>(
            Float(world.x), Float(world.y), Float(world.z), 1
        )
        let targetDepth = -Double(camera.z)
        var result = Sight(support: .outsideView, cameraDepth: targetDepth.isFinite ? targetDepth : nil)
        guard targetDepth > 0.05, targetDepth.isFinite else {
            result.edges = ["behind_or_near_camera"]
            return result
        }
        let px = (projection.fx * Double(camera.x) / targetDepth + projection.cx).rounded()
        let py = (projection.cy - projection.fy * Double(camera.y) / targetDepth).rounded()
        result.pixelX = px.isFinite ? px : nil
        result.pixelY = py.isFinite ? py : nil
        guard px.isFinite, py.isFinite else {
            result.edges = ["invalid_projection"]
            return result
        }
        if px < 0 { result.edges.append("left") }
        if px >= Double(projection.width) { result.edges.append("right") }
        if py < 0 { result.edges.append("top") }
        if py >= Double(projection.height) { result.edges.append("bottom") }
        guard result.edges.isEmpty else { return result }
        let pixelX = Int(px), pixelY = Int(py)
        result.support = .missingDepth
        guard pixelY * projection.width + pixelX < projection.depthValues.count else { return result }
        let measured = Double(projection.depthValues[pixelY * projection.width + pixelX])
        result.measuredDepth = measured.isFinite ? measured : nil
        guard measured > 0.05, measured.isFinite else { return result }
        result.support = measured + cellResolution >= targetDepth ? .covered : .occluded
        return result
    }

    /// Occupied cells from retained frames, re-expressed in the live rover frame,
    /// that the live camera cannot currently see. Where live depth reaches the
    /// spot, live cells already describe it, including an obstacle that has left.
    private static func rememberedHazardCells(snapshot: DepthSafetySnapshot, reach: Double) -> Set<DepthSafetyCell> {
        guard let live = snapshot.projection, !snapshot.history.isEmpty, reach.isFinite else { return [] }
        let geometry = snapshot.geometry
        var result = Set<DepthSafetyCell>()
        for frame in snapshot.history {
            let age = snapshot.timestamp - frame.timestamp
            guard age > 0, age <= DepthVisibilityMemory.maximumAge else { continue }
            for cell in frame.cells {
                let local = SIMD3<Double>(
                    (Double(cell.lateralIndex) + 0.5) * frame.cellResolution,
                    (Double(cell.heightIndex) + 0.5) * frame.cellResolution,
                    (Double(cell.forwardIndex) + 0.5) * frame.cellResolution)
                let world = worldPoint(local, in: frame.projection)
                let relative = world - live.roverOrigin
                let lateral = simd_dot(relative, live.roverRight)
                let forward = simd_dot(relative, live.roverForward)
                let height = relative.y
                guard hypot(lateral, forward) <= reach,
                      height >= geometry.minimumCollisionHeight,
                      height <= geometry.maximumCollisionHeight,
                      !sight(of: world, in: live).support.isCovered else { continue }
                result.insert(self.cell(lateral: lateral, height: height, forward: forward))
            }
        }
        return result
    }

    private static func sweepStepCount(horizon: Double, curvature: Double) -> Int {
        let translationSteps = Int(ceil(horizon / (cellResolution / 2)))
        let rotationSteps = Int(ceil(abs(curvature * horizon) / (Double.pi / 90)))
        return max(1, max(translationSteps, rotationSteps))
    }

    private static func pose(at travel: Double,
                             curvature: Double) -> (x: Double, z: Double, heading: Double) {
        guard abs(curvature) >= 0.000_001 else { return (0, travel, 0) }
        let heading = travel * curvature
        let radius = 1 / curvature
        return (-radius * (1 - cos(heading)), radius * sin(heading), heading)
    }

    private static func connectedClusters(in cells: Set<DepthSafetyCell>) -> [Set<DepthSafetyCell>] {
        var remaining = cells
        var clusters: [Set<DepthSafetyCell>] = []
        while let start = remaining.first {
            var cluster: Set<DepthSafetyCell> = [start]
            var queue = [start]
            remaining.remove(start)
            while let current = queue.popLast() {
                for dl in -1...1 {
                    for dh in -1...1 {
                        for df in -1...1 where dl != 0 || dh != 0 || df != 0 {
                            let neighbor = DepthSafetyCell(
                                lateralIndex: current.lateralIndex + dl,
                                heightIndex: current.heightIndex + dh,
                                forwardIndex: current.forwardIndex + df
                            )
                            if remaining.remove(neighbor) != nil {
                                cluster.insert(neighbor)
                                queue.append(neighbor)
                            }
                        }
                    }
                }
            }
            clusters.append(cluster)
        }
        return clusters
    }

    private static func nearestContact(in cluster: Set<DepthSafetyCell>,
                                       contactTravel: [DepthSafetyCell: Double]) -> Double {
        cluster.compactMap { contactTravel[$0] }.min() ?? .infinity
    }

    private static func hazardObservation(state: DepthSafetyState,
                                          clearance: Double,
                                          supportCount: Int,
                                          age: TimeInterval,
                                          stoppingDistance: Double,
                                          motion: DepthSafetyMotionClass) -> DepthSafetyObservation {
        DepthSafetyObservation(
            state: state,
            clearance: clearance,
            supportCount: supportCount,
            sampleAge: age,
            requiredStoppingDistance: stoppingDistance,
            motionClass: motion,
            speedLimit: state == .caution ? cautionSpeedLimit : nil
        )
    }

    private static func clearObservation(age: TimeInterval,
                                         motion: DepthSafetyMotionClass,
                                         stoppingDistance: Double) -> DepthSafetyObservation {
        DepthSafetyObservation(
            state: .clear,
            clearance: .infinity,
            supportCount: 0,
            sampleAge: age,
            requiredStoppingDistance: stoppingDistance,
            motionClass: motion,
            speedLimit: nil
        )
    }
}
