import CoreVideo
import Foundation
import RoverNav
import simd

enum DepthSafetyEvaluator {
    private static let cellResolution = 0.04
    private static let maximumRawDepthAge: TimeInterval = 0.25
    private static let baseStoppingMargin = 0.25
    private static let assumedLatency: TimeInterval = 0.25
    private static let assumedDeceleration = 0.80
    private static let cautionLeadDistance = 0.35
    private static let cautionSpeedLimit = 0.12
    private static let minimumConnectedCells = 3
    private static let minimumSweptVolumeCoverage = 0.90
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
                         now: TimeInterval) -> DepthSafetyObservation {
        let motion = DepthSafetyMotionClass.classify(command)
        let age = max(0, now - snapshot.timestamp)
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

        let speed = max(abs(command.left), abs(command.right))
        let stoppingDistance = baseStoppingMargin
            + speed * (age + assumedLatency)
            + speed * speed / (2 * assumedDeceleration)
        let cautionDistance = stoppingDistance + cautionLeadDistance
        var contactTravel: [DepthSafetyCell: Double] = [:]
        for occupiedCell in snapshot.cells {
            let point = center(of: occupiedCell)
            if let travel = firstContactTravel(
                lateral: point.x,
                forward: point.z,
                command: command,
                geometry: snapshot.geometry,
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
            if state == .stop {
                return hazardObservation(
                    state: state,
                    clearance: clearance,
                    supportCount: nearestCluster.count,
                    age: age,
                    stoppingDistance: stoppingDistance,
                    motion: motion
                )
            }
            guard sweptVolumeCoverage(
                snapshot: snapshot,
                command: command,
                motion: motion,
                horizon: clearance
            ) >= minimumSweptVolumeCoverage else {
                return .unavailable(.blindSweptVolume, sampleAge: age, motionClass: motion)
            }
            return hazardObservation(
                state: state,
                clearance: clearance,
                supportCount: nearestCluster.count,
                age: age,
                stoppingDistance: stoppingDistance,
                motion: motion
            )
        }

        guard sweptVolumeCoverage(
            snapshot: snapshot,
            command: command,
            motion: motion,
            horizon: cautionDistance
        ) >= minimumSweptVolumeCoverage else {
            return .unavailable(.blindSweptVolume, sampleAge: age, motionClass: motion)
        }
        return clearObservation(age: age, motion: motion, stoppingDistance: stoppingDistance)
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
                                           forward: Double,
                                           command: WheelCommand,
                                           geometry: RoverCollisionGeometry,
                                           horizon: Double) -> Double? {
        let halfWidth = geometry.width / 2 + geometry.lateralSafetyMargin + cellResolution / 2
        let halfLength = geometry.length / 2 + cellResolution / 2
        let motion = DepthSafetyMotionClass.classify(command)
        if motion == .rotating {
            return hypot(lateral, forward) <= hypot(halfLength, halfWidth) ? 0 : nil
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
            if abs(localLateral) <= halfWidth, abs(localForward) <= halfLength {
                return travel
            }
        }
        return nil
    }

    private static func sweptVolumeCoverage(snapshot: DepthSafetySnapshot,
                                            command: WheelCommand,
                                            motion: DepthSafetyMotionClass,
                                            horizon: Double) -> Double {
        guard let projection = snapshot.projection else { return 0 }
        let points = coveragePoints(
            command: command,
            motion: motion,
            geometry: snapshot.geometry,
            horizon: horizon
        )
        guard !points.isEmpty else { return 0 }
        let covered = points.reduce(into: 0) { result, point in
            if hasDepthSupport(at: point, projection: projection) { result += 1 }
        }
        return Double(covered) / Double(points.count)
    }

    private static func coveragePoints(command: WheelCommand,
                                       motion: DepthSafetyMotionClass,
                                       geometry: RoverCollisionGeometry,
                                       horizon: Double) -> [SIMD3<Double>] {
        let halfWidth = geometry.width / 2 + geometry.lateralSafetyMargin
        let halfLength = geometry.length / 2
        let heights = [
            geometry.minimumCollisionHeight + cellResolution / 2,
            (geometry.minimumCollisionHeight + geometry.maximumCollisionHeight) / 2,
            geometry.maximumCollisionHeight - cellResolution / 2,
        ]
        if motion == .rotating {
            let radius = hypot(halfLength, halfWidth)
            return (0..<36).flatMap { index -> [SIMD3<Double>] in
                let angle = 2 * Double.pi * Double(index) / 36
                return heights.map { height in
                    SIMD3(radius * sin(angle), height, radius * cos(angle))
                }
            }
        }

        let average = (command.left + command.right) / 2
        guard average > 0 else { return [] }
        let delta = command.right - command.left
        let curvature = abs(delta) < 0.001 ? 0 : delta / (RoverConfig.wheelBase * average)
        let steps = max(1, Int(ceil(horizon / cellResolution)))
        var result: [SIMD3<Double>] = []
        for index in 0...steps {
            let travel = horizon * Double(index) / Double(steps)
            let pose = pose(at: travel, curvature: curvature)
            let boundary: [(Double, Double)] = [
                (-halfWidth, halfLength),
                (0, halfLength),
                (halfWidth, halfLength),
            ]
            // For forward arcs, validate the leading edge that the forward-facing
            // depth camera can actually observe. Occupied cells still use the full
            // rover footprint in firstContactTravel.
            for (localLateral, localForward) in boundary {
                let lateral = pose.x
                    + localLateral * cos(pose.heading)
                    - localForward * sin(pose.heading)
                let forward = pose.z
                    + localLateral * sin(pose.heading)
                    + localForward * cos(pose.heading)
                for height in heights {
                    result.append(SIMD3(lateral, height, forward))
                }
            }
        }
        return result
    }

    private static func hasDepthSupport(at roverPoint: SIMD3<Double>,
                                        projection: DepthSafetyProjection) -> Bool {
        let world = projection.roverOrigin
            + projection.roverRight * roverPoint.x
            + SIMD3<Double>(0, roverPoint.y, 0)
            + projection.roverForward * roverPoint.z
        let camera = projection.cameraFromWorld * SIMD4<Float>(
            Float(world.x), Float(world.y), Float(world.z), 1
        )
        let targetDepth = -Double(camera.z)
        guard targetDepth > 0.05, targetDepth.isFinite else { return false }
        let pixelX = Int((projection.fx * Double(camera.x) / targetDepth + projection.cx).rounded())
        let pixelY = Int((projection.cy - projection.fy * Double(camera.y) / targetDepth).rounded())
        guard pixelX >= 0, pixelX < projection.width,
              pixelY >= 0, pixelY < projection.height else { return false }
        let measured = Double(projection.depthValues[pixelY * projection.width + pixelX])
        return measured > 0.05
            && measured.isFinite
            && measured + cellResolution >= targetDepth
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
