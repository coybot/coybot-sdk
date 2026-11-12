import Foundation
import ARKit
import RoverNav

/// Turns the ARKit LiDAR scene mesh into a 2D `Costmap` for the RoverNav planner.
/// Vertices in an "obstacle height band" above the floor (i.e. things the rover would
/// hit — walls, furniture legs, people) are projected down onto the ground grid.
public enum CostmapBuilder {
    /// Value geometry copied before crossing actors; ARKit/Metal buffers stay on
    /// the session actor and cannot change while the worker builds the grid.
    struct MeshSnapshot: Sendable {
        let worldVertices: [SIMD3<Float>]
        var expectedFloorY: Float? = nil
        var classifiedFloorY: Float? = nil
    }

    /// Owns copied bytes, never ARKit or Metal storage. Coordinate transformation
    /// can then run off the session actor without racing ARKit buffer updates.
    struct LocalMeshSnapshot: Sendable {
        let bytes: Data
        let stride: Int
        let count: Int
        let transform: simd_float4x4

        func worldVertices() -> [SIMD3<Float>] {
            guard stride >= 12, count > 0, bytes.count >= 12,
                  count - 1 <= (bytes.count - 12) / stride else { return [] }
            return bytes.withUnsafeBytes { raw in
                (0..<count).map { index in
                    let offset = index * stride
                    let x = raw.loadUnaligned(fromByteOffset: offset, as: Float.self)
                    let y = raw.loadUnaligned(fromByteOffset: offset + 4, as: Float.self)
                    let z = raw.loadUnaligned(fromByteOffset: offset + 8, as: Float.self)
                    let world = transform * SIMD4<Float>(x, y, z, 1)
                    return SIMD3(world.x, world.y, world.z)
                }
            }
        }
    }

    static func captureMesh(from meshAnchors: [ARMeshAnchor]) -> MeshSnapshot {
        var worldVertices: [SIMD3<Float>] = []
        worldVertices.reserveCapacity(meshAnchors.reduce(0) { $0 + $1.geometry.vertices.count })
        for anchor in meshAnchors {
            let vertices = anchor.geometry.vertices
            let buffer = vertices.buffer.contents()
            for index in 0..<vertices.count {
                let point = buffer.advanced(by: vertices.offset + vertices.stride * index)
                    .assumingMemoryBound(to: (Float, Float, Float).self).pointee
                let world = anchor.transform * SIMD4<Float>(point.0, point.1, point.2, 1)
                worldVertices.append(SIMD3(world.x, world.y, world.z))
            }
        }
        return .init(worldVertices: worldVertices)
    }

    @MainActor
    static func buildAsync(from meshAnchors: [ARMeshAnchor], center: Vec2,
                           params: Params = Params()) async -> Costmap? {
        let captureStarted = ProcessInfo.processInfo.systemUptime
        let snapshot = captureMesh(from: meshAnchors)
        let captureFinished = ProcessInfo.processInfo.systemUptime
        guard !snapshot.worldVertices.isEmpty else {
            RuntimeFileLog.append("nav_ready_map_snapshot", fields: ["availability": "no_mesh"])
            return nil
        }
        let result = await Task.detached(priority: .userInitiated) {
            buildWithObserved(from: snapshot, center: center, params: params)
        }.value
        var diagnostics = result.diagnostics
        diagnostics["mesh_capture_s"] = String(captureFinished - captureStarted)
        diagnostics["map_worker_and_delivery_s"] = String(ProcessInfo.processInfo.systemUptime - captureFinished)
        RuntimeFileLog.append("nav_ready_map_snapshot", fields: diagnostics)
        return result.map
    }

    @MainActor
    static func planFollowAsync(from anchors: [ARMeshAnchor], start: Vec2, goal: Vec2,
                                expectedFloorY: Float? = nil, classifiedFloorY: Float? = nil) async -> FollowPursuitPlan? {
        let began = ProcessInfo.processInfo.systemUptime
        var local: [LocalMeshSnapshot] = []
        for anchor in anchors {
            let source = anchor.geometry.vertices
            guard source.stride >= 12, source.count >= 0, source.offset >= 0,
                  source.offset <= source.buffer.length else {
                RuntimeFileLog.append("nav_follow_plan_rejected", fields: ["reason": "invalid_mesh_buffer"])
                return nil
            }
            let available = source.buffer.length - source.offset
            guard source.count == 0 || (available >= 12 && source.count - 1 <= (available - 12) / source.stride) else {
                RuntimeFileLog.append("nav_follow_plan_rejected", fields: ["reason": "invalid_mesh_buffer"])
                return nil
            }
            let byteCount = source.count == 0 ? 0 : (source.count - 1) * source.stride + 12
            local.append(LocalMeshSnapshot(bytes: Data(bytes: source.buffer.contents().advanced(by: source.offset),
                count: byteCount), stride: source.stride, count: source.count, transform: anchor.transform))
        }
        let capturedLocal = local
        let captured = ProcessInfo.processInfo.systemUptime
        let worker = Task.detached(priority: .utility) {
            guard !Task.isCancelled else { return nil as FollowPursuitPlan? }
            let snapshot = MeshSnapshot(worldVertices: capturedLocal.flatMap { $0.worldVertices() },
                expectedFloorY: expectedFloorY, classifiedFloorY: classifiedFloorY)
            guard !Task.isCancelled else { return nil }
            return computeFollowPlan(snapshot, start: start, goal: goal)
        }
        let result = await withTaskCancellationHandler { await worker.value } onCancel: { worker.cancel() }
        RuntimeFileLog.append("nav_follow_plan_snapshot", fields: [
            "mesh_capture_s": String(captured - began),
            "worker_and_delivery_s": String(ProcessInfo.processInfo.systemUptime - captured),
            "vertex_count": String(local.reduce(0) { $0 + $1.count }),
            "capture_mode": "copied_local_bytes_worker_transform",
            "outcome": result == nil || result?.rejection != nil ? "no_path" : "planned"])
        return result
    }

    static func planCaptured(_ snapshot: MeshSnapshot, start: Vec2, goal: Vec2,
                              compute: @escaping @Sendable (MeshSnapshot, Vec2, Vec2) -> FollowPursuitPlan? = { mesh, start, goal in
        computeFollowPlan(mesh, start: start, goal: goal)
    }) async -> FollowPursuitPlan? {
        let worker = Task.detached(priority: .utility) { () -> FollowPursuitPlan? in
            guard !Task.isCancelled else { return nil }
            let result = compute(snapshot, start, goal)
            return Task.isCancelled ? nil : result
        }
        return await withTaskCancellationHandler { await worker.value } onCancel: { worker.cancel() }
    }

    private static func computeFollowPlan(_ mesh: MeshSnapshot, start: Vec2, goal: Vec2) -> FollowPursuitPlan? {
        let reason = "no_mesh"
        if !mesh.worldVertices.isEmpty {
            guard mesh.worldVertices.allSatisfy({ $0.x.isFinite && $0.y.isFinite && $0.z.isFinite }) else {
                RuntimeFileLog.append("nav_follow_plan_rejected", fields: ["reason": "invalid_mesh_vertex"])
                return nil
            }
            let built = buildWithObserved(from: mesh, center: start)
            let map = built.map
            switch AStarPlanner().assess(from: start, to: goal, in: map) {
            case .path(let path): return .init(path: path, map: map)
            case .rejected(let failure):
                var diagnostics = built.diagnostics
                diagnostics["reason"] = failure.rawValue
                diagnostics["start_x"] = String(start.x); diagnostics["start_z"] = String(start.y)
                diagnostics["goal_x"] = String(goal.x); diagnostics["goal_z"] = String(goal.y)
                let goalCell = map.worldToCell(goal)
                diagnostics["goal_cell_cost"] = String(map.cost(goalCell.cx, goalCell.cy))
                RuntimeFileLog.append("nav_follow_plan_rejected", fields: diagnostics)
                return .init(path: [], map: map, rejection: failure, diagnostics: diagnostics)
            }
        }
        RuntimeFileLog.append("nav_follow_plan_rejected", fields: ["reason": reason,
            "start_x": String(start.x), "start_z": String(start.y), "goal_x": String(goal.x), "goal_z": String(goal.y)])
        return nil
    }

    public struct Params: Sendable {
        public var resolution: Double        // m/cell
        public var size: Double              // m — square window centered on the robot
        public var floorBand: Double         // ignore anything below this above the floor
        public var ceilingBand: Double       // ignore anything above this (ceilings/overheads)
        public var inflationRadius: Double   // robot radius + safety margin

        public init(resolution: Double = 0.10,
                    size: Double = 12.0,
                    floorBand: Double = 0.10,
                    ceilingBand: Double = 1.8,
                    inflationRadius: Double = 0.25) {
            self.resolution = resolution
            self.size = size
            self.floorBand = floorBand
            self.ceilingBand = ceilingBand
            self.inflationRadius = inflationRadius
        }
    }

    static func estimatedFloorHeight(fromWorldHeights heights: [Float]) -> Float {
        let finite = heights.filter(\.isFinite).sorted()
        guard !finite.isEmpty else { return 0 }
        let index = min(finite.count - 1, Int(Double(finite.count - 1) * 0.05))
        return finite[index]
    }

    static func isObstacleHeight(_ worldY: Float,
                                 floorY: Float,
                                 floorBand: Double = 0.10,
                                 ceilingBand: Double = 1.8) -> Bool {
        let height = worldY - floorY
        return height > Float(floorBand) && height < Float(ceilingBand)
    }

    /// Build a costmap centered on `center` (nav-plane coords) from the mesh anchors.
    public static func build(from meshAnchors: [ARMeshAnchor],
                             center: Vec2,
                             params: Params = Params()) -> Costmap {
        buildWithObserved(from: meshAnchors, center: center, params: params).map
    }

    /// Same as `build`, but also reports which cells have any mesh evidence at all —
    /// including the bare-floor vertices the obstacle pass filters out. That "observed"
    /// signal is what distinguishes open floor the rover has actually seen from unmapped
    /// space beyond a doorway (`Costmap` alone can't: both read as free), and is the input
    /// `FrontierFinder` needs to propose exploration candidates.
    public static func buildWithObserved(from meshAnchors: [ARMeshAnchor],
                                         center: Vec2,
                                         params: Params = Params()) -> (map: Costmap, observed: ObservedGrid) {
        let result = buildWithObserved(from: captureMesh(from: meshAnchors), center: center, params: params)
        return (result.map, result.observed)
    }

    static func buildWithObserved(from snapshot: MeshSnapshot,
                                  center: Vec2,
                                  params: Params = Params()) -> (map: Costmap, observed: ObservedGrid, diagnostics: [String: String]) {
        let cells = Int((params.size / params.resolution).rounded())
        let origin = Vec2(center.x - params.size / 2, center.y - params.size / 2)
        var map = Costmap(width: cells, height: cells, resolution: params.resolution, origin: origin)
        var observed = ObservedGrid(matching: map)

        // Estimate floor height from transformed mesh geometry (ARKit Y is up).
        let worldHeights = snapshot.worldVertices.map(\.y)
        let floorY = estimatedFloorHeight(fromWorldHeights: worldHeights)
        let localFloor = LocalGroundPlane.assess(vertices: snapshot.worldVertices, center: center,
            expectedFloorY: snapshot.expectedFloorY, classifiedFloorY: snapshot.classifiedFloorY)
        var localDropVertices = 0

        var nearbyObstacleCount = 0
        var nearbyObstacleSamples: [String] = []
        var sampledCells: Set<Int> = []
        let startCell = map.worldToCell(center)
        var blockingVertices = 0
        var blockingSamples: [String] = []
        var blockingSampledCells: Set<Int> = []
        var localHeights: [Float] = []
        for world in snapshot.worldVertices {
            let groundPoint = Vec2(Double(world.x), Double(world.z))
            if groundPoint.distance(to: center) <= 0.5 { localHeights.append(world.y) }
            observed.markObserved(at: groundPoint)
            let supportedHeight = localFloor.plane?.height(at: groundPoint)
            let classificationFloor = supportedHeight ?? floorY
            let drop = supportedHeight != nil && world.y < classificationFloor - 0.05
            if drop { localDropVertices += 1 }
            guard drop || isObstacleHeight(
                world.y,
                floorY: classificationFloor,
                floorBand: params.floorBand,
                ceilingBand: params.ceilingBand
            ) else { continue }
            let seed = map.worldToCell(groundPoint)
            map.setCost(seed.cx, seed.cy, Costmap.lethal)
            let dx = seed.cx - startCell.cx, dz = seed.cy - startCell.cy
            // Costmap inflation makes the immediate axial-neighbor ring lethal;
            // further rings add soft cost. Keep this evidence aligned with that rule.
            if (dx == 0 && dz == 0) || (params.inflationRadius > 0 && abs(dx) + abs(dz) <= 1) {
                blockingVertices += 1
                if blockingSamples.count < 16, blockingSampledCells.insert(seed.cy * cells + seed.cx).inserted {
                    blockingSamples.append(String(format: "%.6f,%.6f,%.6f", Double(world.x), Double(world.y), Double(world.z)))
                }
            }
            if groundPoint.distance(to: center) <= 0.5 {
                nearbyObstacleCount += 1
                let cell = seed
                if nearbyObstacleSamples.count < 16, sampledCells.insert(cell.cy * cells + cell.cx).inserted {
                    nearbyObstacleSamples.append(String(format: "%.3f,%.3f,%.3f",
                        groundPoint.x - center.x, Double(world.y - classificationFloor), groundPoint.y - center.y))
                }
            }
        }

        let rawCost = map.cost(startCell.cx, startCell.cy)
        map.inflate(radius: params.inflationRadius)
        var diagnostics: [String: String] = [
            "availability": "mesh", "vertex_count": String(snapshot.worldVertices.count),
            "estimated_floor_y_m": String(localFloor.plane?.height(at: center) ?? floorY),
            "global_floor_y_m": String(floorY), "local_drop_vertex_count": String(localDropVertices),
            "local_floor_y_p05_m": localHeights.isEmpty ? "unavailable" : String(estimatedFloorHeight(fromWorldHeights: localHeights)),
            "local_height_sample_count": String(localHeights.count),
            "map_resolution_m": String(params.resolution), "inflation_radius_m": String(params.inflationRadius),
            "floor_band_m": String(params.floorBand), "ceiling_band_m": String(params.ceilingBand),
            "start_cell_x": String(startCell.cx), "start_cell_z": String(startCell.cy),
            "start_blocking_vertex_count": String(blockingVertices),
            "start_blocking_samples_world_xyz_m": blockingSamples.joined(separator: ";"),
            "raw_start_cost": String(rawCost), "inflated_start_cost": String(map.cost(startCell.cx, startCell.cy)),
            "nearby_obstacle_vertex_count": String(nearbyObstacleCount),
            "local_obstacle_samples_dx_height_dz_m": nearbyObstacleSamples.joined(separator: ";")
        ]
        diagnostics.merge(localFloor.diagnostics) { _, new in new }
        return (map, observed, diagnostics)
    }
}
