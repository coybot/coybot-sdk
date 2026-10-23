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
        let snapshot = captureMesh(from: meshAnchors)
        guard !snapshot.worldVertices.isEmpty else {
            RuntimeFileLog.append("nav_ready_map_snapshot", fields: ["availability": "no_mesh"])
            return nil
        }
        let result = await Task.detached(priority: .userInitiated) {
            buildWithObserved(from: snapshot, center: center, params: params)
        }.value
        RuntimeFileLog.append("nav_ready_map_snapshot", fields: result.diagnostics)
        return result.map
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

        var nearbyObstacleCount = 0
        var nearbyObstacleSamples: [String] = []
        var sampledCells: Set<Int> = []
        for world in snapshot.worldVertices {
            let groundPoint = Vec2(Double(world.x), Double(world.z))
            observed.markObserved(at: groundPoint)
            guard isObstacleHeight(
                world.y,
                floorY: floorY,
                floorBand: params.floorBand,
                ceilingBand: params.ceilingBand
            ) else { continue }
            map.markObstacle(at: groundPoint)
            if groundPoint.distance(to: center) <= 0.5 {
                nearbyObstacleCount += 1
                let cell = map.worldToCell(groundPoint)
                if nearbyObstacleSamples.count < 16, sampledCells.insert(cell.cy * cells + cell.cx).inserted {
                    nearbyObstacleSamples.append(String(format: "%.3f,%.3f,%.3f",
                        groundPoint.x - center.x, Double(world.y - floorY), groundPoint.y - center.y))
                }
            }
        }

        let startCell = map.worldToCell(center)
        let rawCost = map.cost(startCell.cx, startCell.cy)
        map.inflate(radius: params.inflationRadius)
        let diagnostics = [
            "availability": "mesh", "vertex_count": String(snapshot.worldVertices.count),
            "estimated_floor_y_m": String(floorY),
            "raw_start_cost": String(rawCost), "inflated_start_cost": String(map.cost(startCell.cx, startCell.cy)),
            "nearby_obstacle_vertex_count": String(nearbyObstacleCount),
            "local_obstacle_samples_dx_height_dz_m": nearbyObstacleSamples.joined(separator: ";")
        ]
        return (map, observed, diagnostics)
    }
}
