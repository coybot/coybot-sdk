import Foundation

/// Which ground cells have actually been *seen* (any scene-mesh evidence, including bare
/// floor), as opposed to `Costmap`'s free-vs-blocked. The two grids share geometry; the
/// distinction matters because a `Costmap` cell defaults to free, so unmapped space beyond
/// a doorway would otherwise be indistinguishable from open floor the rover has already
/// looked at.
public struct ObservedGrid: Sendable {
    public let width: Int
    public let height: Int
    public let resolution: Double
    public let origin: Vec2

    public private(set) var cells: [Bool]

    public init(width: Int, height: Int, resolution: Double, origin: Vec2 = .zero) {
        precondition(width > 0 && height > 0 && resolution > 0)
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.cells = [Bool](repeating: false, count: width * height)
    }

    /// Same geometry as a costmap, so the two can be built side by side.
    public init(matching map: Costmap) {
        self.init(width: map.width, height: map.height, resolution: map.resolution, origin: map.origin)
    }

    @inline(__always) public func inBounds(_ cx: Int, _ cy: Int) -> Bool {
        cx >= 0 && cy >= 0 && cx < width && cy < height
    }

    public func isObserved(_ cx: Int, _ cy: Int) -> Bool {
        guard inBounds(cx, cy) else { return false }
        return cells[cy * width + cx]
    }

    public mutating func markObserved(at p: Vec2) {
        let cx = Int(((p.x - origin.x) / resolution).rounded(.down))
        let cy = Int(((p.y - origin.y) / resolution).rounded(.down))
        guard inBounds(cx, cy) else { return }
        cells[cy * width + cx] = true
    }
}

/// A cluster of frontier cells — the boundary between space the rover has seen and space
/// it hasn't. In practice these land on doorways, hall mouths, and the edge of the mapped
/// area: exactly the "places you could go look" a mission brain needs when the goal isn't
/// in view. This is a *candidate generator only* — choosing which frontier to chase (or
/// whether to chase any) is the brain's judgment, not an algorithm here.
public struct Frontier: Equatable, Sendable {
    public let centroid: Vec2
    /// Rough physical extent of the cluster (m) — lets a brain prefer a door-sized gap
    /// over a sliver of unmapped floor at the map's edge.
    public let widthMeters: Double
    public let cellCount: Int
    public let outwardDirection: Vec2?

    public init(centroid: Vec2,
                widthMeters: Double,
                cellCount: Int,
                outwardDirection: Vec2? = nil) {
        self.centroid = centroid
        self.widthMeters = widthMeters
        self.cellCount = cellCount
        self.outwardDirection = outwardDirection
    }
}

public enum FrontierFinder {
    /// Frontier cell = observed ∧ free ∧ 4-adjacent to at least one unobserved cell.
    /// Clusters (8-connected) below `minCells` are dropped as noise.
    public static func candidates(costmap: Costmap,
                                  observed: ObservedGrid,
                                  minCells: Int = 3) -> [Frontier] {
        precondition(costmap.width == observed.width && costmap.height == observed.height,
                     "costmap and observed grid must share geometry")
        let w = costmap.width, h = costmap.height

        var isFrontier = [Bool](repeating: false, count: w * h)
        for cy in 0..<h {
            for cx in 0..<w {
                guard observed.isObserved(cx, cy), !costmap.isBlocked(cx, cy) else { continue }
                let touchesUnknown =
                    (cx > 0     && !observed.isObserved(cx - 1, cy)) ||
                    (cx < w - 1 && !observed.isObserved(cx + 1, cy)) ||
                    (cy > 0     && !observed.isObserved(cx, cy - 1)) ||
                    (cy < h - 1 && !observed.isObserved(cx, cy + 1))
                if touchesUnknown { isFrontier[cy * w + cx] = true }
            }
        }

        // Flood-fill 8-connected clusters.
        var visited = [Bool](repeating: false, count: w * h)
        var result: [Frontier] = []
        for cy in 0..<h {
            for cx in 0..<w where isFrontier[cy * w + cx] && !visited[cy * w + cx] {
                var stack = [(cx, cy)]
                visited[cy * w + cx] = true
                var members: [(Int, Int)] = []
                while let (x, y) = stack.popLast() {
                    members.append((x, y))
                    for dy in -1...1 {
                        for dx in -1...1 where dx != 0 || dy != 0 {
                            let nx = x + dx, ny = y + dy
                            guard nx >= 0, ny >= 0, nx < w, ny < h else { continue }
                            let i = ny * w + nx
                            if isFrontier[i] && !visited[i] {
                                visited[i] = true
                                stack.append((nx, ny))
                            }
                        }
                    }
                }
                guard members.count >= minCells else { continue }

                var minX = Int.max, maxX = Int.min, minY = Int.max, maxY = Int.min
                var sum = Vec2.zero
                var outward = Vec2.zero
                for (x, y) in members {
                    sum = sum + costmap.cellCenter(x, y)
                    minX = min(minX, x); maxX = max(maxX, x)
                    minY = min(minY, y); maxY = max(maxY, y)
                    if x > 0, !observed.isObserved(x - 1, y) { outward.x -= 1 }
                    if x < w - 1, !observed.isObserved(x + 1, y) { outward.x += 1 }
                    if y > 0, !observed.isObserved(x, y - 1) { outward.y -= 1 }
                    if y < h - 1, !observed.isObserved(x, y + 1) { outward.y += 1 }
                }
                let extent = Double(max(maxX - minX, maxY - minY) + 1) * costmap.resolution
                let outwardDirection: Vec2? = outward.length == 0
                    ? nil
                    : outward * (1.0 / outward.length)
                result.append(Frontier(centroid: sum * (1.0 / Double(members.count)),
                                       widthMeters: extent,
                                       cellCount: members.count,
                                       outwardDirection: outwardDirection))
            }
        }
        // Big openings first — a stable, meaningful order for prompt rendering.
        return result.sorted { $0.cellCount > $1.cellCount }
    }
}
