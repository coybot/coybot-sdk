import Foundation

/// A 2D occupancy grid ("costmap") over the ground plane.
///
/// Cost values: 0 = free … `Costmap.lethal` (255) = occupied. Values in between
/// come from obstacle *inflation* (a clearance buffer sized to the robot radius),
/// so the planner keeps the rover from clipping walls/furniture.
///
/// On device, the RoverOperator app rebuilds this each planning cycle from the
/// ARKit scene mesh + LiDAR depth (see CostmapBuilder in the iOS target).
public struct Costmap: Sendable {
    public static let lethal: UInt8 = 255
    /// Cells at or above this cost are treated as blocked by the planner.
    public static let lethalThreshold: UInt8 = 253

    public let width: Int          // cells
    public let height: Int         // cells
    public let resolution: Double  // meters per cell
    public let origin: Vec2        // world coord of the (0,0) cell's corner

    public private(set) var cells: [UInt8]

    public init(width: Int, height: Int, resolution: Double, origin: Vec2 = .zero) {
        precondition(width > 0 && height > 0 && resolution > 0)
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.cells = [UInt8](repeating: 0, count: width * height)
    }

    // MARK: - Indexing

    @inline(__always) public func inBounds(_ cx: Int, _ cy: Int) -> Bool {
        cx >= 0 && cy >= 0 && cx < width && cy < height
    }

    @inline(__always) func index(_ cx: Int, _ cy: Int) -> Int { cy * width + cx }

    public func cost(_ cx: Int, _ cy: Int) -> UInt8 {
        guard inBounds(cx, cy) else { return Costmap.lethal }  // out of bounds = blocked
        return cells[index(cx, cy)]
    }

    public func isBlocked(_ cx: Int, _ cy: Int) -> Bool {
        cost(cx, cy) >= Costmap.lethalThreshold
    }

    /// Conservative straight-segment clearance against occupied and inflated cells.
    /// Include every touched cell (including boundaries), rather than sampling centers.
    public func isSegmentClear(from start: Vec2, to end: Vec2, margin: Double) -> Bool {
        guard start.x.isFinite, start.y.isFinite, end.x.isFinite, end.y.isFinite,
              margin.isFinite, margin >= 0 else { return false }
        let low = Vec2(min(start.x, end.x) - margin, min(start.y, end.y) - margin)
        let high = Vec2(max(start.x, end.x) + margin, max(start.y, end.y) + margin)
        guard low.x > origin.x, low.y > origin.y,
              high.x < origin.x + Double(width) * resolution,
              high.y < origin.y + Double(height) * resolution else { return false }
        let first = worldToCell(low)
        let last = worldToCell(high)
        let delta = end - start
        for cy in max(0, first.cy - 1)...min(height - 1, last.cy + 1) {
            for cx in max(0, first.cx - 1)...min(width - 1, last.cx + 1) where cost(cx, cy) > 0 {
                let x = origin.x + Double(cx) * resolution
                let y = origin.y + Double(cy) * resolution
                var entry = 0.0
                var exit = 1.0
                func intersectsAxis(_ position: Double, _ direction: Double, _ min: Double, _ max: Double) -> Bool {
                    if abs(direction) < 1e-12 { return position >= min && position <= max }
                    let a = (min - position) / direction
                    let b = (max - position) / direction
                    entry = Swift.max(entry, Swift.min(a, b))
                    exit = Swift.min(exit, Swift.max(a, b))
                    return entry <= exit
                }
                if intersectsAxis(start.x, delta.x, x - margin, x + resolution + margin),
                   intersectsAxis(start.y, delta.y, y - margin, y + resolution + margin) { return false }
            }
        }
        return true
    }

    // MARK: - World <-> cell

    public func worldToCell(_ p: Vec2) -> (cx: Int, cy: Int) {
        let cx = Int(((p.x - origin.x) / resolution).rounded(.down))
        let cy = Int(((p.y - origin.y) / resolution).rounded(.down))
        return (cx, cy)
    }

    /// World coordinate of a cell center.
    public func cellCenter(_ cx: Int, _ cy: Int) -> Vec2 {
        Vec2(origin.x + (Double(cx) + 0.5) * resolution,
             origin.y + (Double(cy) + 0.5) * resolution)
    }

    // MARK: - Mutation

    public mutating func setCost(_ cx: Int, _ cy: Int, _ value: UInt8) {
        guard inBounds(cx, cy) else { return }
        cells[index(cx, cy)] = value
    }

    /// Mark the cell containing a world point as a lethal obstacle.
    public mutating func markObstacle(at p: Vec2) {
        let c = worldToCell(p)
        setCost(c.cx, c.cy, Costmap.lethal)
    }

    /// Grow obstacles by `radius` meters, writing a decreasing cost gradient so the
    /// planner prefers to stay clear. Cells within `radius` of any lethal cell become
    /// lethal (hard clearance); this is the standard "inflation" step. Call once after
    /// all obstacles are marked.
    public mutating func inflate(radius: Double) {
        guard radius > 0 else { return }
        let r = Int((radius / resolution).rounded(.up))
        guard r > 0 else { return }

        // Snapshot the lethal seed cells.
        var seeds: [(Int, Int)] = []
        for cy in 0..<height {
            for cx in 0..<width where cells[index(cx, cy)] == Costmap.lethal {
                seeds.append((cx, cy))
            }
        }

        let r2 = Double(r * r)
        for (sx, sy) in seeds {
            for dy in -r...r {
                for dx in -r...r {
                    let d2 = Double(dx * dx + dy * dy)
                    if d2 > r2 { continue }
                    let nx = sx + dx, ny = sy + dy
                    guard inBounds(nx, ny) else { continue }
                    let i = index(nx, ny)
                    if cells[i] == Costmap.lethal { continue }
                    // Linear falloff: closer to the obstacle => higher cost, capped just
                    // below lethalThreshold so inflated cells stay traversable-but-costly
                    // except the innermost ring which we make hard-blocked.
                    let dist = d2.squareRoot()
                    let value: UInt8 = dist <= 1.0
                        ? Costmap.lethal
                        : UInt8(max(1, min(252, Int((1.0 - dist / Double(r)) * 252))))
                    if value > cells[i] { cells[i] = value }
                }
            }
        }
    }
}
