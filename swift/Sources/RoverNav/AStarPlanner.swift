import Foundation

public enum PathPlanningFailure: String, Equatable, Sendable {
    case startOutsideMap = "start_outside_map"
    case goalOutsideMap = "goal_outside_map"
    case startBlocked = "start_blocked"
    case goalBlocked = "goal_blocked"
    case noConnectedPath = "no_connected_path"
}

public enum PathPlanningResult: Equatable, Sendable {
    case path([Vec2])
    case rejected(PathPlanningFailure)
}

/// 8-connected A* over a `Costmap`. Returns world-coordinate waypoints (cell
/// centers) from start to goal, or nil if unreachable. Inflated cell cost is added
/// as a soft penalty so paths keep clearance from walls when possible.
public struct AStarPlanner: Sendable {
    public init() {}

    private struct Cell: Hashable { let x: Int; let y: Int }

    // Min-heap keyed on f-score.
    private struct Frontier {
        private var heap: [(f: Double, cell: Cell)] = []
        var isEmpty: Bool { heap.isEmpty }

        mutating func push(_ cell: Cell, f: Double) {
            heap.append((f, cell))
            var i = heap.count - 1
            while i > 0 {
                let p = (i - 1) / 2
                if heap[p].f <= heap[i].f { break }
                heap.swapAt(p, i); i = p
            }
        }

        mutating func pop() -> Cell? {
            guard !heap.isEmpty else { return nil }
            let top = heap[0]
            let last = heap.removeLast()
            if !heap.isEmpty {
                heap[0] = last
                var i = 0
                let n = heap.count
                while true {
                    let l = 2 * i + 1, r = 2 * i + 2
                    var s = i
                    if l < n && heap[l].f < heap[s].f { s = l }
                    if r < n && heap[r].f < heap[s].f { s = r }
                    if s == i { break }
                    heap.swapAt(s, i); i = s
                }
            }
            return top.cell
        }
    }

    /// Plan from `start` world pose to `goal` world point.
    public func plan(from start: Vec2, to goal: Vec2, in map: Costmap) -> [Vec2]? {
        guard case .path(let path) = assess(from: start, to: goal, in: map) else {
            return nil
        }
        return path
    }

    public func assess(from start: Vec2, to goal: Vec2, in map: Costmap) -> PathPlanningResult {
        let s = map.worldToCell(start)
        let g = map.worldToCell(goal)
        let startCell = Cell(x: s.cx, y: s.cy)
        let goalCell = Cell(x: g.cx, y: g.cy)

        guard map.inBounds(startCell.x, startCell.y) else {
            return .rejected(.startOutsideMap)
        }
        guard map.inBounds(goalCell.x, goalCell.y) else {
            return .rejected(.goalOutsideMap)
        }
        guard !map.isBlocked(startCell.x, startCell.y) else {
            return .rejected(.startBlocked)
        }
        guard !map.isBlocked(goalCell.x, goalCell.y) else {
            return .rejected(.goalBlocked)
        }

        func heuristic(_ c: Cell) -> Double {
            // Octile distance (admissible for 8-connectivity).
            let dx = Double(abs(c.x - goalCell.x))
            let dy = Double(abs(c.y - goalCell.y))
            return (dx + dy) + (1.41421356 - 2) * min(dx, dy)
        }

        var gScore: [Cell: Double] = [startCell: 0]
        var cameFrom: [Cell: Cell] = [:]
        var frontier = Frontier()
        frontier.push(startCell, f: heuristic(startCell))
        var closed = Set<Cell>()

        let neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                         (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while let current = frontier.pop() {
            if current == goalCell {
                return .path(reconstruct(cameFrom, current, map))
            }
            if !closed.insert(current).inserted { continue }
            let cg = gScore[current] ?? .infinity

            for (dx, dy) in neighbors {
                let nx = current.x + dx, ny = current.y + dy
                guard map.inBounds(nx, ny), !map.isBlocked(nx, ny) else { continue }
                // Prevent cutting diagonally through a blocked corner.
                if dx != 0 && dy != 0 {
                    if map.isBlocked(current.x + dx, current.y) ||
                       map.isBlocked(current.x, current.y + dy) { continue }
                }
                let n = Cell(x: nx, y: ny)
                if closed.contains(n) { continue }
                let step = (dx != 0 && dy != 0) ? 1.41421356 : 1.0
                // Add inflated-cost penalty (normalized) to favor clearance.
                let penalty = Double(map.cost(nx, ny)) / 255.0 * 3.0
                let tentative = cg + step + penalty
                if tentative < (gScore[n] ?? .infinity) {
                    gScore[n] = tentative
                    cameFrom[n] = current
                    frontier.push(n, f: tentative + heuristic(n))
                }
            }
        }
        return .rejected(.noConnectedPath)
    }

    private func reconstruct(_ cameFrom: [Cell: Cell], _ end: Cell, _ map: Costmap) -> [Vec2] {
        var path = [end]
        var cur = end
        while let prev = cameFrom[cur] {
            path.append(prev)
            cur = prev
        }
        return path.reversed().map { map.cellCenter($0.x, $0.y) }
    }
}
