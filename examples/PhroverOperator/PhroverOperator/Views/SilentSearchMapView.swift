import CoreGraphics
import PhroverKit
import SwiftUI

struct SilentSearchMapTransform: Equatable {
    let missionCenter: CGPoint
    let canvasCenter: CGPoint
    let scale: CGFloat

    static func fit(points: [CGPoint], in size: CGSize, padding: CGFloat) -> Self {
        let finite = points.filter { $0.x.isFinite && $0.y.isFinite }
        guard let first = finite.first else {
            return Self(missionCenter: .zero,
                        canvasCenter: CGPoint(x: size.width / 2, y: size.height / 2), scale: 1)
        }
        let minX = finite.dropFirst().reduce(first.x) { min($0, $1.x) }
        let maxX = finite.dropFirst().reduce(first.x) { max($0, $1.x) }
        let minY = finite.dropFirst().reduce(first.y) { min($0, $1.y) }
        let maxY = finite.dropFirst().reduce(first.y) { max($0, $1.y) }
        let usableWidth = max(1, size.width - padding * 2)
        let usableHeight = max(1, size.height - padding * 2)
        let width = maxX - minX
        let height = maxY - minY
        let xScale = width > 0 ? usableWidth / width : .greatestFiniteMagnitude
        let yScale = height > 0 ? usableHeight / height : .greatestFiniteMagnitude
        let scale = min(xScale, yScale)
        return Self(
            missionCenter: CGPoint(x: (minX + maxX) / 2, y: (minY + maxY) / 2),
            canvasCenter: CGPoint(x: size.width / 2, y: size.height / 2),
            scale: scale.isFinite ? scale : 1
        )
    }

    func canvasPoint(for point: CGPoint) -> CGPoint {
        CGPoint(
            x: canvasCenter.x + (point.x - missionCenter.x) * scale,
            y: canvasCenter.y - (point.y - missionCenter.y) * scale
        )
    }
}

enum SilentSearchMapFrontierStatus: Sendable {
    case available
    case visited
    case rejected
}

struct SilentSearchMapFrontier: Identifiable, Sendable {
    let id: String
    let point: CGPoint
    let status: SilentSearchMapFrontierStatus
}

struct SilentSearchMapState: Sendable {
    var role: RoverRole = .a
    var rover: MissionPose?
    var frontiers: [SilentSearchMapFrontier] = []
    var path: [MissionPoint] = []
    var target: MissionPoint?
    var standOff: MissionPoint?

    static let preview = SilentSearchMapState(
        role: .a,
        rover: MissionPose(position: MissionPoint(x: -0.8, y: -0.35)!, heading: 0.35),
        frontiers: [
            SilentSearchMapFrontier(id: "frontier_1", point: CGPoint(x: -1.1, y: 0.4), status: .available),
            SilentSearchMapFrontier(id: "frontier_2", point: CGPoint(x: -0.7, y: 0.9), status: .visited),
            SilentSearchMapFrontier(id: "frontier_3", point: CGPoint(x: 0.7, y: 0.3), status: .rejected),
        ],
        path: [MissionPoint(x: -0.8, y: -0.35)!, MissionPoint(x: -1.1, y: 0.4)!],
        target: MissionPoint(x: 0.2, y: 0.8)!,
        standOff: MissionPoint(x: -0.4, y: 0.8)!
    )
}

struct SilentSearchMapView: View {
    let state: SilentSearchMapState

    var body: some View {
        Canvas { context, size in
            let transform = SilentSearchMapTransform.fit(points: fittingPoints, in: size, padding: 24)
            drawSectors(context: &context, size: size, transform: transform)
            drawPath(context: &context, transform: transform)
            drawFixedPoints(context: &context, transform: transform)
            drawFrontiers(context: &context, transform: transform)
            drawTargetAndStandOff(context: &context, transform: transform)
            drawRover(context: &context, transform: transform)
        }
        .background(.black.opacity(0.86), in: RoundedRectangle(cornerRadius: 18))
        .overlay(alignment: .topLeading) {
            Text("MISSION MAP")
                .font(.caption.bold().monospaced())
                .foregroundStyle(.white.opacity(0.7))
                .padding(12)
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Silent Search mission map")
        .accessibilityIdentifier("silent_search_map")
    }

    private var fittingPoints: [CGPoint] {
        var points = [CGPoint(x: -1.4, y: -1.1), CGPoint(x: 1.4, y: 1.2), .zero]
        points += state.frontiers.map(\.point)
        points += state.path.map { CGPoint(x: $0.x, y: $0.y) }
        if let rover = state.rover { points.append(CGPoint(x: rover.position.x, y: rover.position.y)) }
        if let target = state.target { points.append(CGPoint(x: target.x, y: target.y)) }
        if let standOff = state.standOff { points.append(CGPoint(x: standOff.x, y: standOff.y)) }
        return points
    }

    private func drawSectors(context: inout GraphicsContext, size: CGSize,
                             transform: SilentSearchMapTransform) {
        let westEdge = transform.canvasPoint(for: CGPoint(x: -SilentSearchGeometry.centerBandHalfWidth, y: 0)).x
        let eastEdge = transform.canvasPoint(for: CGPoint(x: SilentSearchGeometry.centerBandHalfWidth, y: 0)).x
        context.fill(Path(CGRect(x: 0, y: 0, width: westEdge, height: size.height)),
                     with: .color(.blue.opacity(state.role == .a ? 0.16 : 0.07)))
        context.fill(Path(CGRect(x: eastEdge, y: 0, width: max(0, size.width - eastEdge), height: size.height)),
                     with: .color(.orange.opacity(state.role == .b ? 0.16 : 0.07)))
        context.fill(Path(CGRect(x: westEdge, y: 0, width: max(1, eastEdge - westEdge), height: size.height)),
                     with: .color(.red.opacity(0.13)))
        var westLine = Path()
        westLine.move(to: CGPoint(x: westEdge, y: 0)); westLine.addLine(to: CGPoint(x: westEdge, y: size.height))
        var eastLine = Path()
        eastLine.move(to: CGPoint(x: eastEdge, y: 0)); eastLine.addLine(to: CGPoint(x: eastEdge, y: size.height))
        context.stroke(westLine, with: .color(.red.opacity(0.55)), style: StrokeStyle(lineWidth: 1, dash: [5, 4]))
        context.stroke(eastLine, with: .color(.red.opacity(0.55)), style: StrokeStyle(lineWidth: 1, dash: [5, 4]))

        let origin = transform.canvasPoint(for: .zero)
        context.fill(Path(ellipseIn: CGRect(x: origin.x - 5, y: origin.y - 5, width: 10, height: 10)),
                     with: .color(.white))
        var north = Path()
        north.move(to: origin)
        north.addLine(to: transform.canvasPoint(for: CGPoint(x: 0, y: 0.45)))
        context.stroke(north, with: .color(.white), style: StrokeStyle(lineWidth: 2))
        context.draw(Text("N").font(.caption.bold()).foregroundStyle(.white),
                     at: transform.canvasPoint(for: CGPoint(x: 0, y: 0.55)))
    }

    private func drawPath(context: inout GraphicsContext, transform: SilentSearchMapTransform) {
        guard state.path.count > 1 else { return }
        var path = Path()
        path.move(to: transform.canvasPoint(for: CGPoint(x: state.path[0].x, y: state.path[0].y)))
        for point in state.path.dropFirst() {
            path.addLine(to: transform.canvasPoint(for: CGPoint(x: point.x, y: point.y)))
        }
        context.stroke(path, with: .color(.cyan), style: StrokeStyle(lineWidth: 3, lineCap: .round, dash: [8, 5]))
    }

    private func drawFixedPoints(context: inout GraphicsContext, transform: SilentSearchMapTransform) {
        for role in RoverRole.allCases {
            let point = SilentSearchGeometry.rendezvousPoint(for: role)
            let canvas = transform.canvasPoint(for: CGPoint(x: point.x, y: point.y))
            context.stroke(Path(ellipseIn: CGRect(x: canvas.x - 7, y: canvas.y - 7, width: 14, height: 14)),
                           with: .color(role == .a ? .blue : .orange), lineWidth: 2)
        }
    }

    private func drawFrontiers(context: inout GraphicsContext, transform: SilentSearchMapTransform) {
        for frontier in state.frontiers {
            let point = transform.canvasPoint(for: frontier.point)
            let color: Color = switch frontier.status {
            case .available: .yellow
            case .visited: .green
            case .rejected: .red
            }
            let rect = CGRect(x: point.x - 5, y: point.y - 5, width: 10, height: 10)
            context.fill(Path(roundedRect: rect, cornerRadius: 2), with: .color(color))
        }
    }

    private func drawTargetAndStandOff(context: inout GraphicsContext,
                                       transform: SilentSearchMapTransform) {
        if let target = state.target {
            let point = transform.canvasPoint(for: CGPoint(x: target.x, y: target.y))
            context.draw(Text("◎").font(.title.bold()).foregroundStyle(.mint), at: point)
        }
        if let standOff = state.standOff {
            let point = transform.canvasPoint(for: CGPoint(x: standOff.x, y: standOff.y))
            context.stroke(Path(ellipseIn: CGRect(x: point.x - 8, y: point.y - 8, width: 16, height: 16)),
                           with: .color(.mint), style: StrokeStyle(lineWidth: 2, dash: [3, 3]))
        }
    }

    private func drawRover(context: inout GraphicsContext, transform: SilentSearchMapTransform) {
        guard let rover = state.rover else { return }
        let center = transform.canvasPoint(for: CGPoint(x: rover.position.x, y: rover.position.y))
        let angle = CGFloat(-rover.heading)
        let forward = CGPoint(x: center.x + sin(angle) * 13, y: center.y - cos(angle) * 13)
        let left = CGPoint(x: center.x + cos(angle) * 7, y: center.y + sin(angle) * 7)
        let right = CGPoint(x: center.x - cos(angle) * 7, y: center.y - sin(angle) * 7)
        var triangle = Path()
        triangle.move(to: forward); triangle.addLine(to: left); triangle.addLine(to: right); triangle.closeSubpath()
        context.fill(triangle, with: .color(.white))
    }
}
