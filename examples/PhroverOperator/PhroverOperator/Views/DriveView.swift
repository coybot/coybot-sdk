import SwiftUI
import RoverNav
import PhroverKit

/// Manual teleop: prove the phone→ESP32 control loop, live pose, and e-stop.
struct DriveView: View {
    let ar: ARSessionManager
    let control: RoverControl

    private let speed = 0.25 // m/s for teleop

    var body: some View {
        VStack(spacing: 24) {
            telemetry

            Spacer()

            // D-pad: press-and-hold to drive, release to stop.
            VStack(spacing: 12) {
                driveButton("arrow.up", WheelCommand(left: speed, right: speed))
                    .accessibilityIdentifier("e2e_drive_forward")
                HStack(spacing: 12) {
                    driveButton("arrow.turn.up.left", WheelCommand(left: -speed, right: speed))
                        .accessibilityIdentifier("e2e_drive_left")
                    driveButton("arrow.down", WheelCommand(left: -speed, right: -speed))
                        .accessibilityIdentifier("e2e_drive_back")
                    driveButton("arrow.turn.up.right", WheelCommand(left: speed, right: -speed))
                        .accessibilityIdentifier("e2e_drive_right")
                }
            }

            Button(role: .destructive) {
                Task { try? await control.stop() }
            } label: {
                Label("E-STOP", systemImage: "stop.circle.fill").font(.title2.bold())
            }
            .buttonStyle(.borderedProminent)
            .tint(.red)
            .accessibilityIdentifier("e2e_drive_estop")

            Spacer()
        }
        .padding()
        .task { try? await control.enableFeedbackFlow() }
    }

    private var telemetry: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Tracking: \(trackingLabel)")
            if let p = ar.pose {
                Text(String(format: "Pose  x %.2f  y %.2f  yaw %.0f°",
                            p.position.x, p.position.y, p.yaw * 180 / .pi))
            }
        }
        .font(.system(.footnote, design: .monospaced))
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private var trackingLabel: String {
        switch ar.trackingState {
        case .normal: return "normal"
        case .limited: return "limited"
        case .notAvailable: return "none"
        @unknown default: return "?"
        }
    }

    private func driveButton(_ symbol: String, _ cmd: WheelCommand) -> some View {
        Image(systemName: symbol)
            .font(.largeTitle)
            .frame(width: 72, height: 72)
            .background(.tint.opacity(0.15), in: RoundedRectangle(cornerRadius: 12))
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { _ in Task { try? await control.send(cmd) } }
                    .onEnded { _ in Task { try? await control.stop() } }
            )
    }
}
