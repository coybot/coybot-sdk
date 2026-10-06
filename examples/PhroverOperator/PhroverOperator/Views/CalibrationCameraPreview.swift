import PhroverKit
import SwiftUI

struct CalibrationCameraPreview: View {
    let image: UIImage?
    let projection: CalibrationViewProjection
    let progress: Int

    var body: some View {
        VStack(spacing: 12) {
            GeometryReader { geometry in
                ZStack {
                    Color.black
                    if let image {
                        Image(uiImage: image)
                            .resizable()
                            .scaledToFit()
                            .frame(maxWidth: .infinity, maxHeight: .infinity)
                            .accessibilityHidden(true)
                        if let corners = projection.visibleCorners {
                            markerPolygon(corners, imageSize: image.size, bounds: geometry.frame(in: .local))
                        }
                    } else {
                        ProgressView().tint(.white)
                    }
                }
                .clipShape(RoundedRectangle(cornerRadius: 16))
                .accessibilityElement(children: .contain)
                .accessibilityLabel("Calibration camera preview")
                .accessibilityIdentifier("silent_search_calibration_preview")
            }
            .frame(height: 260)

            if let markerText = projection.markerText {
                Text(markerText)
                    .font(.subheadline.monospaced().weight(.semibold))
                    .accessibilityIdentifier("silent_search_calibration_marker")
            }

            VStack(spacing: 8) {
                stageRow("QR decoded", state: projection.stages.qrDecoded, id: "silent_search_calibration_qr_stage")
                stageRow("LiDAR corners grounded", state: projection.stages.cornersGrounded, id: "silent_search_calibration_grounding_stage")
                stageRow("Sample accepted", state: projection.stages.sampleAccepted, id: "silent_search_calibration_sample_stage")
            }

            ProgressView(value: Double(progress), total: 3)
            Text("\(progress) of 3")
                .font(.subheadline.monospacedDigit())
                .accessibilityIdentifier("silent_search_calibration_progress")
            Text(projection.guidance)
                .font(.callout)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
                .accessibilityIdentifier("silent_search_calibration_guidance")
        }
    }

    private func markerPolygon(
        _ corners: OrientedMarkerCorners, imageSize: CGSize, bounds: CGRect
    ) -> some View {
        let transform = CalibrationPreviewTransform(imageSize: imageSize, previewBounds: bounds)
        var path = Path()
        path.move(to: transform.point(for: corners.topLeft))
        path.addLine(to: transform.point(for: corners.topRight))
        path.addLine(to: transform.point(for: corners.bottomRight))
        path.addLine(to: transform.point(for: corners.bottomLeft))
        path.closeSubpath()
        return path
            .stroke(.yellow, style: StrokeStyle(lineWidth: 4, lineJoin: .round))
            .shadow(color: .black.opacity(0.8), radius: 2)
            .accessibilityElement()
            .accessibilityLabel("Decoded marker outline")
            .accessibilityIdentifier("silent_search_calibration_polygon")
    }

    private func stageRow(_ title: String, state: CalibrationStageState, id: String) -> some View {
        HStack {
            Circle()
                .fill(state.color)
                .frame(width: 12, height: 12)
            Text(title)
            Spacer()
            Text(state.accessibilityValue)
                .foregroundStyle(.secondary)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 9)
        .background(.background, in: RoundedRectangle(cornerRadius: 10))
        .accessibilityElement(children: .ignore)
        .accessibilityLabel(title)
        .accessibilityValue(state.accessibilityValue)
        .accessibilityIdentifier(id)
    }
}

private extension CalibrationStageState {
    var color: Color {
        switch self {
        case .waiting: .gray
        case .pending: .yellow
        case .succeeded: .green
        }
    }

    var accessibilityValue: String {
        switch self {
        case .waiting: "Waiting"
        case .pending: "Pending"
        case .succeeded: "Succeeded"
        }
    }
}
