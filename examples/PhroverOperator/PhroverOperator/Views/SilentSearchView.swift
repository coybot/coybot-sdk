import PhroverKit
import SwiftUI

struct SilentSearchView<Model: SilentSearchViewModel>: View {
    @Bindable var viewModel: Model

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 18) {
                    phaseHeader
                    phaseContent
                    if !isTerminal && viewModel.phase != .displayingQR {
                        SilentSearchMapView(state: viewModel.mapState)
                            .frame(minHeight: 260)
                    }
                    controls
                }
                .padding()
            }
            .background(Color(.systemGroupedBackground))
            .navigationTitle("Silent Search")
            .navigationBarTitleDisplayMode(.inline)
            .task { viewModel.refreshReadiness() }
        }
    }

    private var phaseHeader: some View {
        VStack(spacing: 7) {
            Text(viewModel.title)
                .font(isTerminal ? .system(size: 44, weight: .black, design: .rounded) : .title2.bold())
                .multilineTextAlignment(.center)
                .accessibilityIdentifier(terminalIdentifier)
            Text(viewModel.detail)
                .foregroundStyle(isFailure ? .red : .secondary)
                .multilineTextAlignment(.center)
                .accessibilityIdentifier(isFailure ? "silent_search_failure_reason" : "silent_search_phase_detail")
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, isTerminal ? 32 : 8)
    }

    @ViewBuilder
    private var phaseContent: some View {
        switch viewModel.phase {
        case .setup:
            setup
        case .calibrating:
            calibration
        case .readyForExchange:
            Label("Marker SILENT_SEARCH_01 accepted", systemImage: "checkmark.seal.fill")
                .foregroundStyle(.green)
        case .displayingQR:
            qrPresentation
        case .scanning:
            scanner
        case .searching:
            metrics([("Sector", viewModel.role.searchSector.rawValue.capitalized),
                     ("Policy", "Sector constrained"), ("Target", viewModel.targetLabel)])
        case .returning:
            metrics([("Destination", "Fixed rendezvous"), ("Policy", "Sector constrained")])
        case .rendezvousRotating:
            metrics([("Motion", "Optical alignment"), ("Timeout", "30 seconds after alignment")])
        case .intentionalWait:
            Label("Motors stopped", systemImage: "pause.circle.fill")
                .font(.headline)
                .foregroundStyle(.blue)
        case .converging:
            metrics([("Target", viewModel.targetLabel), ("Stand-off", "0.60 m"),
                     ("Policy", "Acknowledged convergence")])
        case .terminalFound, .terminalNotFound, .terminalFailure:
            EmptyView()
        }
    }

    private var setup: some View {
        VStack(spacing: 14) {
            Picker("Rover role", selection: $viewModel.role) {
                Text("Rover A · West").tag(RoverRole.a)
                Text("Rover B · East").tag(RoverRole.b)
            }
            .pickerStyle(.segmented)
            .accessibilityIdentifier("silent_search_role_picker")
            Picker("Target class", selection: $viewModel.targetLabel) {
                ForEach(viewModel.supportedTargetLabels, id: \.self) { label in
                    Text(label).tag(label)
                }
            }
            .pickerStyle(.menu)
            .accessibilityIdentifier("silent_search_target_picker")
            Stepper(value: $viewModel.durationSeconds, in: 30...3600, step: 30) {
                LabeledContent("Search duration", value: "\(viewModel.durationSeconds) seconds")
            }
            .accessibilityIdentifier("silent_search_duration")
            VStack(spacing: 9) {
                ForEach(Array(viewModel.readinessItems.enumerated()), id: \.offset) { _, item in
                    HStack {
                        Image(systemName: item.1 ? "checkmark.circle.fill" : "xmark.circle.fill")
                            .foregroundStyle(item.1 ? .green : .red)
                        Text(item.0)
                        Spacer()
                        Text(item.1 ? "Ready" : "Required").foregroundStyle(.secondary)
                    }
                }
            }
            .padding()
            .background(.background, in: RoundedRectangle(cornerRadius: 14))
            Button("Refresh readiness") { viewModel.refreshReadiness() }
                .buttonStyle(.bordered)
        }
    }

    private var calibration: some View {
        CalibrationCameraPreview(
            image: viewModel.calibrationPreviewImage,
            projection: viewModel.calibrationProjection,
            progress: viewModel.calibrationProgress
        )
        .padding()
        .onAppear { viewModel.startCalibrationPreview() }
        .onDisappear { viewModel.stopCalibrationPreview() }
    }

    @ViewBuilder
    private var qrPresentation: some View {
        if let image = viewModel.qrImage {
            Image(image, scale: 1, label: Text("Silent Search QR code"))
                .interpolation(.none)
                .resizable()
                .scaledToFit()
                .padding(18)
                .background(.white, in: RoundedRectangle(cornerRadius: 12))
                .accessibilityIdentifier("silent_search_qr")
        } else {
            ProgressView("Preparing QR code")
        }
    }

    private var scanner: some View {
        VStack(spacing: 12) {
            ZStack {
                RoundedRectangle(cornerRadius: 22)
                    .stroke(viewModel.opticalTimedOut ? .red : .blue,
                            style: StrokeStyle(lineWidth: 4, dash: [18, 8]))
                Image(systemName: viewModel.opticalTimedOut ? "qrcode.viewfinder" : "camera.viewfinder")
                    .font(.system(size: 64, weight: .thin))
                    .foregroundStyle(viewModel.opticalTimedOut ? .red : .blue)
            }
            .frame(height: 210)
            .accessibilityElement(children: .ignore)
            .accessibilityLabel(viewModel.opticalTimedOut ? "QR scanner timed out" : "QR scanner active")
            .accessibilityIdentifier("silent_search_scanner")
            Text("Only a valid message for this mission, marker, role, sequence, and phase is accepted.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
    }

    private func metrics(_ values: [(String, String)]) -> some View {
        VStack(spacing: 10) {
            ForEach(Array(values.enumerated()), id: \.offset) { _, value in
                LabeledContent(value.0, value: value.1)
            }
        }
        .padding()
        .background(.background, in: RoundedRectangle(cornerRadius: 14))
    }

    @ViewBuilder
    private var controls: some View {
        if viewModel.phase == .setup || viewModel.phase == .readyForExchange {
            Button(viewModel.phase == .setup ? "Start calibration" : "Begin optical exchange") {
                viewModel.start()
            }
            .buttonStyle(.borderedProminent)
            .controlSize(.large)
            .disabled(!viewModel.canStart)
            .accessibilityIdentifier("silent_search_start")
        }
        if viewModel.phase == .displayingQR || viewModel.phase == .scanning {
            VStack {
                if viewModel.phase == .displayingQR {
                    Button("Partner scanned code") { viewModel.completeQRPresentation() }
                        .buttonStyle(.borderedProminent)
                        .accessibilityIdentifier("silent_search_qr_complete")
                }
                HStack {
                if viewModel.opticalTimedOut {
                    Button("Retry") { viewModel.retry() }
                        .buttonStyle(.borderedProminent)
                        .accessibilityIdentifier("silent_search_retry")
                }
                Button("Abort", role: .destructive) { viewModel.abort() }
                    .buttonStyle(.bordered)
                    .accessibilityIdentifier("silent_search_abort")
                }
            }
            .controlSize(.large)
        }
        if viewModel.showsStop {
            Button("Stop", role: .destructive) { viewModel.stop() }
                .buttonStyle(.borderedProminent)
                .tint(.red)
                .controlSize(.large)
                .frame(maxWidth: .infinity)
                .accessibilityIdentifier("silent_search_stop")
        }
    }

    private var isTerminal: Bool {
        [.terminalFound, .terminalNotFound, .terminalFailure].contains(viewModel.phase)
    }
    private var isFailure: Bool { viewModel.phase == .terminalFailure }
    private var terminalIdentifier: String {
        switch viewModel.phase {
        case .terminalFound: "silent_search_terminal_found"
        case .terminalNotFound: "silent_search_terminal_not_found"
        case .terminalFailure: "silent_search_terminal_failure"
        default: "silent_search_phase_title"
        }
    }
}
