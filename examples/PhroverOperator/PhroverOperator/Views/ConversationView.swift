import SwiftUI
import PhroverKit
import PhroverCloud
import CoreImage
import UIKit

/// Voice UI. Push-to-talk (hold the mic button) rather than always-on wake-word
/// listening — simpler and more reliable in noisy environments.
///
/// Backed by `MissionAgent`: Apple Intelligence gets the first reasoning attempt, an
/// optional configured cloud brain is the second stage, and supported object missions
/// retain a deterministic offline fallback when neither brain produces a usable action.
struct ConversationView: View {
    let ar: ARSessionManager
    let control: RoverControl
    let nav: NavigationController
    let topology: SessionRoomTopology
    let cloudBrain: CloudBrain?
    let doorwayEvidenceProvider: CloudDoorwayEvidenceProvider?

    @Environment(\.scenePhase) private var scenePhase
    @State private var speechIn = SpeechIn()
    @State private var speechOut = SpeechOut()
    @State private var agent: MissionAgent?
    @State private var missionPhase: MissionAgent.Phase = .idle
    @State private var lastCommand = ConversationView.makeInitialLastCommandState()
    @State private var authorized = false
    @State private var navigationDebug = NavigationDebugSummary()
    @State private var brainAvailability = ConversationView.makeInitialBrainAvailability()
    @State private var perception: ARPerceptionSource?
    @State private var detector: Detector?
    @State private var detectorLoaded = false

    var body: some View {
        VStack(spacing: 12) {
            ScrollView {
                VStack(spacing: 18) {
                    if !statusLabel.isEmpty {
                        Text(statusLabel).font(.headline)
                    }

                    LiveCameraDebugPanel(
                        ar: ar,
                        summary: navigationDebug,
                        perception: perception,
                        detectorLoaded: detectorLoaded,
                        fixtureVisibleObjects: Self.perceptionDiagnosticsFixture()
                    )
                        .frame(maxWidth: 320)

                    if let message = brainAvailability?.operatorMessage {
                        Text(message)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .multilineTextAlignment(.center)
                            .frame(maxWidth: 320)
                            .accessibilityIdentifier("brain-availability-message")
                    }

                    if let record = lastCommand.record {
                        VStack(alignment: .leading, spacing: 8) {
                            Text(record.command)
                                .font(.headline)
                                .accessibilityIdentifier("last-command-text")
                            Text(record.status.rawValue)
                                .font(.subheadline.weight(.semibold))
                                .foregroundStyle(statusColor(record.status))
                                .accessibilityIdentifier("last-command-status")
                            Text(record.message)
                                .font(.subheadline)
                                .foregroundStyle(.secondary)
                                .accessibilityIdentifier("last-command-message")
                        }
                        .frame(maxWidth: 320, alignment: .leading)
                        .padding(14)
                        .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 14))
                        .accessibilityElement(children: .contain)
                        .accessibilityIdentifier("last-command-card")
                    }
                }
                .frame(maxWidth: .infinity)
            }
            .scrollIndicators(.hidden)

            VStack(spacing: 12) {
                if let commandText {
                    Text(commandText)
                        .font(.subheadline)
                        .foregroundStyle(.secondary)
                        .multilineTextAlignment(.center)
                        .lineLimit(2)
                        .frame(maxWidth: 320)
                        .accessibilityIdentifier("recognized-command-text")
                }

                if agent != nil {
                    Text(phaseStatusLabel)
                        .font(.subheadline)
                        .padding(.horizontal, 18)
                        .padding(.vertical, 10)
                        .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 12))
                }

                Image(systemName: "mic.circle.fill")
                    .font(.system(size: 72))
                    .frame(width: 96, height: 96)
                    .contentShape(Circle())
                    .foregroundStyle(speechIn.state == .listening ? .red : .accentColor)
                    .accessibilityLabel("Push to talk")
                    .accessibilityIdentifier("push-to-talk-control")
                    .gesture(
                        DragGesture(minimumDistance: 0)
                            .onChanged { _ in startListening() }
                            .onEnded { _ in speechIn.finish() }
                    )
            }
            .padding(.bottom, 8)
        }
        .padding(.horizontal)
        .padding(.top, 20)
        .task {
            authorized = await speechIn.requestAuthorization()
            let detector = await Detector()
            detector.setInferenceEnabled(scenePhase == .active)
            let perception = ARPerceptionSource(ar: ar, detector: detector)
            self.detector = detector
            self.perception = perception
            detectorLoaded = detector.isLoaded
            let voice = SpeechRoverVoice(out: speechOut, speechIn: speechIn)
            let onDevice = OnDeviceBrain()
            let availability = Self.brainAvailabilityFixture() ?? onDevice.availability
            brainAvailability = availability
            RuntimeFileLog.append("on_device_brain_availability", fields: [
                "state": availability.logValue,
            ])
            let telemetry: MissionTelemetrySink = { event, fields in
                RuntimeFileLog.append(event, fields: fields)
            }
            // "follow me" needs a follower; without one the agent can only say it can't.
            let follower = FollowController(perception: perception,
                                            drive: LocalRoverDrive(ar: ar, control: control))
            let brain: RoverBrain = HybridBrain(
                cloud: cloudBrain,
                onDevice: onDevice,
                missionTelemetry: telemetry
            )
            agent = MissionAgent(
                motion: nav,
                perception: perception,
                voice: voice,
                follower: follower,
                roomTopology: topology,
                doorwayEvidenceProvider: doorwayEvidenceProvider,
                phaseDidChange: { phase in
                    missionPhase = phase
                },
                roomTransitionStateDidChange: { state in
                    navigationDebug.apply(state)
                },
                commandStatusDidChange: { status in
                    lastCommand.reduce(status)
                },
                missionTelemetry: telemetry
            ) { brain }
        }
        .onChange(of: scenePhase) { _, phase in
            detector?.setInferenceEnabled(phase == .active)
        }
    }

    private static func makeInitialBrainAvailability() -> OnDeviceBrainAvailability? {
        brainAvailabilityFixture()
    }

    private static func brainAvailabilityFixture() -> OnDeviceBrainAvailability? {
#if DEBUG
        let arguments = ProcessInfo.processInfo.arguments
        if arguments.contains("-ui-test-brain-model-not-ready") { return .modelNotReady }
        if arguments.contains("-ui-test-brain-available") { return .available }
#endif
        return nil
    }

    private static func makeInitialLastCommandState() -> LastCommandState {
        var state = LastCommandState()
#if DEBUG
        if ProcessInfo.processInfo.arguments.contains("-ui-test-failed-command") {
            state.reduce(.recognized(id: 1, command: "Go to our room"))
            state.reduce(.failed(
                id: 1,
                command: "Go to our room",
                message: "I can't safely see the space needed to turn."
            ))
        }
#endif
        return state
    }

    private static func perceptionDiagnosticsFixture() -> String? {
#if DEBUG
        if ProcessInfo.processInfo.arguments.contains("-ui-test-perception-diagnostics") {
            return "refrigerator 99%"
        }
#endif
        return nil
    }

    private var statusLabel: String {
        if !authorized { return "Enable Speech Recognition in Settings" }
        switch speechIn.state {
        case .listening: return "Listening…"
        case .processing: return "Processing speech…"
        case .unavailable: return "Speech recognition unavailable"
        case .idle: return ""
        }
    }

    private func phaseLabel(_ phase: MissionAgent.Phase) -> String {
        switch phase {
        case .idle: return "Ready"
        case .thinking: return "Thinking…"
        case .acting: return "On it…"
        case .waitingForAnswer: return "Waiting for your answer…"
        }
    }

    private var phaseStatusLabel: String {
        if speechIn.state == .listening {
            return speechIn.partialTranscript.isEmpty ? "Listening…" : "Processing speech…"
        }
        if speechIn.state == .processing { return "Processing speech…" }
        return phaseLabel(missionPhase)
    }

    private var commandText: String? {
        let partial = speechIn.partialTranscript.trimmingCharacters(in: .whitespacesAndNewlines)
        if !partial.isEmpty { return partial }
        return lastCommand.record?.command
    }

    private func statusColor(_ status: LastCommandState.DisplayStatus) -> Color {
        switch status {
        case .listening, .recognized, .working: return .accentColor
        case .succeeded: return .green
        case .failed: return .red
        case .cancelled: return .secondary
        }
    }

    private func startListening() {
        guard authorized, agent != nil, speechIn.state != .listening else { return }
        do {
            try speechIn.start(onEvent: { event in
                lastCommand.reduce(event)
            }) { captureID, utterance in
                lastCommand.finalizeCapture(captureID)
                Task { @MainActor in
                    await agent?.handle(utterance)
                }
            }
        } catch {
            RuntimeFileLog.append("speech_capture_ui_error", fields: [
                "error": String(describing: error)
            ])
        }
    }
}

private struct LiveCameraDebugPanel: View {
    let ar: ARSessionManager
    let summary: NavigationDebugSummary
    let perception: ARPerceptionSource?
    let detectorLoaded: Bool
    let fixtureVisibleObjects: String?

    @Environment(\.displayScale) private var displayScale
    @State private var previewImage: UIImage?
    @State private var visibleObjects = "none"
    @State private var lastDetectionAt = Date.distantPast

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Group {
                if let previewImage {
                    Image(uiImage: previewImage)
                        .resizable()
                        .scaledToFit()
                } else {
                    ZStack {
                        Color.black.opacity(0.08)
                        Image(systemName: "camera")
                            .font(.title2)
                            .foregroundStyle(.secondary)
                    }
                }
            }
            .frame(maxWidth: .infinity)
            .frame(height: 160)
            .background(.black.opacity(0.06))
            .clipShape(RoundedRectangle(cornerRadius: 8))
            .overlay(alignment: .topLeading) {
                Text("Live")
                    .font(.caption2.bold())
                    .padding(.horizontal, 6)
                    .padding(.vertical, 3)
                    .background(.thinMaterial, in: Capsule())
                    .padding(6)
            }

            VStack(alignment: .leading, spacing: 2) {
                Text("Tracking: \(trackingLabel)")
                Text("Detector: \(effectiveDetectorLoaded ? "loaded" : "unavailable")")
                    .accessibilityIdentifier("detector-status-text")
                Text("Visible: \(effectiveVisibleObjects)")
                    .lineLimit(1)
                    .accessibilityIdentifier("visible-objects-text")
                Text("Openings: \(summary.openingsText)")
                Text("Doorway candidates: \(summary.doorwayCandidatesText)")
                Text("Target: \(summary.targetText)")
                Text("Transition: \(summary.transitionText)")
                    .lineLimit(2)
            }
            .font(.system(.caption2, design: .monospaced))
            .foregroundStyle(.secondary)
        }
        .task {
            await refreshLoop()
        }
    }

    private var trackingLabel: String {
        switch ar.trackingState {
        case .normal: return "normal"
        case .limited: return "limited"
        case .notAvailable: return "none"
        @unknown default: return "?"
        }
    }

    private var effectiveDetectorLoaded: Bool {
        fixtureVisibleObjects != nil || detectorLoaded
    }

    private var effectiveVisibleObjects: String {
        fixtureVisibleObjects ?? visibleObjects
    }

    @MainActor
    private func refreshLoop() async {
        while !Task.isCancelled {
            refresh()
            try? await Task.sleep(for: .milliseconds(500))
        }
    }

    @MainActor
    private func refresh() {
        guard let buffer = ar.latestPixelBuffer else {
            previewImage = nil
            return
        }
        previewImage = Self.previewImage(from: buffer, scale: displayScale)
        guard fixtureVisibleObjects == nil,
              detectorLoaded,
              let perception,
              Date().timeIntervalSince(lastDetectionAt) >= 1 else { return }
        lastDetectionAt = Date()
        visibleObjects = PerceptionDebugSummary.visibleObjects(perception.detectObjects())
    }

    private static func previewImage(from pixelBuffer: CVPixelBuffer, scale: CGFloat) -> UIImage? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else { return nil }
        return UIImage(cgImage: cgImage, scale: scale, orientation: .right)
    }
}
