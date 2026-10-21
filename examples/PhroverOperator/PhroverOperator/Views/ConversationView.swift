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
    private let scripted: Bool
    private let mayStartFollow: @MainActor () -> Bool
    @State private var model: ConversationViewModel

    init(ar: ARSessionManager, control: RoverControl, nav: NavigationController,
         topology: SessionRoomTopology, cloudBrain: CloudBrain?,
         doorwayEvidenceProvider: CloudDoorwayEvidenceProvider?,
         model: ConversationViewModel? = nil, scripted: Bool = false,
         mayStartFollow: @escaping @MainActor () -> Bool = { true }) {
        self.ar = ar
        self.control = control
        self.nav = nav
        self.topology = topology
        self.cloudBrain = cloudBrain
        self.doorwayEvidenceProvider = doorwayEvidenceProvider
        self.scripted = scripted
        self.mayStartFollow = mayStartFollow
        _model = State(initialValue: model ?? ConversationViewModel())
    }

    @Environment(\.scenePhase) private var scenePhase
    @State private var speechIn = SpeechIn()
    @State private var speechOut = SpeechOut()
    @State private var agent: MissionAgent?
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

                    if !scripted {
                        LiveCameraDebugPanel(
                            ar: ar,
                            summary: navigationDebug,
                            perception: perception,
                            detectorLoaded: detectorLoaded,
                            detector: detector,
                            trackedFrame: model.trackedPersonFrameID,
                            fixtureVisibleObjects: Self.perceptionDiagnosticsFixture()
                        )
                            .frame(maxWidth: 320)
                    }

                    if !model.status.isEmpty {
                        Text(model.status)
                            .fixedSize(horizontal: false, vertical: true)
                            .frame(maxWidth: 320, alignment: .leading)
                            .accessibilityIdentifier("talk_follow_status")
                    }
                    if let message = model.errorMessage {
                        Text(message).foregroundStyle(.red)
                    }
                    if model.showsStopFollowing {
                        Button("Stop Following", role: .destructive) {
                            Task { await model.stopFollowing() }
                        }
                        .accessibilityIdentifier("talk_stop_following")
                    }

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

                if agent != nil && model.status.isEmpty && model.errorMessage == nil {
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
            guard !scripted else { return }
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
            let brain: RoverBrain = HybridBrain(
                cloud: cloudBrain,
                onDevice: onDevice,
                missionTelemetry: telemetry
            )
            agent = MissionAgent(
                motion: nav,
                perception: perception,
                voice: voice,
                roomTopology: topology,
                doorwayEvidenceProvider: doorwayEvidenceProvider,
                phaseDidChange: { phase in
                    model.receiveMissionPhase(phase)
                },
                roomTransitionStateDidChange: { state in
                    navigationDebug.apply(state)
                },
                commandStatusDidChange: { status in
                    lastCommand.reduce(status)
                },
                missionTelemetry: telemetry
            ) { brain }
            let followClock = SystemFollowClock()
            let follow = FollowMeCoordinator(
                perception: ARFollowMePerceptionSource(ar: ar, detector: detector),
                motion: NavigationFollowMeMotion(navigation: nav), clock: followClock,
                configuration: .continuous
            )
            guard let agent else { return }
            let router = OperatorCommandRouter(mission: agent, follow: follow,
                                               mayStartFollow: mayStartFollow, clock: followClock)
            model.configure(submit: { await router.submit($0) },
                            stop: { await router.stop() }, followState: { follow.state },
                            targetFrame: { follow.trackedPersonFrameID },
                            readySignalClearance: { follow.readySignalClearance },
                            inhibit: { follow.inhibitMotion() },
                            submitFinalized: { text, receivedAt in
                                await router.submit(text, finalizedTextReceivedAt: receivedAt)
                            }, monotonic: { followClock.now })
            authorized = await speechIn.requestAuthorization()
        }
        .onDisappear {
            model.prepareToLeave()
            Task { _ = await model.leaveTalk() }
        }
        .onChange(of: scenePhase) { _, phase in
            detector?.setInferenceEnabled(phase == .active)
            if phase != .active {
                model.prepareToLeave()
                Task { _ = await model.leaveTalk() }
            }
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
        if scripted { return "" }
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
        return phaseLabel(model.missionPhase)
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
                    await model.submitFinalSpeech(utterance)
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
    let detector: Detector?
    let trackedFrame: ARFrameID?
    let fixtureVisibleObjects: String?

    @Environment(\.displayScale) private var displayScale
    @State private var previewImage: UIImage?
    @State private var visibleObjects = "none"
    @State private var predictions: [Detector.Detection] = []
    @State private var personVerification: [PersonBodyVerifier.Decision]?
    @State private var previewFrame: ARFrameID?
    @State private var previewAge: TimeInterval?

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Group {
                if let previewImage {
                    Image(uiImage: previewImage)
                        .resizable()
                        .scaledToFit()
                        .overlay {
                            GeometryReader { geometry in
                                ForEach(predictions.indices, id: \.self) { index in
                                    let detection = predictions[index]
                                    let box = detection.boundingBox.intersection(CGRect(x: 0, y: 0, width: 1, height: 1))
                                    let checked = PerceptionDebugSummary.verification(forDetectionAt: index,
                                        in: predictions, decisions: personVerification)?.accepted ?? false
                                    if !box.isNull, box.width > 0, box.height > 0 {
                                        Rectangle().stroke(checked ? Color.green : Color.orange, lineWidth: 1)
                                            .frame(width: box.width * geometry.size.width, height: box.height * geometry.size.height)
                                            .overlay(alignment: .topLeading) {
                                                Text("\(detection.label) (raw)")
                                                    .font(.system(size: 8, design: .monospaced))
                                                    .foregroundStyle(.black)
                                                    .background(checked ? Color.green.opacity(0.8) : Color.orange.opacity(0.8))
                                            }
                                            .position(x: box.midX * geometry.size.width, y: (1 - box.midY) * geometry.size.height)
                                    }
                                }
                            }
                        }
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
                Text(personVerification == nil || personVerification?.contains(where: \.verificationUnavailable) == true
                     ? "Person check: unavailable — no verified candidate" :
                    ((personVerification?.filter(\.accepted).count ?? 0) == 0 ? "Person check: none body-verified" :
                        "Person check: \(personVerification?.filter(\.accepted).count ?? 0) body candidate(s)"))
                if let trackedFrame {
                    Text("Follow target: acquired • frame \(trackedFrame.generation):\(trackedFrame.sequence)")
                } else {
                    Text("Follow target: none")
                }
                if let previewFrame, let previewAge {
                    Text("Frame: \(previewFrame.generation):\(previewFrame.sequence) • age \(String(format: "%.0f", previewAge * 1000)) ms")
                }
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
            await refresh()
            try? await Task.sleep(for: .milliseconds(500))
        }
    }

    @MainActor
    private func refresh() async {
        guard let snapshot = ar.latestSnapshot else {
            previewImage = nil
            predictions = []
            personVerification = nil
            previewFrame = nil
            previewAge = nil
            visibleObjects = "none"
            return
        }
        guard let detector else {
            previewImage = Self.previewImage(from: snapshot.image, scale: displayScale)
            predictions = []
            personVerification = nil
            previewFrame = nil
            previewAge = nil
            return
        }
        guard let evaluation = await detector.followPreviewEvaluationAsync(
            snapshot, at: ProcessInfo.processInfo.systemUptime), !Task.isCancelled,
            ar.sessionGeneration == evaluation.snapshot.id.generation else { return }
        previewImage = Self.previewImage(from: evaluation.snapshot.image, scale: displayScale)
        predictions = evaluation.receipt.frame.detections
        personVerification = evaluation.receipt.personVerification
        previewFrame = evaluation.snapshot.id
        let age = ProcessInfo.processInfo.systemUptime - evaluation.snapshot.timestamp
        previewAge = age.isFinite && age >= 0 ? age : nil
        visibleObjects = PerceptionDebugSummary.visibleObjects(predictions.map {
            PerceivedObject(label: $0.label, confidence: $0.confidence,
                normalizedPoint: CGPoint(x: $0.boundingBox.midX, y: $0.boundingBox.midY))
        })
    }

    private static func previewImage(from pixelBuffer: CVPixelBuffer, scale: CGFloat) -> UIImage? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else { return nil }
        return UIImage(cgImage: cgImage, scale: scale, orientation: .right)
    }
}
