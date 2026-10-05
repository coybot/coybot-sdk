import Foundation
import PhroverKit

/// Wraps an on-device-primary/cloud-fallback pair behind a single `RoverBrain`, so
/// `MissionAgent` doesn't need to know anything about connectivity or failure recovery.
/// The local Apple Intelligence brain runs to completion when cloud fallback is not
/// usable. Online missions give it a bounded first chance before trying cloud.
@MainActor
public final class HybridBrain: RoverBrain {
    private let cloud: RoverBrain?
    private let onDevice: RoverBrain
    private let primaryTimeout: Duration
    private let isOnline: () -> Bool
    private let missionTelemetry: MissionTelemetrySink

    public init(
        cloud: RoverBrain?,
        onDevice: RoverBrain,
        primaryTimeout: Duration = .seconds(8),
        isOnline: @escaping () -> Bool = { NetworkMonitor.shared.isOnline },
        missionTelemetry: @escaping MissionTelemetrySink = { event, fields in
            RuntimeFileLog.append(event, fields: fields)
        }
    ) {
        self.cloud = cloud
        self.onDevice = onDevice
        self.primaryTimeout = primaryTimeout
        self.isOnline = isOnline
        self.missionTelemetry = missionTelemetry
    }

    public func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        guard cloud != nil, isOnline() else {
            return try await onDeviceAction(context)
        }

        let primaryResult = await PrimaryStageRace.run(
            brain: onDevice,
            context: context,
            timeout: primaryTimeout
        )
        try Task.checkCancellation()

        switch primaryResult {
        case .output(let output):
            logSelection("on_device", reason: "primary", missionID: context.missionID)
            return output
        case .failure(let error):
            if error is CancellationError {
                throw error
            }
            return try await cloudAction(context, reason: "on_device_failed", primaryError: error)
        case .timedOut:
            return try await cloudAction(
                context,
                reason: "on_device_timed_out",
                primaryError: PrimaryStageTimeoutError(timeout: primaryTimeout)
            )
        case .cancelled:
            throw CancellationError()
        }
    }

    private func onDeviceAction(_ context: MissionContext) async throws -> BrainOutput {
        do {
            let output = try await onDevice.nextAction(context)
            logSelection("on_device", reason: "primary", missionID: context.missionID)
            return output
        } catch is CancellationError {
            throw CancellationError()
        } catch {
            logSelection(
                "on_device",
                reason: "on_device_failed",
                missionID: context.missionID,
                error: error
            )
            throw error
        }
    }

    private func cloudAction(
        _ context: MissionContext,
        reason: String,
        primaryError: Error
    ) async throws -> BrainOutput {
        try Task.checkCancellation()
        guard let cloud, isOnline() else {
            logSelection("on_device", reason: reason, missionID: context.missionID, error: primaryError)
            throw primaryError
        }

        do {
            let output = try await cloud.nextAction(context)
            logSelection("cloud", reason: reason, missionID: context.missionID, error: primaryError)
            return output
        } catch is CancellationError {
            throw CancellationError()
        } catch {
            logSelection("cloud", reason: "cloud_failed", missionID: context.missionID, error: error)
            throw error
        }
    }

    private func logSelection(
        _ brain: String,
        reason: String,
        missionID: Int?,
        error: Error? = nil
    ) {
        var fields = [
            "mission": missionID.map(String.init) ?? "none",
            "brain": brain,
            "reason": reason,
        ]
        if let error {
            fields["error"] = error.localizedDescription
        }
        missionTelemetry("mission_brain_selected", fields)
    }
}

private enum PrimaryStageResult {
    case output(BrainOutput)
    case failure(Error)
    case timedOut
    case cancelled
}

@MainActor
private final class PrimaryStageRace {
    private var continuation: CheckedContinuation<PrimaryStageResult, Never>?
    private var tasks: [Task<Void, Never>] = []
    private var cancellationRequested = false

    static func run(brain: RoverBrain, context: MissionContext, timeout: Duration) async -> PrimaryStageResult {
        let race = PrimaryStageRace()
        return await withTaskCancellationHandler {
            await withCheckedContinuation { continuation in
                race.start(
                    brain: brain,
                    context: context,
                    timeout: timeout,
                    continuation: continuation
                )
            }
        } onCancel: {
            Task { @MainActor in
                race.cancel()
            }
        }
    }

    private func start(
        brain: RoverBrain,
        context: MissionContext,
        timeout: Duration,
        continuation: CheckedContinuation<PrimaryStageResult, Never>
    ) {
        self.continuation = continuation
        guard !cancellationRequested, !Task.isCancelled else {
            finish(.cancelled)
            return
        }

        tasks = [
            Task { @MainActor in
                do {
                    self.finish(.output(try await brain.nextAction(context)))
                } catch is CancellationError {
                    self.finish(.cancelled)
                } catch {
                    self.finish(.failure(error))
                }
            },
            Task { @MainActor in
                do {
                    try await Task.sleep(for: timeout)
                } catch {
                    return
                }
                self.finish(.timedOut)
            }
        ]
    }

    private func cancel() {
        cancellationRequested = true
        finish(.cancelled)
    }

    private func finish(_ result: PrimaryStageResult) {
        guard let continuation else { return }
        self.continuation = nil
        let runningTasks = tasks
        tasks.removeAll()
        runningTasks.forEach { $0.cancel() }
        continuation.resume(returning: result)
    }
}

private struct PrimaryStageTimeoutError: LocalizedError {
    let timeout: Duration

    var errorDescription: String? {
        "On-device brain did not respond within \(timeout)."
    }
}
