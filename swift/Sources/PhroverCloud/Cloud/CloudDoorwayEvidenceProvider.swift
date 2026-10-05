import Foundation
import PhroverKit

@MainActor
public final class CloudDoorwayEvidenceProvider: DoorwayEvidenceProviding {
    private let brain: RoverBrain
    private let timeout: Duration
    private let isOnline: () -> Bool

    public init(
        brain: RoverBrain,
        timeout: Duration = .seconds(1.5),
        isOnline: @escaping () -> Bool = { NetworkMonitor.shared.isOnline }
    ) {
        self.brain = brain
        self.timeout = timeout
        self.isOnline = isOnline
    }

    public func boostValues(
        forFrame frame: Data,
        candidates: [DoorwayCandidate]
    ) async -> [DoorwayCandidateID: Double] {
        guard isOnline(), !candidates.isEmpty else { return [:] }

        let context = MissionContext(
            utterance: "Choose the doorway candidate most likely to lead into another room.",
            frameJPEG: frame,
            explorationCandidates: candidates.map {
                ExplorationCandidate(
                    id: $0.id.rawValue,
                    worldPoint: $0.planePoint,
                    widthMeters: $0.widthMeters
                )
            }
        )

        let result = await BrainRace.run(brain: brain, context: context, timeout: timeout)

        guard case let .output(output) = result,
              case let .explore(candidateID) = output.decision,
              candidates.contains(where: { $0.id.rawValue == candidateID }) else {
            return [:]
        }

        return Dictionary(uniqueKeysWithValues: candidates.map {
            ($0.id, $0.id.rawValue == candidateID ? 1 : 0)
        })
    }
}

private enum BrainResult: Sendable {
    case output(BrainOutput)
    case noEvidence
}

@MainActor
private final class BrainRace {
    private var continuation: CheckedContinuation<BrainResult, Never>?
    private var tasks: [Task<Void, Never>] = []

    static func run(
        brain: RoverBrain,
        context: MissionContext,
        timeout: Duration
    ) async -> BrainResult {
        let race = BrainRace()
        return await withCheckedContinuation { continuation in
            race.continuation = continuation
            race.tasks = [
                Task { @MainActor in
                    do {
                        race.resolve(.output(try await brain.nextAction(context)))
                    } catch {
                        race.resolve(.noEvidence)
                    }
                },
                Task {
                    do {
                        try await Task.sleep(for: timeout)
                    } catch {
                        return
                    }
                    await race.resolve(.noEvidence)
                },
            ]
        }
    }

    private func resolve(_ result: BrainResult) {
        guard let continuation else { return }
        self.continuation = nil
        tasks.forEach { $0.cancel() }
        continuation.resume(returning: result)
    }
}
