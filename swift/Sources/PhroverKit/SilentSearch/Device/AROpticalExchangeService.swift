import Foundation

public struct AROpticalObservation: Equatable, Sendable {
    public let frameID: ARFrameID
    public let observation: OpticalObservation

    public init(frameID: ARFrameID, observation: OpticalObservation) {
        self.frameID = frameID
        self.observation = observation
    }
}

public enum AROpticalExchangeError: Error, Equatable, Sendable {
    case timedOut
    case cancelled
}

@MainActor
public final class AROpticalExchangeService: SilentSearchOpticalExchanging {
    public typealias Scanner = (OpticalFrame) throws -> [OpticalObservation]
    public typealias Presenter = (Data?) async throws -> Void

    private let sessionManager: ARSessionManager
    private let clock: any SilentSearchClock
    private let scanner: Scanner
    private let presenter: Presenter
    private var operation: Task<Data, Error>?
    private var operationID: UUID?
    private var presentation: Task<Void, Error>?
    private var presentationID: UUID?

    public init(sessionManager: ARSessionManager, clock: any SilentSearchClock,
                scanner: OpticalQRCodeScanner = OpticalQRCodeScanner(),
                presenter: @escaping Presenter) {
        self.sessionManager = sessionManager
        self.clock = clock
        self.scanner = { try scanner.scan($0) }
        self.presenter = presenter
    }

    init(sessionManager: ARSessionManager, clock: any SilentSearchClock,
         scanner: @escaping Scanner, presenter: @escaping Presenter) {
        self.sessionManager = sessionManager
        self.clock = clock
        self.scanner = scanner
        self.presenter = presenter
    }

    public func observations(in snapshot: ARFrameSnapshot) throws -> [AROpticalObservation] {
        let frame = OpticalFrame(pixelBuffer: snapshot.image, arFrameID: snapshot.id,
                                 monotonicTimestamp: snapshot.timestamp)
        return try scanner(frame).compactMap { observation in
            guard observation.frameID == snapshot.id.sequence,
                  observation.monotonicTimestamp == snapshot.timestamp else { return nil }
            return AROpticalObservation(frameID: snapshot.id, observation: observation)
        }
    }

    public func present(payload: Data) async throws {
        try Task.checkCancellation()
        presentation?.cancel()
        let presentation = Task { @MainActor [presenter] in try await presenter(payload) }
        let presentationID = UUID()
        self.presentation = presentation
        self.presentationID = presentationID
        defer {
            if self.presentationID == presentationID {
                self.presentation = nil
                self.presentationID = nil
            }
        }
        try await withTaskCancellationHandler {
            try await presentation.value
        } onCancel: {
            presentation.cancel()
        }
    }

    public func scan(until deadline: SilentSearchInstant) async throws -> Data {
        cancel()
        let operation = Task { @MainActor [weak self] () throws -> Data in
            guard let self else { throw AROpticalExchangeError.cancelled }
            let stream = AsyncThrowingStream<Data, Error> { continuation in
                let frameTask = Task { @MainActor [weak self] in
                    guard let self else {
                        continuation.finish(throwing: AROpticalExchangeError.cancelled)
                        return
                    }
                    for await snapshot in self.sessionManager.snapshots() {
                        if Task.isCancelled { break }
                        do {
                            if let payload = try self.observations(in: snapshot).first?.observation.payload {
                                continuation.yield(payload)
                                continuation.finish()
                                return
                            }
                        } catch {
                            continuation.finish(throwing: error)
                            return
                        }
                    }
                    continuation.finish(throwing: AROpticalExchangeError.cancelled)
                }
                let timeoutTask = Task { @MainActor [weak self] in
                    guard let self else { return }
                    do {
                        try await self.clock.sleep(until: deadline)
                        if !Task.isCancelled {
                            continuation.finish(throwing: AROpticalExchangeError.timedOut)
                        }
                    } catch {
                        if !Task.isCancelled { continuation.finish(throwing: error) }
                    }
                }
                continuation.onTermination = { @Sendable _ in
                    frameTask.cancel()
                    timeoutTask.cancel()
                }
            }
            for try await payload in stream { return payload }
            throw AROpticalExchangeError.cancelled
        }
        self.operation = operation
        let operationID = UUID()
        self.operationID = operationID
        defer {
            if self.operationID == operationID {
                self.operation = nil
                self.operationID = nil
            }
        }
        do {
            return try await operation.value
        } catch is CancellationError {
            throw AROpticalExchangeError.cancelled
        }
    }

    public func cancel() {
        operation?.cancel()
        operation = nil
        operationID = nil
        presentation?.cancel()
        presentation = nil
        presentationID = nil
        Task { @MainActor [presenter] in try? await presenter(nil) }
    }
}
