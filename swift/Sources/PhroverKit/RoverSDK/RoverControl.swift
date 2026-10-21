import Foundation
import RoverNav

public enum RoverCommandLinkReadiness: Equatable, Sendable {
    case available
    case unavailable
}

/// Low-level driver for the WAVE ROVER ESP32 over WiFi HTTP.
///
/// Speaks the Waveshare JSON command protocol: commands are sent as
/// `GET /js?json={"T":1,"L":<left>,"R":<right>}`. WAVE ROVER maps these
/// values to open-loop PWM; encoder-equipped bases may interpret them as m/s.
public actor RoverControl {
    private let baseURL: URL
    private let session: URLSession
    private let retrySleep: @Sendable (TimeInterval) async -> Void

    /// Timestamp of the last successful command; the comms watchdog reads this.
    public private(set) var lastAckAt: Date?

    public init(host: String = RoverConfig.defaultHost, session: URLSession = .shared) {
        self.baseURL = URL(string: "http://\(host)")!
        self.session = session
        self.retrySleep = { try? await Task.sleep(for: .seconds($0)) }
    }

    init(host: String = RoverConfig.defaultHost, session: URLSession,
         retrySleep: @escaping @Sendable (TimeInterval) async -> Void) {
        self.baseURL = URL(string: "http://\(host)")!
        self.session = session
        self.retrySleep = retrySleep
    }

    // MARK: - Motion

    func sendNavigationWithReceipt(_ cmd: WheelCommand) async -> RoverCommandDiagnosticResult {
        let authorization = cmd.left != 0 || cmd.right != 0 ? FollowTurnBurstTransportScope.authorization : nil
        if let authorization {
            authorization.transportCapture?.recordTiming(.init(boundary: .actorEntry, uptime: authorization.uptime()))
        }
        return await sendJSONDiagnostic(["T": RoverConfig.Opcode.speedControl, "L": cmd.right, "R": cmd.left],
            authorization: authorization)
    }

    func sendNavigationWithReceipt(_ cmd: WheelCommand,
                                   authorization: FollowTurnBurstAuthorization) async -> RoverCommandDiagnosticResult {
        let selected = cmd.left != 0 || cmd.right != 0 ? authorization : nil
        selected?.transportCapture?.recordTiming(.init(boundary: .actorEntry, uptime: authorization.uptime()))
        return await sendJSONDiagnostic(["T": RoverConfig.Opcode.speedControl, "L": cmd.right, "R": cmd.left],
            authorization: selected)
    }

    func stopWithReceipt() async -> RoverCommandDiagnosticResult {
        // WAVE ROVER's chassis dispatcher implements T:1, not T:0. Its HTTP
        // endpoint also returns 200 for unhandled opcodes, so T:0 can appear
        // acknowledged while leaving the previous wheel outputs running.
        // Bypass burst authorization: stopping must work after its expiry/fence.
        await sendJSONDiagnostic(["T": RoverConfig.Opcode.speedControl, "L": 0.0, "R": 0.0])
    }

    /// Stream a differential-drive command. The single source of motion for autonomy.
    public func send(_ cmd: WheelCommand) async throws {
        try await sendJSON(["T": RoverConfig.Opcode.speedControl,
                            "L": cmd.left,
                            "R": cmd.right])
    }

    /// Send a navigation command after converting RoverNav yaw to the mounted
    /// WAVE ROVER's physical turn direction. Manual drive commands use `send(_:)`.
    public func sendNavigation(_ cmd: WheelCommand) async throws {
        // RoverNav uses mathematical CCW-positive yaw. ARKit's x/world-z ground plane
        // reports the opposite physical turn sign, so swap wheel channels only at the
        // WAVE ROVER boundary. Forward/reverse commands are unchanged by the swap.
        _ = try await sendNavigationWithReceipt(cmd).get()
    }

    /// Command both wheel outputs to zero. Safe to repeat; used by e-stop and
    /// the watchdog. The receipt confirms HTTP acceptance, not physical rest.
    public func stop() async throws {
        _ = try await stopWithReceipt().get()
    }

    /// Ask the base to stream continuous chassis + IMU feedback (parsed by `RoverFeedback`).
    public func enableFeedbackFlow() async throws {
        try await sendJSON(["T": RoverConfig.Opcode.feedbackFlowOn, "cmd": 1])
    }

    /// Verify the command link without issuing a wheel-motion opcode.
    public func probeLink() async throws {
        try await sendJSON(["T": RoverConfig.Opcode.feedbackFlowOn, "cmd": 1])
    }

    public func commandLinkReadiness() -> AsyncStream<RoverCommandLinkReadiness> {
        AsyncStream { continuation in
            let task = Task {
                var previous: RoverCommandLinkReadiness?
                while !Task.isCancelled {
                    let readiness: RoverCommandLinkReadiness
                    do {
                        try await probeLink()
                        readiness = .available
                    } catch {
                        readiness = .unavailable
                    }
                    if readiness != previous, readiness == .available || previous != nil {
                        continuation.yield(readiness)
                    }
                    previous = readiness
                    try? await Task.sleep(for: .seconds(1))
                }
            }
            continuation.onTermination = { @Sendable _ in task.cancel() }
        }
    }

    // MARK: - Transport

    private func sendJSON(_ payload: [String: Any]) async throws {
        _ = try await sendJSONDiagnostic(payload).get()
    }

    private func sendJSONDiagnostic(_ payload: [String: Any],
                                    authorization: FollowTurnBurstAuthorization? = nil) async -> RoverCommandDiagnosticResult {
        let data: Data
        do { data = try JSONSerialization.data(withJSONObject: payload, options: [.sortedKeys]) }
        catch { return .init(receipt: .unknown, failure: error) }
        guard let json = String(data: data, encoding: .utf8) else {
            return .init(receipt: .unknown, failure: RoverControlError.encodingFailed)
        }
        var comps = URLComponents(url: baseURL.appendingPathComponent(RoverConfig.jsonCommandPath),
                                  resolvingAgainstBaseURL: false)!
        comps.queryItems = [URLQueryItem(name: "json", value: json)]
        guard let url = comps.url else { return .init(receipt: .unknown, failure: RoverControlError.encodingFailed) }

        var req = URLRequest(url: url)
        req.httpMethod = "GET"
        req.timeoutInterval = RoverConfig.commsWatchdogTimeout
        req.cachePolicy = .reloadIgnoringLocalCacheData
        req.setValue("close", forHTTPHeaderField: "Connection")
        req.setValue("no-cache", forHTTPHeaderField: "Cache-Control")

        let attempts = max(1, RoverConfig.commandRetryAttempts)
        var lastError: Error?
        var lastStatusCode: Int?

        for attempt in 1...attempts {
            var requestEntry: TimeInterval?
            if let authorization {
                authorization.transportCapture?.recordTiming(.init(boundary: .authorizationStart,
                    uptime: authorization.uptime(), attempt: attempt))
                let attemptAuthorized = await authorization.authorizeAttempt?() ?? true
                authorization.transportCapture?.recordTiming(.init(boundary: .authorizationEnd,
                    uptime: authorization.uptime(), attempt: attempt))
                let eligibilityTime = authorization.uptime()
                let time = authorization.uptime() // Last provider read, immediately before final request gates.
                // Only fixed, locked bookkeeping follows the gates. No injected
                // callbacks, provider reads, logging or suspension before HTTP.
                if Task.isCancelled {
                    authorization.transportCapture?.recordTiming(.init(boundary: .eligibility,
                        uptime: time, attempt: attempt, eligible: false))
                    return .init(receipt: .init(httpStatus: lastStatusCode, acknowledged: false,
                        acknowledgementUTC: nil, attempts: attempt - 1, outcome: "cancelled"),
                        failure: FollowTurnBurstTransportDenial.cancelled)
                }
                if !time.isFinite || !authorization.deadline.isFinite || time >= authorization.deadline {
                    authorization.transportCapture?.recordTiming(.init(boundary: .eligibility,
                        uptime: time, attempt: attempt, eligible: false))
                    return .init(receipt: .init(httpStatus: lastStatusCode, acknowledged: false,
                        acknowledgementUTC: nil, attempts: attempt - 1, outcome: "expired"),
                        failure: FollowTurnBurstTransportDenial.expired)
                }
                if !attemptAuthorized || !authorization.isAuthorized() {
                    // Prepared authority samples its clock inside the locked
                    // predicate. Preserve expiry when that final sample crosses
                    // the immutable budget after the earlier eligibility read.
                    let deniedAt = authorization.uptime()
                    let expired = !deniedAt.isFinite || deniedAt >= authorization.deadline
                    authorization.transportCapture?.recordTiming(.init(boundary: .eligibility,
                        uptime: deniedAt, attempt: attempt, eligible: false))
                    return .init(receipt: .init(httpStatus: lastStatusCode, acknowledged: false,
                        acknowledgementUTC: nil, attempts: attempt - 1, outcome: expired ? "expired" : "fenced"),
                        failure: expired ? FollowTurnBurstTransportDenial.expired : .fenced)
                }
                authorization.transportCapture?.recordTiming(.init(boundary: .eligibility,
                    uptime: eligibilityTime, attempt: attempt, eligible: true))
                authorization.transportCapture?.recordTiming(.init(boundary: .requestStart,
                    uptime: time, attempt: attempt))
                requestEntry = time
            }
            var didLogResponse = false
            var statusCode: Int?
            var recordedEntry = false
            func recordActualAttempt() {
                guard !recordedEntry, let requestEntry, let authorization else { return }
                recordedEntry = true
                // The attempt actually entered URLSession. Notify injected
                // observers only after its drain, never inside final eligibility.
                authorization.didEnterAttempt?(.init(operationID: authorization.operationID,
                    attempt: attempt, entryUptime: requestEntry))
            }
            do {
                let (_, response) = try await session.data(for: req)
                recordActualAttempt()
                guard let http = response as? HTTPURLResponse else {
                    RuntimeFileLog.append("rover_command_request", fields: Self.requestLogFields(url: url,
                                                                                                  attempt: attempt,
                                                                                                  maxAttempts: attempts,
                                                                                                  statusCode: nil,
                                                                                                  error: RoverControlError.invalidResponse))
                    didLogResponse = true
                    throw RoverControlError.invalidResponse
                }
                RuntimeFileLog.append("rover_command_request", fields: Self.requestLogFields(url: url,
                                                                                              attempt: attempt,
                                                                                              maxAttempts: attempts,
                                                                                              statusCode: http.statusCode,
                                                                                              error: nil))
                didLogResponse = true
                statusCode = http.statusCode
                lastStatusCode = http.statusCode
                guard (200...299).contains(http.statusCode) else {
                    throw RoverControlError.serverError(http.statusCode)
                }
                lastAckAt = Date()
                return .init(receipt: .init(httpStatus: http.statusCode, acknowledged: true,
                    acknowledgementUTC: lastAckAt, attempts: attempt, outcome: "acknowledged"), failure: nil)
            } catch {
                recordActualAttempt()
                lastError = error
                if !didLogResponse {
                    RuntimeFileLog.append("rover_command_request", fields: Self.requestLogFields(url: url,
                                                                                                  attempt: attempt,
                                                                                                  maxAttempts: attempts,
                                                                                                  statusCode: nil,
                                                                                                  error: error))
                }
                guard attempt < attempts, Self.isRetryableTransportError(error) else {
                    let cancelled = error is CancellationError || (error as? URLError)?.code == .cancelled
                    return .init(receipt: .init(httpStatus: statusCode, acknowledged: false,
                        acknowledgementUTC: nil, attempts: attempt,
                        outcome: cancelled ? "cancelled" : "failed"), failure: error)
                }

                if let authorization {
                    let time = authorization.uptime()
                    let denial: FollowTurnBurstTransportDenial?
                    let outcome: String
                    if Task.isCancelled { denial = .cancelled; outcome = "cancelled" }
                    else if !time.isFinite || !authorization.deadline.isFinite || time >= authorization.deadline {
                        denial = .expired; outcome = "expired"
                    } else if !authorization.isAuthorized() { denial = .fenced; outcome = "fenced" }
                    else { denial = nil; outcome = "unknown" }
                    if let denial {
                        return .init(receipt: .init(httpStatus: statusCode, acknowledged: false,
                            acknowledgementUTC: nil, attempts: attempt, outcome: outcome), failure: denial)
                    }
                }

                RuntimeFileLog.append("rover_command_retry", fields: [
                    "attempt": "\(attempt)",
                    "max": "\(attempts)",
                    "error": error.localizedDescription
                ])
                await retrySleep(RoverConfig.commandRetryBackoff)
            }
        }

        return .init(receipt: .unknown, failure: lastError ?? RoverControlError.invalidResponse)
    }

    static func requestLogFields(url: URL,
                                 attempt: Int,
                                 maxAttempts: Int,
                                 statusCode: Int?,
                                 error: Error?) -> [String: String] {
        var fields: [String: String] = [
            "url": url.absoluteString,
            "attempt": "\(attempt)",
            "max": "\(maxAttempts)"
        ]
        if let statusCode {
            fields["status"] = "\(statusCode)"
        } else if error != nil {
            fields["status"] = "transport_error"
        } else {
            fields["status"] = "unknown"
        }
        if let error {
            fields["error"] = error.localizedDescription
        }
        return fields
    }

    private static func isRetryableTransportError(_ error: Error) -> Bool {
        let nsError = error as NSError
        guard nsError.domain == NSURLErrorDomain else { return false }

        switch URLError.Code(rawValue: nsError.code) {
        case .timedOut,
             .cannotConnectToHost,
             .networkConnectionLost,
             .notConnectedToInternet,
             .cannotFindHost,
             .dnsLookupFailed:
            return true
        default:
            return false
        }
    }
}

public enum RoverControlError: LocalizedError {
    case encodingFailed
    case invalidResponse
    case serverError(Int)

    public var errorDescription: String? {
        switch self {
        case .encodingFailed: return "Failed to encode rover command."
        case .invalidResponse: return "Invalid response from rover."
        case .serverError(let code): return "Rover returned error \(code)."
        }
    }
}
