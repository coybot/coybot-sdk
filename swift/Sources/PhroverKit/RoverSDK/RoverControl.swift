import Foundation
import RoverNav

public enum RoverCommandLinkReadiness: Equatable, Sendable {
    case available
    case unavailable
}

/// Low-level driver for the WAVE ROVER ESP32 over WiFi HTTP.
///
/// Speaks the Waveshare JSON command protocol: commands are sent as
/// `GET /js?json={"T":1,"L":<left>,"R":<right>}`. Left/right are wheel linear velocities (m/s).
public actor RoverControl {
    private let baseURL: URL
    private let session: URLSession

    /// Timestamp of the last successful command; the comms watchdog reads this.
    public private(set) var lastAckAt: Date?

    public init(host: String = RoverConfig.defaultHost, session: URLSession = .shared) {
        self.baseURL = URL(string: "http://\(host)")!
        self.session = session
    }

    // MARK: - Motion

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
        try await sendJSON(["T": RoverConfig.Opcode.speedControl,
                            "L": cmd.right,
                            "R": cmd.left])
    }

    /// Hard stop. Safe to call repeatedly; used by e-stop and the watchdog.
    public func stop() async throws {
        try await sendJSON(["T": RoverConfig.Opcode.emergencyStop])
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
        let data = try JSONSerialization.data(withJSONObject: payload, options: [.sortedKeys])
        guard let json = String(data: data, encoding: .utf8) else {
            throw RoverControlError.encodingFailed
        }
        var comps = URLComponents(url: baseURL.appendingPathComponent(RoverConfig.jsonCommandPath),
                                  resolvingAgainstBaseURL: false)!
        comps.queryItems = [URLQueryItem(name: "json", value: json)]
        guard let url = comps.url else { throw RoverControlError.encodingFailed }

        var req = URLRequest(url: url)
        req.httpMethod = "GET"
        req.timeoutInterval = RoverConfig.commsWatchdogTimeout
        req.cachePolicy = .reloadIgnoringLocalCacheData
        req.setValue("close", forHTTPHeaderField: "Connection")
        req.setValue("no-cache", forHTTPHeaderField: "Cache-Control")

        let attempts = max(1, RoverConfig.commandRetryAttempts)
        var lastError: Error?

        for attempt in 1...attempts {
            var didLogResponse = false
            do {
                let (_, response) = try await session.data(for: req)
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
                guard (200...299).contains(http.statusCode) else {
                    throw RoverControlError.serverError(http.statusCode)
                }
                lastAckAt = Date()
                return
            } catch {
                lastError = error
                if !didLogResponse {
                    RuntimeFileLog.append("rover_command_request", fields: Self.requestLogFields(url: url,
                                                                                                  attempt: attempt,
                                                                                                  maxAttempts: attempts,
                                                                                                  statusCode: nil,
                                                                                                  error: error))
                }
                guard attempt < attempts, Self.isRetryableTransportError(error) else {
                    throw error
                }

                RuntimeFileLog.append("rover_command_retry", fields: [
                    "attempt": "\(attempt)",
                    "max": "\(attempts)",
                    "error": error.localizedDescription
                ])
                try? await Task.sleep(for: .seconds(RoverConfig.commandRetryBackoff))
            }
        }

        throw lastError ?? RoverControlError.invalidResponse
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
