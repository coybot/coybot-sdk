import CryptoKit
import Foundation

public enum OpticalMessageCodecError: Error, Equatable, Sendable {
    case malformedUTF8
    case malformedJSON
    case invalidEnvelope
    case invalidBody
    case unsupportedVersion
    case noncanonicalJSON
    case invalidChecksum
    case stale
    case future
    case payloadTooLarge
}

public struct OpticalMessageCodec: Sendable {
    public static let maximumPayloadBytes = 1_200
    public static let maximumAgeMilliseconds: Int64 = 120_000
    public static let maximumFutureMilliseconds: Int64 = 5_000

    public init() {}

    public func encode(_ message: OpticalMessage) throws -> Data {
        try validate(message)
        var envelope = try envelopeObject(for: message)
        let unsigned = try canonicalData(envelope)
        envelope["c"] = checksum(for: unsigned)
        let data = try canonicalData(envelope)
        guard data.count <= Self.maximumPayloadBytes else { throw OpticalMessageCodecError.payloadTooLarge }
        return data
    }

    public func decode(_ data: Data, nowMilliseconds: Int64) throws -> OpticalMessage {
        let message = try decode(data)
        try validateTimestamp(of: message, nowMilliseconds: nowMilliseconds)
        return message
    }

    public func decode(_ data: Data) throws -> OpticalMessage {
        guard data.count <= Self.maximumPayloadBytes else { throw OpticalMessageCodecError.payloadTooLarge }
        guard String(data: data, encoding: .utf8) != nil else { throw OpticalMessageCodecError.malformedUTF8 }
        let value: Any
        do { value = try JSONSerialization.jsonObject(with: data, options: [.fragmentsAllowed]) }
        catch { throw OpticalMessageCodecError.malformedJSON }
        guard var envelope = value as? [String: Any],
              Set(envelope.keys) == ["b", "c", "i", "k", "m", "q", "r", "t", "v"] else {
            throw OpticalMessageCodecError.invalidEnvelope
        }
        guard let suppliedChecksum = envelope.removeValue(forKey: "c") as? String,
              isHex(suppliedChecksum, length: 32) else { throw OpticalMessageCodecError.invalidEnvelope }
        let unsigned = try canonicalData(envelope)
        guard checksum(for: unsigned) == suppliedChecksum else { throw OpticalMessageCodecError.invalidChecksum }
        envelope["c"] = suppliedChecksum
        guard try canonicalData(envelope) == data else { throw OpticalMessageCodecError.noncanonicalJSON }

        let message = try parse(envelope)
        try validate(message)
        return message
    }

    public func validateTimestamp(of message: OpticalMessage, nowMilliseconds: Int64) throws {
        let age = nowMilliseconds.subtractingReportingOverflow(message.timestampMilliseconds)
        guard !age.overflow else { throw OpticalMessageCodecError.stale }
        if age.partialValue > Self.maximumAgeMilliseconds { throw OpticalMessageCodecError.stale }
        if age.partialValue < -Self.maximumFutureMilliseconds { throw OpticalMessageCodecError.future }
    }

    public func messageLinkHash(for message: OpticalMessage) throws -> String {
        messageLinkHash(for: try encode(message))
    }

    public func messageLinkHash(for canonicalMessage: Data) -> String {
        Self.hex(SHA256.hash(data: canonicalMessage))
    }

    private func validate(_ message: OpticalMessage) throws {
        guard message.version == OpticalMessage.protocolVersion else { throw OpticalMessageCodecError.unsupportedVersion }
        guard message.sequence >= 1,
              matches(message.markerID, pattern: #"^[A-Z0-9_-]{1,24}$"#) else {
            throw OpticalMessageCodecError.invalidEnvelope
        }
        switch (message.kind, message.body) {
        case let (.offer, .offer(body)):
            guard (30...3600).contains(Int(body.searchDurationSeconds)),
                  matches(body.targetLabel, pattern: #"^[a-z0-9_-]{1,40}$"#) else { throw OpticalMessageCodecError.invalidBody }
        case let (.accept, .accept(body)):
            guard validHash(body.offerHash) else { throw OpticalMessageCodecError.invalidBody }
        case let (.searchCommit, .searchCommit(body)):
            guard validHash(body.acceptanceHash) else { throw OpticalMessageCodecError.invalidBody }
        case let (.searchAck, .searchAck(body)), let (.convergeAck, .convergeAck(body)):
            guard validHash(body.hash) else { throw OpticalMessageCodecError.invalidBody }
        case let (.status, .status(body)):
            guard body.confidenceBasisPoints.map({ $0 <= 10_000 }) ?? true,
                  body.previousStatusHash.map(validHash) ?? true,
                  (message.role == .a && body.previousStatusHash == nil) ||
                    (message.role == .b && body.previousStatusHash != nil) else {
                throw OpticalMessageCodecError.invalidBody
            }
            if body.found {
                guard let label = body.label, matches(label, pattern: #"^[a-z0-9_-]{1,40}$"#),
                      body.x != nil, body.y != nil, body.confidenceBasisPoints != nil,
                      let count = body.sampleCount, count > 0 else { throw OpticalMessageCodecError.invalidBody }
            } else if body.label != nil || body.x != nil || body.y != nil ||
                        body.confidenceBasisPoints != nil || body.sampleCount != nil {
                throw OpticalMessageCodecError.invalidBody
            }
        case let (.decision, .decision(body)):
            guard validHash(body.roverAStatusHash), validHash(body.roverBStatusHash) else { throw OpticalMessageCodecError.invalidBody }
            if body.outcome == .found {
                guard body.x != nil, body.y != nil else { throw OpticalMessageCodecError.invalidBody }
            } else if body.x != nil || body.y != nil { throw OpticalMessageCodecError.invalidBody }
        case let (.converge, .converge(body)):
            guard validHash(body.decisionHash), (body.x == nil) == (body.y == nil) else { throw OpticalMessageCodecError.invalidBody }
        default:
            throw OpticalMessageCodecError.invalidBody
        }
    }

    private func envelopeObject(for message: OpticalMessage) throws -> [String: Any] {
        ["b": try bodyObject(message.body), "i": message.missionID.uuidString.lowercased(),
         "k": message.kind.rawValue, "m": message.markerID, "q": message.sequence,
         "r": message.role.rawValue, "t": message.timestampMilliseconds, "v": message.version]
    }

    private func bodyObject(_ body: OpticalMessageBody) throws -> [String: Any] {
        let data: Data
        let encoder = JSONEncoder()
        switch body {
        case let .offer(value): data = try encoder.encode(value)
        case let .accept(value): data = try encoder.encode(value)
        case let .searchCommit(value): data = try encoder.encode(value)
        case let .searchAck(value): data = try encoder.encode(value)
        case let .status(value): data = try encoder.encode(value)
        case let .decision(value): data = try encoder.encode(value)
        case let .converge(value): data = try encoder.encode(value)
        case let .convergeAck(value): data = try encoder.encode(value)
        }
        guard let object = try JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            throw OpticalMessageCodecError.invalidBody
        }
        return object
    }

    private func parse(_ envelope: [String: Any]) throws -> OpticalMessage {
        guard let version = integer(envelope["v"], as: Int.self),
              let missionString = envelope["i"] as? String,
              missionString == missionString.lowercased(), let missionID = UUID(uuidString: missionString),
              let kindString = envelope["k"] as? String, let kind = OpticalMessageKind(rawValue: kindString),
              let sequence = integer(envelope["q"], as: UInt64.self),
              let roleString = envelope["r"] as? String, let role = RoverRole(rawValue: roleString),
              let markerID = envelope["m"] as? String,
              let timestamp = integer(envelope["t"], as: Int64.self),
              let body = envelope["b"] as? [String: Any] else { throw OpticalMessageCodecError.invalidEnvelope }
        if version != OpticalMessage.protocolVersion { throw OpticalMessageCodecError.unsupportedVersion }
        let parsedBody = try parseBody(body, kind: kind, role: role)
        return OpticalMessage(version: version, missionID: missionID, kind: kind, sequence: sequence,
                              role: role, markerID: markerID, timestampMilliseconds: timestamp, body: parsedBody)
    }

    private func parseBody(_ object: [String: Any], kind: OpticalMessageKind, role: RoverRole) throws -> OpticalMessageBody {
        func decoded<T: Decodable>(_ type: T.Type, keys: Set<String>) throws -> T {
            guard Set(object.keys) == keys, !object.values.contains(where: { $0 is NSNull }) else {
                throw OpticalMessageCodecError.invalidBody
            }
            do { return try JSONDecoder().decode(type, from: canonicalData(object)) }
            catch { throw OpticalMessageCodecError.invalidBody }
        }
        switch kind {
        case .offer:
            guard let ra = object["ra"] as? [String: Any], let rb = object["rb"] as? [String: Any],
                  Set(ra.keys) == ["x", "y", "h"], Set(rb.keys) == ["x", "y", "h"] else { throw OpticalMessageCodecError.invalidBody }
            return .offer(try decoded(OfferBody.self, keys: ["d", "e", "n", "w", "ra", "rb"]))
        case .accept: return .accept(try decoded(AcceptBody.self, keys: ["h", "u"]))
        case .searchCommit: return .searchCommit(try decoded(SearchCommitBody.self, keys: ["d", "h", "s"]))
        case .searchAck: return .searchAck(try decoded(HashAcknowledgementBody.self, keys: ["h"]))
        case .status:
            var keys: Set<String> = ["f"]
            if role == .b { keys.insert("p") }
            guard object["f"] is Bool else { throw OpticalMessageCodecError.invalidBody }
            if object["f"] as? Bool == true { keys.formUnion(["n", "x", "y", "c", "z"]) }
            return .status(try decoded(StatusBody.self, keys: keys))
        case .decision:
            var keys: Set<String> = ["a", "b", "o"]
            if object["o"] as? String == OpticalDecisionOutcome.found.rawValue { keys.formUnion(["x", "y"]) }
            return .decision(try decoded(DecisionBody.self, keys: keys))
        case .converge:
            let keys: Set<String> = object["x"] == nil ? ["h", "s"] : ["h", "s", "x", "y"]
            return .converge(try decoded(ConvergeBody.self, keys: keys))
        case .convergeAck: return .convergeAck(try decoded(HashAcknowledgementBody.self, keys: ["h"]))
        }
    }

    private func canonicalData(_ object: Any) throws -> Data {
        guard JSONSerialization.isValidJSONObject(object) else { throw OpticalMessageCodecError.malformedJSON }
        return try JSONSerialization.data(withJSONObject: object, options: [.sortedKeys, .withoutEscapingSlashes])
    }

    private func checksum(for data: Data) -> String { String(Self.hex(SHA256.hash(data: data)).prefix(32)) }
    private func validHash(_ value: String) -> Bool { isHex(value, length: 64) }
    private func isHex(_ value: String, length: Int) -> Bool {
        value.count == length && value.range(of: #"^[0-9a-f]+$"#, options: .regularExpression) != nil
    }
    private func matches(_ value: String, pattern: String) -> Bool {
        value.range(of: pattern, options: .regularExpression) != nil
    }
    private func integer<T: FixedWidthInteger>(_ value: Any?, as: T.Type) -> T? {
        guard let number = value as? NSNumber, CFGetTypeID(number) != CFBooleanGetTypeID() else { return nil }
        let text = number.stringValue
        guard !text.contains("."), !text.lowercased().contains("e") else { return nil }
        return T(text)
    }
    private static func hex<S: Sequence>(_ bytes: S) -> String where S.Element == UInt8 {
        bytes.map { String(format: "%02x", $0) }.joined()
    }
}
