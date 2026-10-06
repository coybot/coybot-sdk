import XCTest
import CryptoKit
@testable import PhroverKit

final class OpticalMessageCodecTests: XCTestCase {
    private let missionID = UUID(uuidString: "00000000-0000-0000-0000-000000000001")!
    private let offerTime: Int64 = 1_786_406_400_000

    func testGoldenOfferHasExactCanonicalBytesAndChecksum() throws {
        let message = OpticalMessage(
            missionID: missionID,
            kind: .offer,
            sequence: 1,
            role: .a,
            markerID: "SILENT_SEARCH_01",
            timestampMilliseconds: offerTime,
            body: .offer(OfferBody(
                searchDurationSeconds: 180,
                centerHalfWidthMillimeters: 250,
                targetLabel: "chair",
                markerWidthMillimeters: 200,
                roverARendezvous: OpticalPose(x: -600, y: -800, headingMillidegrees: 0),
                roverBRendezvous: OpticalPose(x: 600, y: -800, headingMillidegrees: 0)
            ))
        )

        let encoded = try OpticalMessageCodec().encode(message)

        XCTAssertEqual(
            String(decoding: encoded, as: UTF8.self),
            #"{"b":{"d":180,"e":250,"n":"chair","ra":{"h":0,"x":-600,"y":-800},"rb":{"h":0,"x":600,"y":-800},"w":200},"c":"9141f298cb2bbd51adbe56cb93861b7b","i":"00000000-0000-0000-0000-000000000001","k":"offer","m":"SILENT_SEARCH_01","q":1,"r":"a","t":1786406400000,"v":1}"#
        )
        XCTAssertEqual(
            OpticalMessageCodec().messageLinkHash(for: encoded),
            "6770a79db857e4cc35d1bc1e7643deec17935bbfdc1165422da2fc8d3439a049"
        )
    }

    func testAllEightTypedKindsRoundTripWithExactBodyKeys() throws {
        let hashA = String(repeating: "a", count: 64)
        let hashB = String(repeating: "b", count: 64)
        let messages: [(OpticalMessage, String)] = [
            (message(.offer, .offer(OfferBody(searchDurationSeconds: 30, centerHalfWidthMillimeters: -250,
                targetLabel: "fire_extinguisher", markerWidthMillimeters: 200,
                roverARendezvous: OpticalPose(x: .min, y: .max, headingMillidegrees: -180_000),
                roverBRendezvous: OpticalPose(x: 600, y: -800, headingMillidegrees: 179_999)))),
             #""b":{"d":30,"e":-250,"n":"fire_extinguisher","ra":{"h":-180000,"x":-2147483648,"y":2147483647},"rb":{"h":179999,"x":600,"y":-800},"w":200}"#),
            (message(.accept, .accept(AcceptBody(offerHash: hashA, roverBWallTimeMilliseconds: offerTime)), role: .b),
             #""b":{"h":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","u":1786406400000}"#),
            (message(.searchCommit, .searchCommit(SearchCommitBody(deadlineMilliseconds: offerTime + 180_000,
                acceptanceHash: hashB, startMilliseconds: offerTime + 30_000))),
             #""b":{"d":1786406580000,"h":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","s":1786406430000}"#),
            (message(.searchAck, .searchAck(HashAcknowledgementBody(hash: hashA)), role: .b),
             #""b":{"h":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}"#),
            (message(.status, .status(StatusBody(found: true, label: "chair", x: -123, y: 456,
                confidenceBasisPoints: 10_000, sampleCount: .max))),
             #""b":{"c":10000,"f":true,"n":"chair","x":-123,"y":456,"z":65535}"#),
            (message(.decision, .decision(DecisionBody(roverAStatusHash: hashA, roverBStatusHash: hashB,
                outcome: .found, x: 10, y: 20))),
             #""b":{"a":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","b":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","o":"found","x":10,"y":20}"#),
            (message(.converge, .converge(ConvergeBody(decisionHash: hashA,
                releaseMilliseconds: offerTime + 30_000, x: 10, y: 20))),
             #""b":{"h":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","s":1786406430000,"x":10,"y":20}"#),
            (message(.convergeAck, .convergeAck(HashAcknowledgementBody(hash: hashB)), role: .b),
             #""b":{"h":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"}"#)
        ]

        for (message, bodyFragment) in messages {
            let encoded = try OpticalMessageCodec().encode(message)
            XCTAssertTrue(String(decoding: encoded, as: UTF8.self).contains(bodyFragment), message.kind.rawValue)
            XCTAssertEqual(try OpticalMessageCodec().decode(encoded, nowMilliseconds: offerTime), message)
            XCTAssertLessThanOrEqual(encoded.count, OpticalMessageCodec.maximumPayloadBytes)
        }
    }

    func testFoundAndNotFoundStatusDecisionAndConvergenceSchemas() throws {
        let hash = String(repeating: "c", count: 64)
        let bodies: [(OpticalMessageKind, OpticalMessageBody, RoverRole)] = [
            (.status, .status(StatusBody(found: false)), .a),
            (.status, .status(StatusBody(found: false, previousStatusHash: hash)), .b),
            (.decision, .decision(DecisionBody(roverAStatusHash: hash, roverBStatusHash: hash, outcome: .notFound)), .a),
            (.decision, .decision(DecisionBody(roverAStatusHash: hash, roverBStatusHash: hash, outcome: .conflict)), .a),
            (.converge, .converge(ConvergeBody(decisionHash: hash, releaseMilliseconds: offerTime + 30_000)), .a)
        ]
        for (kind, body, role) in bodies {
            let value = message(kind, body, role: role)
            let encoded = try OpticalMessageCodec().encode(value)
            XCTAssertEqual(try OpticalMessageCodec().decode(encoded, nowMilliseconds: offerTime), value)
        }
    }

    func testRejectsBodyCombinationsForbiddenBySchema() {
        let hash = String(repeating: "d", count: 64)
        assertEncodeError(.invalidBody, message(.status, .status(StatusBody(found: false, label: "chair"))))
        assertEncodeError(.invalidBody, message(.status, .status(StatusBody(found: false)), role: .b))
        assertEncodeError(.invalidBody, message(.status, .status(StatusBody(found: false,
            previousStatusHash: hash)), role: .a))
        assertEncodeError(.invalidBody, message(.status, .status(StatusBody(found: true, label: "chair", x: 1, y: 2,
            confidenceBasisPoints: 10_001, sampleCount: 3))))
        assertEncodeError(.invalidBody, message(.decision, .decision(DecisionBody(roverAStatusHash: hash,
            roverBStatusHash: hash, outcome: .found))))
        assertEncodeError(.invalidBody, message(.decision, .decision(DecisionBody(roverAStatusHash: hash,
            roverBStatusHash: hash, outcome: .conflict, x: 1, y: 2))))
        assertEncodeError(.invalidBody, message(.converge, .converge(ConvergeBody(decisionHash: hash,
            releaseMilliseconds: offerTime, x: 1))))
        assertEncodeError(.invalidBody, message(.accept, .offer(OfferBody(searchDurationSeconds: 180,
            centerHalfWidthMillimeters: 250, targetLabel: "chair", markerWidthMillimeters: 200,
            roverARendezvous: OpticalPose(x: 0, y: 0, headingMillidegrees: 0),
            roverBRendezvous: OpticalPose(x: 0, y: 0, headingMillidegrees: 0)))))
    }

    func testRejectsMalformedTamperedUnknownAndNoncanonicalPayloads() throws {
        let codec = OpticalMessageCodec()
        let encoded = try codec.encode(message(.searchAck,
            .searchAck(HashAcknowledgementBody(hash: String(repeating: "a", count: 64))), role: .b))
        let tampered = Data(String(decoding: encoded, as: UTF8.self)
            .replacingOccurrences(of: String(repeating: "a", count: 64),
                                  with: String(repeating: "a", count: 63) + "b").utf8)
        XCTAssertThrowsError(try codec.decode(tampered, nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .invalidChecksum)
        }
        XCTAssertThrowsError(try codec.decode(Data([0xff]), nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .malformedUTF8)
        }
        XCTAssertThrowsError(try codec.decode(Data("{".utf8), nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .malformedJSON)
        }
        XCTAssertThrowsError(try codec.decode(Data("[]".utf8), nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .invalidEnvelope)
        }

        let unknown = try signedPayload(changing: encoded) { envelope in envelope["extra"] = 1 }
        XCTAssertThrowsError(try codec.decode(unknown, nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .invalidEnvelope)
        }
        let unknownBody = try signedPayload(changing: encoded) { envelope in
            var body = envelope["b"] as! [String: Any]
            body["extra"] = 1
            envelope["b"] = body
        }
        XCTAssertThrowsError(try codec.decode(unknownBody, nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .invalidBody)
        }
        let noncanonical = Data(String(decoding: encoded, as: UTF8.self).replacingOccurrences(of: #"{"b""#, with: #"{ "b""#).utf8)
        XCTAssertThrowsError(try codec.decode(noncanonical, nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .noncanonicalJSON)
        }
    }

    func testRejectsUnsupportedVersionInvalidIntegerRangesAndNull() throws {
        assertEncodeError(.unsupportedVersion, OpticalMessage(version: 2, missionID: missionID, kind: .searchAck,
            sequence: 1, role: .b, markerID: "SILENT_SEARCH_01", timestampMilliseconds: offerTime,
            body: .searchAck(HashAcknowledgementBody(hash: String(repeating: "a", count: 64)))))
        assertEncodeError(.invalidEnvelope, OpticalMessage(missionID: missionID, kind: .searchAck, sequence: 0,
            role: .b, markerID: "SILENT_SEARCH_01", timestampMilliseconds: offerTime,
            body: .searchAck(HashAcknowledgementBody(hash: String(repeating: "a", count: 64)))))
        let encoded = try OpticalMessageCodec().encode(message(.searchAck,
            .searchAck(HashAcknowledgementBody(hash: String(repeating: "a", count: 64))), role: .b))
        let overflow = try signedPayload(changing: encoded) { $0["q"] = "18446744073709551616" }
        XCTAssertThrowsError(try OpticalMessageCodec().decode(overflow, nowMilliseconds: offerTime))
        let null = try signedPayload(changing: encoded) { envelope in
            var body = envelope["b"] as! [String: Any]
            body["h"] = NSNull()
            envelope["b"] = body
        }
        XCTAssertThrowsError(try OpticalMessageCodec().decode(null, nowMilliseconds: offerTime)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .invalidBody)
        }
    }

    func testTimestampValidityIncludesExactBoundaries() throws {
        let codec = OpticalMessageCodec()
        let encoded = try codec.encode(message(.searchAck,
            .searchAck(HashAcknowledgementBody(hash: String(repeating: "a", count: 64))), role: .b))
        XCTAssertNoThrow(try codec.decode(encoded, nowMilliseconds: offerTime + 120_000))
        XCTAssertNoThrow(try codec.decode(encoded, nowMilliseconds: offerTime - 5_000))
        XCTAssertThrowsError(try codec.decode(encoded, nowMilliseconds: offerTime + 120_001)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .stale)
        }
        XCTAssertThrowsError(try codec.decode(encoded, nowMilliseconds: offerTime - 5_001)) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, .future)
        }
    }

    private func message(_ kind: OpticalMessageKind, _ body: OpticalMessageBody,
                         role: RoverRole = .a, sequence: UInt64 = 1) -> OpticalMessage {
        OpticalMessage(missionID: missionID, kind: kind, sequence: sequence, role: role,
                       markerID: "SILENT_SEARCH_01", timestampMilliseconds: offerTime, body: body)
    }

    private func assertEncodeError(_ expected: OpticalMessageCodecError, _ message: OpticalMessage,
                                   file: StaticString = #filePath, line: UInt = #line) {
        XCTAssertThrowsError(try OpticalMessageCodec().encode(message), file: file, line: line) {
            XCTAssertEqual($0 as? OpticalMessageCodecError, expected, file: file, line: line)
        }
    }

    private func signedPayload(changing data: Data,
                               _ change: (inout [String: Any]) -> Void) throws -> Data {
        var envelope = try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: Any])
        change(&envelope)
        envelope.removeValue(forKey: "c")
        let unsigned = try JSONSerialization.data(withJSONObject: envelope, options: [.sortedKeys])
        envelope["c"] = SHA256.hash(data: unsigned).prefix(16).map { String(format: "%02x", $0) }.joined()
        return try JSONSerialization.data(withJSONObject: envelope, options: [.sortedKeys])
    }
}
