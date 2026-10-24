import Foundation
import RoverNav

/// Pure, conservative spatial association; no person identity is inferred.
public struct FollowTargetTracker {
    public let configuration: FollowMeConfiguration

    public init(configuration: FollowMeConfiguration = FollowMeConfiguration()) {
        self.configuration = configuration
    }

    func selectInitialEvaluated(_ people: [FollowPersonObservation], now: TimeInterval, frameID: ARFrameID? = nil)
        -> (decision: FollowPersonObservation?, evaluation: FollowAssociationEvaluation) {
        var evidence: [FollowCandidateEvidence] = []
        let indices = people.indices.filter { index in
            var candidate = FollowCandidateEvidence()
            var accepted = eligible(people[index], now: now, evidence: &candidate)
            if let frameID, people[index].frameID != frameID {
                candidate.rejection = "candidate_frame_mismatch"
                accepted = false
            }
            evidence.append(candidate)
            return accepted
        }
        let selected = indices.min { a, b in
            let ax = people[a].boundingBox.midX - 0.5
            let ay = people[a].boundingBox.midY - 0.5
            let bx = people[b].boundingBox.midX - 0.5
            let by = people[b].boundingBox.midY - 0.5
            return ax * ax + ay * ay < bx * bx + by * by
        }
        return (selected.map { people[$0] }, evaluation(people, evidence: evidence, mode: "initial",
            outcome: selected == nil ? "lost" : "initial", selected: selected, matchedCount: nil))
    }

    public func selectInitial(_ people: [FollowPersonObservation], now: TimeInterval) -> FollowPersonObservation? {
        selectInitialEvaluated(people, now: now).decision
    }

    public func continueTrack(_ people: [FollowPersonObservation], previous: FollowPersonObservation,
                              predictedPosition: Vec2, now: TimeInterval) -> FollowTrackMatch {
        continueTrackEvaluated(people, previous: previous, predictedPosition: predictedPosition, now: now).decision
    }

    /// `worldGate` overrides the fixed world-distance gate (a coasting track's gate widens with
    /// its uncertainty); `requireScreenAssociation: false` drops the box-overlap test, which only
    /// means something against a box from the previous frame, not one seen seconds ago.
    func continueTrackEvaluated(_ people: [FollowPersonObservation], previous: FollowPersonObservation,
                                predictedPosition: Vec2, now: TimeInterval, frameID: ARFrameID? = nil,
                                worldGate: Double? = nil, requireScreenAssociation: Bool = true)
        -> (decision: FollowTrackMatch, evaluation: FollowAssociationEvaluation) {
        evaluateMatches(people, now: now, mode: "continuity") { candidate, evidence in
            guard candidate.frameID.generation == previous.frameID.generation else {
                evidence.rejection = "frame_generation_mismatch"; return false
            }
            if let frameID, candidate.frameID != frameID {
                evidence.rejection = "candidate_frame_mismatch"; return false
            }
            let distance = candidate.position.distance(to: predictedPosition)
            evidence.record("world_distance_m", distance)
            guard distance <= (worldGate ?? configuration.maximumWorldDistance) else {
                evidence.rejection = "world_distance_exceeded"; return false
            }
            if !requireScreenAssociation { return true }
            let iou = boxIoU(candidate.boundingBox, previous.boundingBox)
            evidence.record("box_iou", Double(iou))
            if iou >= configuration.minimumBoxIoU { return true }
            let dx = candidate.boundingBox.midX - previous.boundingBox.midX
            let dy = candidate.boundingBox.midY - previous.boundingBox.midY
            let displacement = hypot(dx, dy)
            evidence.record("screen_displacement", Double(displacement))
            guard displacement <= configuration.maximumScreenCenterDistance else {
                evidence.rejection = "screen_association_rejected"; return false
            }
            return true
        }
    }

    func reacquireEvaluated(_ people: [FollowPersonObservation], lastPosition: Vec2, now: TimeInterval,
                            expectedGeneration: UInt64? = nil, frameID: ARFrameID? = nil)
        -> (decision: FollowTrackMatch, evaluation: FollowAssociationEvaluation) {
        evaluateMatches(people, now: now, mode: "reacquisition") { candidate, evidence in
            if let expectedGeneration, candidate.frameID.generation != expectedGeneration {
                evidence.rejection = "frame_generation_mismatch"; return false
            }
            if let frameID, candidate.frameID != frameID {
                evidence.rejection = "candidate_frame_mismatch"; return false
            }
            let distance = candidate.position.distance(to: lastPosition)
            evidence.record("reacquisition_distance_m", distance)
            guard distance <= configuration.reacquisitionDistance else {
                evidence.rejection = "reacquisition_distance_exceeded"; return false
            }
            return true
        }
    }

    public func reacquire(_ people: [FollowPersonObservation], lastPosition: Vec2,
                          now: TimeInterval) -> FollowTrackMatch {
        reacquireEvaluated(people, lastPosition: lastPosition, now: now).decision
    }

    public func standOffGoal(rover: Vec2, person: Vec2) -> Vec2? {
        let dx = person.x - rover.x
        let dy = person.y - rover.y
        let distance = hypot(dx, dy)
        guard distance.isFinite, distance > configuration.maximumHoldDistance else { return nil }
        let travel = distance - configuration.standOffDistance
        return Vec2(rover.x + dx / distance * travel, rover.y + dy / distance * travel)
    }

    private func evaluateMatches(_ people: [FollowPersonObservation], now: TimeInterval, mode: String,
                                 gate: (FollowPersonObservation, inout FollowCandidateEvidence) -> Bool)
        -> (decision: FollowTrackMatch, evaluation: FollowAssociationEvaluation) {
        var evidence: [FollowCandidateEvidence] = []
        var matches: [Int] = []
        for index in people.indices {
            var candidate = FollowCandidateEvidence()
            if eligible(people[index], now: now, evidence: &candidate) {
                candidate.matched = gate(people[index], &candidate)
                if candidate.matched == true { matches.append(index) }
            }
            evidence.append(candidate)
        }
        let selected = matches.count == 1 ? matches[0] : nil
        let outcome = matches.isEmpty ? "lost" : matches.count > 1 ? "ambiguous" :
            mode == "continuity" ? "continued" : "reacquired"
        return (match(matches.map { people[$0] }), evaluation(people, evidence: evidence, mode: mode,
            outcome: outcome, selected: selected, matchedCount: matches.count))
    }

    private func eligible(_ observation: FollowPersonObservation, now: TimeInterval,
                          evidence: inout FollowCandidateEvidence) -> Bool {
        let box = observation.boundingBox
        evidence.record("confidence", Double(observation.confidence))
        guard observation.confidence.isFinite else { evidence.rejection = "confidence_nonfinite"; return false }
        guard observation.confidence >= configuration.minimumConfidence else {
            evidence.rejection = "confidence_below_minimum"; return false
        }
        let age = now - observation.timestamp
        evidence.record("observation_age_s", age)
        guard age.isFinite else { evidence.rejection = "observation_age_nonfinite"; return false }
        guard age >= 0 else { evidence.rejection = "observation_from_future"; return false }
        guard age <= configuration.maximumObservationAge else { evidence.rejection = "observation_stale"; return false }
        let finite = observation.position.x.isFinite && observation.position.y.isFinite &&
            observation.pose.position.x.isFinite && observation.pose.position.y.isFinite &&
            observation.pose.yaw.isFinite &&
            box.minX.isFinite && box.minY.isFinite && box.maxX.isFinite && box.maxY.isFinite &&
            box.width > 0 && box.height > 0
        evidence.metrics["finite_geometry"] = .bool(finite)
        evidence.metrics["finite_geometry_availability"] = .string("available")
        guard finite else { evidence.rejection = "invalid_geometry"; return false }
        let visible = observation.bodyVerified
            ? box.minX >= 0 && box.minY > 0 && box.maxX <= 1 && box.maxY <= 1
            : box.minX > 0 && box.minY > 0 && box.maxX < 1 && box.maxY < 1
        guard visible else {
            evidence.rejection = "clipped_box"; return false
        }
        evidence.eligible = true
        return true
    }

    private func evaluation(_ people: [FollowPersonObservation], evidence: [FollowCandidateEvidence],
                            mode: String, outcome: String, selected: Int?, matchedCount: Int?) -> FollowAssociationEvaluation {
        .init(mode: mode, outcome: outcome,
              candidates: people.indices.map { evidence[$0].payload(people[$0], index: $0) },
              selectedIndex: selected, eligibleCount: evidence.filter(\.eligible).count, matchedCount: matchedCount,
              thresholds: ["minimum_confidence": .number(Double(configuration.minimumConfidence)),
                "maximum_observation_age_s": .number(configuration.maximumObservationAge),
                "maximum_world_distance_m": .number(configuration.maximumWorldDistance),
                "minimum_box_iou": .number(Double(configuration.minimumBoxIoU)),
                "maximum_screen_center_distance": .number(Double(configuration.maximumScreenCenterDistance)),
                "reacquisition_distance_m": .number(configuration.reacquisitionDistance)])
    }

    private func boxIoU(_ a: CGRect, _ b: CGRect) -> CGFloat {
        let intersection = a.intersection(b)
        guard !intersection.isNull else { return 0 }
        let intersectionArea = intersection.width * intersection.height
        let union = a.width * a.height + b.width * b.height - intersectionArea
        return union > 0 ? intersectionArea / union : 0
    }

    private func match(_ candidates: [FollowPersonObservation]) -> FollowTrackMatch {
        switch candidates.count {
        case 0: return .lost
        case 1: return .matched(candidates[0])
        default: return .ambiguous
        }
    }
}
