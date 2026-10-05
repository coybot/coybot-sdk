# Apple-Intelligence-Primary Offline Object Missions

**Date:** 2026-08-09

## Goal

Keep Apple Intelligence as Phrover's primary mission brain while ensuring supported object-navigation missions remain operational when the on-device language model is temporarily unavailable. Enrich local detections with color evidence so commands such as "go to the black chair in the other room and come back" can be executed offline.

## Product Behavior

- `Go to the refrigerator` navigates to a confidently detected refrigerator, stops approximately 0.30 m in front of it, and remains there.
- `Go to the refrigerator and come back` performs the same outbound mission, then returns to the pose where that command began.
- `Go to the black chair in the other room` searches the current room, traverses safe unexplored openings when necessary, stops at a locally confirmed black chair, and remains there.
- A return leg runs only when the utterance explicitly contains a supported return phrase such as `come back`, `go back`, or `return`.
- Cloud vision can improve interpretation and grounding when configured, but none of the behaviors above require cloud access.
- Emergency stop and depth safety remain authoritative over every brain, parser, search, and return decision.

## Architecture Decision

Apple Intelligence remains the primary planner. `MissionAgent` first asks the on-device brain for the next action. When configured and online, cloud vision is an optional second-stage enhancement after the on-device brain cannot produce a usable decision. A deterministic offline intent parser and object-mission executor are the final fallback when the on-device model is unavailable, times out before its first decision, or returns an availability error and no cloud enhancement succeeds. The fallback is deliberately limited to object-navigation missions; unsupported commands fail with a specific explanation instead of pretending to understand them.

Visual understanding is split from mission reasoning. RoverYOLO supplies object category, confidence, bounding box, and center point. A local attribute analyzer examines pixels inside the bounding box and adds color evidence. Apple Intelligence receives those enriched observations as text, while the fallback executor consumes the same structured observations directly.

## Components

### Foundation Model Availability

`OnDeviceBrain` exposes structured availability instead of only a Boolean:

```swift
public enum OnDeviceBrainAvailability: Equatable, Sendable {
    case available
    case deviceNotEligible
    case appleIntelligenceNotEnabled
    case modelNotReady
}
```

The app logs `on_device_brain_availability` at Talk initialization and whenever a mission encounters an unavailable model. The Talk screen shows a concise diagnostic. `modelNotReady` tells the operator to keep the phone on Wi-Fi and power; `appleIntelligenceNotEnabled` directs them to Apple Intelligence & Siri settings.

### Offline Mission Intent

`OfflineObjectMissionIntent` represents the bounded fallback contract:

```swift
public struct OfflineObjectMissionIntent: Equatable, Sendable {
    public let objectQuery: String
    public let targetLabel: String
    public let requestedColors: Set<LocalObjectColor>
    public let searchOtherRooms: Bool
    public let shouldReturn: Bool
}
```

`OfflineObjectMissionIntentParser` recognizes navigation verbs and extracts a general noun phrase. It does not contain an object-name allowlist. It normalizes known aliases already supported by mission grounding, including `fridge` to `refrigerator`, strips location phrases such as `in the other room`, and strips return phrases after recording `shouldReturn`.

The parser rejects blank targets, non-navigation commands, and multi-action requests outside this contract. Apple Intelligence continues to handle unrestricted natural-language requests when available.

### Enriched Perception

`PerceivedObject` gains an optional normalized bounding box and local color evidence while preserving its existing initializer for source compatibility:

```swift
public struct PerceivedObject: Equatable, Sendable {
    public var label: String
    public var confidence: Float
    public var normalizedPoint: CGPoint
    public var normalizedBoundingBox: CGRect?
    public var colorEvidence: [ObjectColorEvidence]
}
```

`LocalObjectColor` initially supports `black`, `white`, `gray`, `red`, `orange`, `yellow`, `green`, `blue`, `purple`, and `brown`. `LocalObjectColorAnalyzer` samples an inset region of each detector bounding box, ignores very dark edge and background outliers, and converts sampled pixels into luminance, saturation, and hue evidence. Black requires low luminance across a majority of valid samples; chromatic colors require sufficient saturation plus a matching hue interval.

Category confidence and color confidence are independent. A visual target matches only when:

- RoverYOLO category confidence is at least `0.90`.
- The canonical object label matches the requested target label.
- Every requested color has local confidence at or above `0.70`.

Commands without a requested color continue to match only on category confidence.

### Apple Intelligence Context

`OnDeviceBrain.promptText` includes enriched descriptions such as:

```text
Visible now: black chair (96% object confidence, 82% color confidence)
```

The model still decides the plan and next action. Local perception, not the text model, is the source of truth for whether a visual target satisfies category, confidence, and color constraints.

### Brain Selection

The current cloud-first `HybridBrain` order changes to:

1. Apple Intelligence on-device brain.
2. Configured cloud brain only when the on-device brain is unavailable, times out, or rejects the request as unsupported.
3. Deterministic offline object fallback when neither brain produces a usable first decision.

Every selection is logged through `mission_brain_selected` with `brain=on_device`, `brain=cloud`, or `brain=offline_object_fallback` and a structured reason. A cloud decision remains subject to the same local grounding and safety checks as an on-device decision.

### Offline Object Mission Executor

When neither Apple Intelligence nor the optional cloud enhancement produces a usable first decision, `MissionAgent` asks the fallback parser for an `OfflineObjectMissionIntent`. If parsing succeeds, a bounded object mission runs without another brain call:

1. Record the command start pose, start room, and session generation.
2. Check the latest enriched detections.
3. If the target is visible and grounded, navigate to its LiDAR-unprojected point using the existing visual-target stop clearance.
4. If it is absent, rotate using existing slow scan turns of 20-30 degrees and wait for a camera frame newer than the pre-turn frame before evaluating detections.
5. After one bounded local scan, select the highest-ranked safe unexplored opening.
6. Traverse the opening, mark it visited, rescan, and continue until the target is found or the configured room/opening budget is exhausted.
7. Stop approximately 0.30 m from the confirmed target.
8. If `shouldReturn` is false, finish successfully.
9. If `shouldReturn` is true, execute the recorded return route and finish at the command start pose.

The executor locks `objectQuery` for the complete mission. A changing camera view or a later detection cannot replace the requested target.

### Room Discovery and Return

The mission records each crossed doorway and room in order. `SessionRoomTopology` adds a shortest-doorway-path query over its existing room/doorway graph. The outbound search refuses already searched rooms and previously rejected doorway candidates.

For return, the mission traverses the recorded doorway route in reverse. Once it reaches the starting room, it uses the existing bounded heading alignment and navigation retry logic to reach the original pose. A session-generation change invalidates the route and fails safely because poses and topology no longer share the original session frame.

## Decision Precedence

1. Emergency stop utterance or UI E-stop.
2. Depth, person, tracking, and communication safety.
3. Apple Intelligence decision.
4. Optional cloud enhancement after the on-device brain cannot decide.
5. Offline object fallback after both configured brain stages fail to produce a first decision.
6. Explicit failure for unsupported commands.

Cloud availability never changes whether the documented object missions work offline.

## Failure Handling

- Missing pose: reject before motion.
- Apple Intelligence disabled or model not ready: log exact reason, then attempt the bounded fallback.
- Unsupported fallback utterance: stop and report that Apple Intelligence is unavailable for that command.
- Detector unavailable: stop; category and color targets cannot be safely confirmed.
- No color evidence for a color-qualified target: continue searching; never silently accept an unqualified object.
- No fresh frame after a turn: stop that scan step and retry within the existing bounded budget.
- No safe opening: stop and report search exhaustion.
- Rover transport failure: stop immediately and preserve the failure status.
- Tracking/session reset: stop and invalidate the return route.
- Return doorway or start pose unreachable: stop and report a return failure without claiming mission success.

## Telemetry

Add structured events for:

- `on_device_brain_availability`
- `mission_offline_fallback_started`
- `mission_offline_fallback_rejected`
- `mission_attribute_match`
- `mission_attribute_rejected`
- `mission_room_search_started`
- `mission_room_searched`
- `mission_doorway_crossed`
- `mission_return_route_started`
- `mission_return_route_step`
- `mission_return_route_completed`

Each motion event retains mission ID, pose, goal, distance, wheel command, request URL, and response status through the existing runtime logging path.

## Testing Strategy

All behavior changes use test-first development.

### Unit Tests

- Map every Foundation Models unavailable reason to an app-level diagnostic.
- Parse visible-object, color-qualified, other-room, explicit-return, and no-return commands.
- Reject unsupported fallback commands without motion.
- Classify synthetic black, white, gray, and chromatic image regions.
- Require 90% category confidence and 70% requested-color confidence.
- Preserve the target lock across camera changes.
- Compute forward and reverse doorway paths without cycles.

### Mission Tests

- Use Apple Intelligence output when the primary brain is available.
- Ask the optional cloud brain only after the on-device brain is unavailable, times out, or rejects the request.
- Fall back to the deterministic executor only after the configured brain stages fail to produce a first decision.
- Reach a visible refrigerator and remain there without return language.
- Reach a visible refrigerator and return when explicitly requested.
- Scan slowly and wait for a fresh frame before matching.
- Search a second room for a black chair and stop at the target.
- Search a second room for a black chair and reverse the doorway route when return is requested.
- Stop safely for stale depth, missing pose, transport failure, tracking reset, and exhausted openings.

### Device Verification

- Confirm the device log includes the exact Apple Intelligence availability state.
- Test with Apple Intelligence available and verify `mission_brain_selected` precedes navigation.
- Test with the model unavailable and verify `mission_offline_fallback_started` precedes navigation.
- Test both refrigerator command forms.
- Test black-chair search from another room with airplane mode enabled while remaining connected to the rover Wi-Fi network.
- Pull runtime and brain logs after each mission and verify terminal status, target lock, doorway path, stop distance, and return completion.

## Non-Goals

- Training or bundling a local vision-language model.
- Persisting named-room maps across application launches.
- Recognizing arbitrary material, brand, ownership, or fine-grained visual attributes beyond the local color palette.
- Allowing cloud output to bypass local category, color, LiDAR, or safety validation.
- Automatically returning when the command does not explicitly request it.
