# Session-Local Room Topology MVP Implementation Plan

> **For agentic workers:** Execute task-by-task with TDD. Preserve the pre-existing uncommitted navigation and mission-recovery work; do not reset or overwrite it. Run code review against the approved design before the final commit.

**Goal:** Make standalone room-transition commands deterministically traverse one safe doorway, confirm entry into another session-local room, and stop.

**Architecture:** Add outward frontier geometry in `RoverNav`; keep a topological room graph in a focused `PhroverKit` module; route standalone room-transition intent through a deterministic `MissionAgent` branch; retain `NavigationController` as the sole motion/safety authority; use `PhroverCloud` only for an optional candidate-ranking boost.

**Tech stack:** Swift 6, XCTest, ARKit, Core Motion, SwiftUI, RoverNav, PhroverKit, PhroverCloud, Xcode iOS Simulator.

**Approved design:** `docs/superpowers/specs/2026-07-29-session-local-room-topology-mvp-design.md`

## Global constraints

- Keep the existing uncommitted stale-state reset, fresh-frame scan, inertial-heading, pose-rebase, and visited-opening recovery changes unless a failing test proves a narrower replacement is necessary.
- Do not fold `DoorwayCandidate` into brain-oriented `ExplorationCandidate`; their lifecycle semantics differ.
- Do not let topology send motor commands or bypass navigation safety.
- Geometry must work offline. Cloud evidence may only reorder geometrically viable candidates.
- A room graph changes only after a confirmed doorway-plane crossing.
- A standalone room-transition mission performs at most one 360-degree scan refresh and three candidate attempts.
- Do not add persistence, room names, room polygons, or multi-rover topology.
- Stage only task-owned files and hunks; the working tree contains unrelated changes.

---

### Task 1: Establish a Reproducible iOS SDK Test Gate

**Files:**
- Modify: `examples/PhroverOperator/PhroverOperator.xcodeproj/project.pbxproj`
- Add: `examples/PhroverOperator/PhroverOperator.xcodeproj/xcshareddata/xcschemes/PhroverSDKTests.xcscheme`
- Add: `scripts/test-swift-sdk.sh`
- Modify: `.github/workflows/swift.yml`
- Modify: `Package.swift`
- Modify: `README.md`

**Interfaces:**
- Produces a committed `PhroverSDKTests` scheme exposing hostless `RoverNavTests` and `PhroverKitTests` targets backed by the existing SwiftPM test directories.
- Produces one repository command that selects an available iOS 26+ iPhone simulator and runs both regular suites.

- [ ] **Step 1: Add native unit-test targets to the existing Xcode project**

Create hostless iOS unit-test targets named `RoverNavTests` and `PhroverKitTests`. Use file-system-synchronized groups pointing to:

- `swift/Tests/RoverNavTests`
- `swift/Tests/PhroverKitTests`

Link `RoverNavTests` to the `RoverNav` package product. Link `PhroverKitTests` to `PhroverKit` and `RoverNav`. Set iOS 26.0, Swift 6, generated Info.plists, and no test host.

- [ ] **Step 2: Add the shared SDK test scheme**

Configure `PhroverSDKTests.xcscheme` to build and test only the two regular SDK test targets. Exclude `PhroverOperatorUITests`, `PhroverKitLiveProbes`, and `PhroverSimTests`.

- [ ] **Step 3: Add the wrapper command**

Create `scripts/test-swift-sdk.sh` to:

1. select the first available iOS 26+ iPhone simulator unless `SIM_UDID` is supplied;
2. print only that identifier when invoked with `--print-udid`;
3. run `xcodebuild test` against `PhroverOperator.xcodeproj` and `PhroverSDKTests` otherwise;
4. forward additional `xcodebuild` arguments so focused `-only-testing:` runs remain possible.

- [ ] **Step 4: Replace stale CI and documentation commands**

Update `.github/workflows/swift.yml`, `Package.swift` commentary, and `README.md` to use the committed scheme. Keep `swift build --target RoverNav` as a compile smoke test, not as a test claim.

- [ ] **Step 5: Verify the existing test suites execute**

Run:

```bash
scripts/test-swift-sdk.sh
swift build --target RoverNav
git diff --check
```

Expected: existing `RoverNavTests` and `PhroverKitTests` run on an iOS 26+ simulator; the pure `RoverNav` target builds on macOS; no whitespace errors.

---

### Task 2: Give Frontiers an Outward Direction

**Files:**
- Modify: `swift/Sources/RoverNav/FrontierFinder.swift`
- Modify: `swift/Sources/RoverNav/Geometry.swift` only if shared vector helpers are needed
- Modify: `swift/Tests/RoverNavTests/FrontierFinderTests.swift`

**Interfaces:**
- Extends `Frontier` with `outwardDirection: Vec2?`.
- Preserves source compatibility with `outwardDirection: Vec2? = nil` in the initializer.

- [ ] **Step 1: Write failing direction tests**

Cover a straight observed-to-unknown boundary, a rotated boundary, and a symmetric/degenerate cluster. Assert that valid directions are unit length and point toward unknown cells; degenerate evidence produces `nil` without dropping the frontier.

- [ ] **Step 2: Run the focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:RoverNavTests/FrontierFinderTests
```

Expected: compile or assertion failure because `Frontier` has no outward direction.

- [ ] **Step 3: Derive the aggregate outward vector**

For each frontier member, accumulate vectors to its four-adjacent unobserved neighbors. Normalize the aggregate only when it has non-zero stable support. Keep the existing centroid, width, cell count, and sort behavior.

- [ ] **Step 4: Verify**

```bash
scripts/test-swift-sdk.sh -only-testing:RoverNavTests/FrontierFinderTests
```

Expected: all frontier tests pass.

---

### Task 3: Model Sessions, Rooms, Doorways, and Candidates

**Files:**
- Add: `swift/Sources/PhroverKit/Topology/RoomTopologyModels.swift`
- Add: `swift/Sources/PhroverKit/Topology/SessionRoomTopology.swift`
- Add: `swift/Sources/PhroverKit/Topology/DoorwayEvidenceProviding.swift`
- Add: `swift/Tests/PhroverKitTests/SessionRoomTopologyTests.swift`

**Interfaces:**
- Produces `RoomID`, `DoorwayID`, `DoorwayCandidateID`, `Room`, `Doorway`, `DoorwayCandidate`, `RoomTopologySnapshot`, and `RoomTopologyManaging`.
- `SessionRoomTopology` depends only on Foundation and RoverNav data types.
- `startSession(generation:initialPose:)` and `reset(forSessionGeneration:)` bind the graph to an explicit generation; topology never infers a new generation from an arriving pose.
- `DoorwayEvidenceProviding` accepts a frame plus candidate descriptors and returns candidate boost values in `[0, 1]`.
- Ranking consumes explicit `DoorwayCandidateAssessment` values supplied by the caller (`isReachable`, beyond-plane goal, and path distance); topology never reaches into `NavigationController`.
- Ranking accepts a mission-owned exclusion set rather than storing mission rejection in the session graph.

- [ ] **Step 1: Write failing session and candidate tests**

Assert that `startSession(generation:initialPose:)` binds the graph to the supplied generation and creates `room_1`; poses cannot implicitly start a session; width filtering accepts 0.65–2.0 m; candidates without direction are excluded; nearby observations within 0.5 m and 35 degrees preserve identity; caller-supplied reachability/path assessments drive ranking; mission exclusions remove candidates without mutating session state; reset clears IDs, graph, and pending state.

- [ ] **Step 2: Run the focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

Expected: compile failure because topology types do not exist.

- [ ] **Step 3: Implement the shallow model API and candidate identity**

Keep IDs deterministic and session-local (`room_1`, `doorway_1`, `doorway_candidate_1`). Store only session topology; `MissionAgent` owns the rejection set and passes it into ranking. Include known doorways incident to the current room as selectable candidates even when no frontier remains, orienting each doorway plane toward its opposite room. Emit `room_session_started` and `room_session_reset` through an injected telemetry sink defaulting to `RuntimeFileLog`.

- [ ] **Step 4: Implement deterministic ranking**

Order by unexplored status, beyond-plane goal reachability, geometric quality, optional visual boost, path length, then stable ID. Exclude candidates rejected for the active mission. Do not allow visual evidence to make an invalid candidate viable.

- [ ] **Step 5: Verify**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

Expected: all session, identity, filtering, reset, and ranking tests pass.

---

### Task 4: Confirm Doorway Crossings and Build the Topology Graph

**Files:**
- Modify: `swift/Sources/PhroverKit/Topology/RoomTopologyModels.swift`
- Modify: `swift/Sources/PhroverKit/Topology/SessionRoomTopology.swift`
- Modify: `swift/Tests/PhroverKitTests/SessionRoomTopologyTests.swift`

**Interfaces:**
- Adds `beginTransition`, `observeTransition`, `confirmTransition`, `rejectTransition`, and `abandonTransition`.
- `observeTransition` is read-only with respect to the room graph and returns `readyForConfirmation` only after the geometric threshold is satisfied; `confirmTransition` performs the atomic graph mutation after motion has stopped.
- Adds a plain `TransitionObservation` value in `RoomTopologyModels.swift` carrying pose, frame sequence, timestamp, normal-tracking state, and session generation; observations from any generation other than the topology session are ignored. Task 5 adapts live AR samples into this already-tested type.

- [ ] **Step 1: Write failing crossing tests**

Cover approach without crossing, lateral movement, stale/repeated frames, wrong session generation, abnormal tracking, a one-frame pose jump, three consecutive fresh poses at least 0.35 m beyond the plane, observation without confirmation, explicit confirmation, reverse traversal, duplicate doorway observations, rejection, and abandonment.

- [ ] **Step 2: Observe failures**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

- [ ] **Step 3: Implement signed-distance transition tracking**

Record the approach side and require progression from that side through the plane to `+0.35 m`. Count only newer normal-tracking frames. Reset the confirmation count when a sample falls short. Reject discontinuous samples rather than allowing them to create rooms.

- [ ] **Step 4: Promote confirmed transitions**

Have `observeTransition` return readiness without mutating the graph. After the caller stops motion, `confirmTransition` atomically creates a new room and bidirectional doorway for a first crossing, or selects the known opposite endpoint for an existing doorway, then emits `doorway_crossed` and `room_transition_completed`.

- [ ] **Step 5: Verify**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

Expected: all geometry and graph tests pass.

---

### Task 5: Expose Coherent Pose Samples and AR Session Resets

**Files:**
- Add: `swift/Sources/PhroverKit/Perception/PoseObservation.swift`
- Add: `swift/Sources/PhroverKit/Perception/RoomSessionCoordinator.swift`
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Add: `swift/Tests/PhroverKitTests/ARSessionManagerTests.swift`
- Add: `swift/Tests/PhroverKitTests/RoomSessionCoordinatorTests.swift`
- Modify test fakes in: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Produces one coherent `PoseObservation` with pose, frame sequence, timestamp, tracking quality, and session generation, plus a lossless adapter to topology’s `TransitionObservation`.
- Exposes the latest observation through `RoverPerception` with a backward-compatible default.
- Exposes an explicit AR reset callback or typed lifecycle observer.
- Adds an internal primitive-sample ingestion/reset seam so tests use plain `PoseObservation` values rather than constructing `ARFrame`.
- Produces a testable `RoomSessionCoordinator` that owns the monotonic session generation and orders awaited navigation stop, pending-transition abandonment, `topology.reset(forSessionGeneration:)`, and `ARSessionManager.resetTracking(generation:)` without importing the example app. The same newly allocated generation is passed to both topology and AR state before frames resume. The coordinator observes pose samples and calls `topology.startSession(generation:initialPose:)` exactly once when the first fresh, normal observation for that active generation arrives.

- [ ] **Step 1: Write failing freshness and reset tests**

Through the primitive sample-ingestion seam, assert monotonic freshness within a session, explicit generation change on reset, synchronous clearing of stale pose/frame/depth state, rejection of old-generation samples, and reset notification before new-session samples are accepted. In `RoomSessionCoordinatorTests`, assert awaited stop → abandon → topology reset(new generation) → AR reset(same generation) ordering; then assert that limited, stale, and mismatched samples do not start a room, while the first fresh normal sample for the active generation creates `room_1` exactly once.

- [ ] **Step 2: Observe failure**

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/ARSessionManagerTests \
  -only-testing:PhroverKitTests/RoomSessionCoordinatorTests
```

- [ ] **Step 3: Implement coherent observations**

Publish pose, frame sequence, timestamp, tracking state, and generation together from one AR frame update. Retain the current WIP’s `frameSequence`, `isTrackingNormal`, and `inertialYaw` behavior.

- [ ] **Step 4: Add explicit reset and interruption lifecycle**

Make `start()` delegate to a reset boundary that clears stale AR-derived data before `session.run(...resetTracking...)`. Handle interruption, interruption end, and session failure by suspending fresh-pose use; only an actual reset clears topology.

- [ ] **Step 5: Verify**

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/ARSessionManagerTests \
  -only-testing:PhroverKitTests/RoomSessionCoordinatorTests
```

---

### Task 6: Add Read-Only Goal Assessment and Tracking-Safe Navigation

**Files:**
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`
- Modify: `swift/Tests/PhroverKitTests/RotationCommandTests.swift`

**Interfaces:**
- Adds `NavigationGoalAssessment` containing goal, reachability, and path distance.
- Adds a non-driving `assessGoal(_:)` seam to `RoverMotion`, with a test-friendly default.
- Adds `stopAndWait() async` to `RoverMotion`; `NavigationController` must await the rover stop request before returning. Keep synchronous `cancel()` for emergency callers, but never use it as the stop-before-confirmation guarantee.

- [ ] **Step 1: Write failing assessment and tracking tests**

Assert that assessment does not change navigation state or send commands; unreachable beyond-plane goals are reported; `stopAndWait()` does not return before the stop transport completes; stale/limited tracking stops motion and waits only within a bounded interval; every terminal scan failure sends stop.

- [ ] **Step 2: Observe failure**

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/NavigationSafetyTests \
  -only-testing:PhroverKitTests/RotationCommandTests
```

- [ ] **Step 3: Implement read-only assessment**

Reuse the planner and current costmap without assigning `path`, changing `state`, or starting `loop`. Return path length for ranking.

- [ ] **Step 4: Finish the existing scan recovery work**

Preserve inertial progress, bounded pulse count, one direction reversal, fresh-frame wait, and AR pose rebasing. Make all failure exits stop the rover. A 360-degree room scan must call bounded 30-degree scan steps; never pass `2π` to an API that normalizes angles.

- [ ] **Step 5: Verify**

Run the focused command from Step 2. Expected: assessment, scan, and safety regressions pass.

---

### Task 7: Parse Standalone Room-Transition Intent

**Files:**
- Add: `swift/Sources/PhroverKit/Voice/RoomTransitionIntent.swift`
- Add: `swift/Tests/PhroverKitTests/RoomTransitionIntentTests.swift`

**Interfaces:**
- Produces `RoomTransitionIntent.matches(_:)` as a pure deterministic parser.

- [ ] **Step 1: Write the intent matrix**

Positive cases: “go to the other room,” “move into another room,” and “enter the next room,” including case and punctuation variants.

Negative cases: “go to the chair in the other room,” “find another room’s table,” “stop,” blank input, and unrelated uses of “room.”

- [ ] **Step 2: Observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RoomTransitionIntentTests
```

- [ ] **Step 3: Implement the smallest parser**

Require an approved transition verb and destination phrase. Reject concrete-target modifiers rather than trying to become a general natural-language parser.

- [ ] **Step 4: Verify**

Run the focused command from Step 2. Expected: the complete intent matrix passes.

---

### Task 8: Implement the Happy-Path Room-Transition Mission

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Add: `swift/Tests/PhroverKitTests/RoomTransitionMissionTests.swift`
- Modify shared fakes in: `swift/Tests/PhroverKitTests/MissionAgentTests.swift` if necessary

**Interfaces:**
- Injects `RoomTopologyManaging` and optional `DoorwayEvidenceProviding` into `MissionAgent`.
- Adds an isolated deterministic `runRoomTransitionMission` path.

- [ ] **Step 1: Write the failing happy-path integration test**

Given a stable initial pose, one viable doorway, a reachable beyond-plane goal, and fresh crossing poses, assert:

- the general brain is never called;
- stale non-idle motion is canceled before topology selection;
- exactly one navigation goal is issued;
- awaited motion stop completes before `confirmTransition` is called;
- current room remains `room_1` until stop completion and then changes to `room_2`;
- completion telemetry is emitted.

Also assert an object-target command mentioning another room still calls the existing brain path. Add a second room-transition command from `room_2` where no frontier remains but the known incident doorway is ranked with its normal reversed; confirm return to `room_1`.

- [ ] **Step 2: Observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RoomTransitionMissionTests
```

- [ ] **Step 3: Route intent before `runLoop`**

After emergency/busy checks, cancel stale motion before pose/topology validation. Begin mission memory normally, then branch standalone room intent into `runRoomTransitionMission` before calling `currentBrain()`.

- [ ] **Step 4: Observe crossing while navigation is active**

Ingest only newer normal-tracking observations from the topology’s active session generation. When observation returns `readyForConfirmation`, `await motion.stopAndWait()` first, then call `confirmTransition` to atomically commit topology, speak a concise success response if current UI conventions require it, and return idle.

- [ ] **Step 5: Verify**

Run the focused command from Step 2. Expected: happy path and object-target regression pass.

---

### Task 9: Add Bounded Candidate and Scan Recovery

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Modify: `swift/Sources/PhroverKit/Topology/SessionRoomTopology.swift`
- Modify: `swift/Tests/PhroverKitTests/RoomTransitionMissionTests.swift`

**Interfaces:**
- Adds mission-local candidate rejection and one bounded candidate-selection loop.

- [ ] **Step 1: Write failing recovery tests**

Cover:

- no initial candidate triggers at most twelve 30-degree scan steps and refreshes frontiers between steps;
- the total scan budget remains at most twelve steps across the entire mission, including failures after candidates are discovered;
- blocked/unreachable candidate moves to the next candidate;
- failed crossing does not mutate the graph;
- no candidate is attempted twice in one mission;
- no more than three candidates are attempted;
- tracking interruption stops and resumes only on a fresh normal observation;
- “stop” and mission cancellation abandon pending transition;
- exhaustion reports “I couldn’t find a safe route into another room.”

- [ ] **Step 2: Observe failures**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RoomTransitionMissionTests
```

- [ ] **Step 3: Implement the bounded loop**

Create a fresh exclusion set and one `remainingScanSteps = 12` counter when the mission begins. Pass exclusions into topology ranking, decrement the single counter for every scan step anywhere in the mission, and discard both at mission end. Refresh candidates after each scan step and recoverable navigation outcome. Reuse existing motion failure classification and safety stops; do not mark blocked candidates as traversed or globally visited. Add a regression proving a candidate excluded in one mission is eligible in the next.

- [ ] **Step 4: Verify**

Run the focused command from Step 2. Expected: all recovery, budget, and cancellation cases pass with no unbounded task.

---

### Task 10: Add Optional Cloud Doorway Evidence

**Files:**
- Add: `swift/Sources/PhroverCloud/Cloud/CloudDoorwayEvidenceProvider.swift`
- Modify: `Package.swift`
- Add: `swift/Tests/PhroverCloudTests/CloudDoorwayEvidenceProviderTests.swift`
- Modify: `examples/PhroverOperator/PhroverOperator.xcodeproj/project.pbxproj`
- Modify: `examples/PhroverOperator/PhroverOperator.xcodeproj/xcshareddata/xcschemes/PhroverSDKTests.xcscheme`

**Interfaces:**
- Produces a `PhroverCloud` implementation of `DoorwayEvidenceProviding`.
- Reuses the existing cloud vision brain and `/rover/act` wire contract; no new server endpoint is required.

- [ ] **Step 1: Expose `PhroverCloudTests` in SwiftPM and Xcode**

Add the test target and include it in `PhroverSDKTests`. Update `scripts/test-swift-sdk.sh` default gate to run all three regular test targets.

- [ ] **Step 2: Write failing adapter tests**

Use an injected `RoverBrain` fake. Present the frame and geometrically viable candidates as a constrained doorway-ranking context. The existing wire response is discrete, not probabilistic: map a returned `.explore(candidateId:)` choice to a `1.0` boost for that candidate and `0.0` for the others. Assert that non-explore output, unknown IDs, and errors are treated as no evidence; do not invent a confidence field that `/rover/act` does not return.

- [ ] **Step 3: Implement the adapter**

Keep the adapter in `PhroverCloud`. Do not make `PhroverKit` depend on cloud types. Enforce the 1.5-second deadline at the mission/provider boundary; timeout and offline state return no evidence.

- [ ] **Step 4: Verify ranking remains optional**

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverCloudTests/CloudDoorwayEvidenceProviderTests \
  -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

Expected: cloud selection boosts a viable candidate; geometry-only behavior is unchanged on every failure path.

---

### Task 11: Wire App-Scoped Topology and Lifecycle

**Files:**
- Modify: `examples/PhroverOperator/PhroverOperator/App/PhroverOperatorApp.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift`
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Sources/PhroverKit/Perception/RoomSessionCoordinator.swift`
- Modify: `swift/Tests/PhroverKitTests/RoomSessionCoordinatorTests.swift`
- Modify: `swift/Tests/PhroverKitTests/RoomTransitionMissionTests.swift`

**Interfaces:**
- Creates one `SessionRoomTopology` beside `ARSessionManager` and `NavigationController`.
- Passes topology and optional cloud evidence into `MissionAgent`.
- Uses the SDK-level `RoomSessionCoordinator` to reset topology synchronously whenever AR tracking is reset.

- [ ] **Step 1: Complete the SDK lifecycle coordinator regression**

In `RoomSessionCoordinatorTests`, assert that an AR reset awaits navigation stop, abandons a pending transition, allocates one new generation, passes that exact generation to topology and AR reset, prevents old- or mismatched-generation poses from recreating the graph, and starts topology exactly once from the first fresh normal pose of the new generation. Keep SwiftUI composition as a build-verified thin adapter rather than trying to import the example app into `PhroverKitTests`.

- [ ] **Step 2: Move ownership above SwiftUI view recreation**

Create topology in `PhroverOperatorApp`, pass it through `RootView`, and inject it into `ConversationView`/`MissionAgent`. Add a doorway evidence provider to `CloudSession` and configure its token source alongside `CloudBrain`.

- [ ] **Step 3: Wire reset ordering**

At each reset-tracking call, invoke the coordinator: await navigation stop, abandon pending transition, allocate a new generation, reset topology for that generation, clear/reset AR state with the same generation, then resume the AR session. Do not reset topology for a temporary interruption unless tracking is actually reset.

- [ ] **Step 4: Build and test**

```bash
scripts/test-swift-sdk.sh
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination "platform=iOS Simulator,id=${SIM_UDID}"
```

Expected: all regular SDK tests pass and the app builds.

---

### Task 12: Complete Telemetry, Regression Review, and Device Acceptance

**Files:**
- Modify only where missing: topology, mission, navigation, app, and tests from Tasks 2–11
- Modify: `README.md` with device-log pull and acceptance commands

**Required telemetry:**
- `room_session_started`
- `doorway_candidate_ranked`
- `room_transition_started`
- `doorway_crossed`
- `room_transition_completed`
- `room_transition_rejected`
- `room_session_reset`

- [ ] **Step 1: Add telemetry assertions**

Assert stable session-local IDs and ensure rejection/cancellation never emits crossing or completion.

- [ ] **Step 2: Run the full automated gate**

```bash
scripts/test-swift-sdk.sh
swift build --target RoverNav
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination "platform=iOS Simulator,id=${SIM_UDID}"
git diff --check
```

Expected: all commands pass.

- [ ] **Step 3: Run two-axis code review**

Review the complete diff against repository standards and `docs/superpowers/specs/2026-07-29-session-local-room-topology-mvp-design.md`. Resolve all blocking findings without discarding pre-existing intended WIP.

- [ ] **Step 4: Deploy and run device acceptance**

On a fresh AR session with the rover connected:

1. issue “Go to the other room”;
2. verify one viable doorway is selected and crossed;
3. verify current room changes from `room_1` to `room_2` only after crossing;
4. issue the reverse room-transition command and verify return to `room_1` through the known doorway;
5. test one blocked candidate and confirm rerouting remains bounded.

- [ ] **Step 5: Pull and assert the device log**

```bash
xcrun devicectl list devices
: "${DEVICE_ID:?Set DEVICE_ID to the connected iPhone identifier shown above}"
LOG_PATH="${TMPDIR:-/tmp}/phrover-runtime-room-topology.log"

xcrun devicectl device copy from \
  --device "$DEVICE_ID" \
  --domain-type appDataContainer \
  --domain-identifier bot.coy.phrover \
  --source Documents/phrover-runtime.log \
  --destination "$LOG_PATH"

rg "room_transition_(started|completed)|doorway_crossed|room_session_reset" "$LOG_PATH"
```

Expected: start, crossing, and completion events occur in order with stable IDs; no stale navigation failure, repeated rejected candidate, false room creation, or unbounded scan appears in the acceptance window.

- [ ] **Step 6: Commit only after acceptance**

Stage the intended implementation and test files, inspect the staged diff, and commit with a message that states the root cause: stale motion state plus brittle post-turn tracking and repeated opening selection prevented deterministic room transitions.
