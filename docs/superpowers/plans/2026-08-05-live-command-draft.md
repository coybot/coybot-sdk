# Live Command Draft Implementation Plan

**Goal:** Show a command card immediately on microphone press, update it from live speech partials, and preserve safe final-command dispatch.

**Approved design:** `docs/superpowers/specs/2026-08-05-live-command-draft-design.md`

### Task 1: Add identified speech-capture events

**Files:**
- Modify `swift/Sources/PhroverKit/Voice/SpeechIn.swift`
- Modify `swift/Tests/PhroverKitTests/SpeechFinalizationWatchdogTests.swift` or add focused speech event tests

1. Write failing tests for monotonic capture IDs, immediate started event, retained non-empty partials, final closure, no-speech failure, and stale callback rejection.
2. Add `SpeechCaptureID` and typed start/partial/failure events.
3. Keep final transcript dispatch separate; partials must never call the mission handler.
4. Run the focused speech tests.

### Task 2: Reduce draft events into the command card

**Files:**
- Modify `swift/Sources/PhroverKit/Voice/LastCommandState.swift`
- Modify `swift/Tests/PhroverKitTests/LastCommandStateTests.swift`
- Modify `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift`

1. Write failing reducer tests for immediate Listening state, partial updates, no-speech failure, final replacement, and stale-event rejection.
2. Extend `LastCommandState` with one active capture identity and draft transitions.
3. Wire `ConversationView.startListening` to capture events on the main actor.
4. Preserve existing identified `MissionCommandStatus` handling and accessibility IDs.
5. Run reducer tests and build the app target.

### Task 3: Verify and deploy

1. Run focused speech/reducer suites and `scripts/test-swift-sdk.sh`.
2. Build the physical-device target.
3. Install and launch `bot.coy.phrover` on the paired iPhone.
4. Verify immediate Listening card, partial update when available, final Recognized/Working transition, and silent-attempt failure.
5. Pull the runtime log if device behavior differs.

**Constraints:** Do not start missions from partial speech, add history, weaken navigation safety, or stage unrelated dirty-worktree changes.
