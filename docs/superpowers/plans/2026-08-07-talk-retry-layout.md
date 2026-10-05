# Talk Retry Layout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep push-to-talk fully visible and interactive after a failed voice command so the operator can hold the mic and submit a fresh command.

**Architecture:** Split `ConversationView` into a scrollable information region and a fixed voice-control region. Add a Debug-only launch-argument fixture that seeds the failed-command card, allowing an XCUITest to verify that the mic remains above the tab bar and hittable.

**Tech Stack:** Swift 6, SwiftUI, XCTest/XCUITest, Xcode iOS device test runner

## Global Constraints

- Preserve the failed command card while the screen is idle.
- Retry is operator initiated; never automatically replay a failed navigation command.
- Do not change `SpeechIn`, mission safety behavior, or rover command behavior.
- Keep all UI-test fixture behavior behind `#if DEBUG` and `-ui-test-failed-command`.
- Preserve unrelated worktree changes.

---

### Task 1: Keep Push-to-Talk Hittable After Failure

**Files:**
- Modify: `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift:21-89`
- Test: `examples/PhroverOperator/PhroverOperatorUITests/PhroverOperatorUITests.swift`

**Interfaces:**
- Consumes: `LastCommandState.reduce(_ status: MissionCommandStatus)` to create a deterministic failed card in Debug UI tests.
- Produces: accessibility element `push-to-talk-control`, fixed above the `Talk` tab bar and retaining the existing hold/release gesture behavior.

- [ ] **Step 1: Write the failing UI regression test**

Add this test to `PhroverOperatorUITests`:

```swift
func testTalk_failedCommandKeepsPushToTalkHittable() throws {
    let app = XCUIApplication()
    app.launchArguments.append("-ui-test-failed-command")
    app.launch()

    let talkTab = app.buttons["Talk"]
    XCTAssertTrue(talkTab.waitForExistence(timeout: 15))
    talkTab.tap()

    let failedCard = app.otherElements["last-command-card"]
    XCTAssertTrue(failedCard.waitForExistence(timeout: 10))

    let mic = app.descendants(matching: .any)
        .matching(identifier: "push-to-talk-control").element
    XCTAssertTrue(mic.waitForExistence(timeout: 10))
    XCTAssertTrue(mic.isHittable)
    XCTAssertLessThanOrEqual(mic.frame.maxY, talkTab.frame.minY)
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
xcodebuild test \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'platform=iOS,id=C40C8EA8-E545-5B47-ADEA-CD8118AE844C' \
  -derivedDataPath /private/tmp/phrover-talk-retry-red \
  -only-testing:PhroverOperatorUITests/PhroverOperatorUITests/testTalk_failedCommandKeepsPushToTalkHittable
```

Expected: FAIL because `push-to-talk-control` does not exist and the failed-card fixture is not yet installed.

- [ ] **Step 3: Add the Debug-only failed-command fixture**

Change the `lastCommand` state initializer in `ConversationView` to call a private factory:

```swift
@State private var lastCommand = ConversationView.makeInitialLastCommandState()

private static func makeInitialLastCommandState() -> LastCommandState {
    var state = LastCommandState()
#if DEBUG
    if ProcessInfo.processInfo.arguments.contains("-ui-test-failed-command") {
        state.reduce(.recognized(id: 1, command: "Go to our room"))
        state.reduce(.failed(
            id: 1,
            command: "Go to our room",
            message: "I can't safely see the space needed to turn."
        ))
    }
#endif
    return state
}
```

- [ ] **Step 4: Split scrollable information from fixed voice controls**

Replace the single overflowing `VStack` body with this structure while retaining the existing camera, transcript, and command-card contents:

```swift
VStack(spacing: 12) {
    ScrollView {
        VStack(spacing: 18) {
            if !statusLabel.isEmpty {
                Text(statusLabel).font(.headline)
            }

            LiveCameraDebugPanel(ar: ar, summary: navigationDebug)
                .frame(maxWidth: 320)

            Text(speechIn.partialTranscript)
                .foregroundStyle(.secondary)
                .frame(minHeight: 40)
                .multilineTextAlignment(.center)

            if let record = lastCommand.record {
                VStack(alignment: .leading, spacing: 8) {
                    Text(record.command)
                        .font(.headline)
                        .accessibilityIdentifier("last-command-text")
                    Text(record.status.rawValue)
                        .font(.subheadline.weight(.semibold))
                        .foregroundStyle(statusColor(record.status))
                        .accessibilityIdentifier("last-command-status")
                    Text(record.message)
                        .font(.subheadline)
                        .foregroundStyle(.secondary)
                        .accessibilityIdentifier("last-command-message")
                }
                .frame(maxWidth: 320, alignment: .leading)
                .padding(14)
                .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 14))
                .accessibilityElement(children: .contain)
                .accessibilityIdentifier("last-command-card")
            }
        }
        .frame(maxWidth: .infinity)
    }
    .scrollIndicators(.hidden)

    VStack(spacing: 12) {
        if agent != nil {
            Text(phaseStatusLabel)
                .font(.subheadline)
                .padding(.horizontal, 18)
                .padding(.vertical, 10)
                .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 12))
        }

        Image(systemName: "mic.circle.fill")
            .font(.system(size: 72))
            .frame(width: 96, height: 96)
            .contentShape(Circle())
            .foregroundStyle(speechIn.state == .listening ? .red : .accentColor)
            .accessibilityLabel("Push to talk")
            .accessibilityIdentifier("push-to-talk-control")
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { _ in startListening() }
                    .onEnded { _ in speechIn.finish() }
            )
    }
    .padding(.bottom, 8)
}
.padding(.horizontal)
.padding(.top, 20)
```

Remove the old `.offset(y: -36)`, `.padding(.bottom, 40)`, and trailing `Spacer()` so dynamic command content cannot push the mic under the tab bar.

- [ ] **Step 5: Run the focused UI test and verify GREEN**

Run:

```bash
xcodebuild test \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'platform=iOS,id=C40C8EA8-E545-5B47-ADEA-CD8118AE844C' \
  -derivedDataPath /private/tmp/phrover-talk-retry-green \
  -only-testing:PhroverOperatorUITests/PhroverOperatorUITests/testTalk_failedCommandKeepsPushToTalkHittable
```

Expected: PASS; the failed card exists, the mic is hittable, and its lower edge is no lower than the Talk tab's upper edge.

- [ ] **Step 6: Run related regression tests**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-talk-retry-sdk.xcresult \
  -only-testing:PhroverKitTests/LastCommandStateTests
```

Expected: PASS with zero failures.

- [ ] **Step 7: Verify formatting and build for the connected iPhone**

Run:

```bash
git diff --check
xcodebuild build \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'platform=iOS,id=C40C8EA8-E545-5B47-ADEA-CD8118AE844C' \
  -derivedDataPath /private/tmp/phrover-talk-retry-build \
  -allowProvisioningUpdates
```

Expected: `git diff --check` exits 0 and Xcode reports `BUILD SUCCEEDED`.

- [ ] **Step 8: Install and launch on the iPhone**

Run:

```bash
xcrun devicectl device install app \
  --device C40C8EA8-E545-5B47-ADEA-CD8118AE844C \
  /private/tmp/phrover-talk-retry-build/Build/Products/Debug-iphoneos/PhroverOperator.app
xcrun devicectl device process launch \
  --device C40C8EA8-E545-5B47-ADEA-CD8118AE844C \
  --terminate-existing bot.coy.phrover
```

Expected: installation and launch both succeed.

- [ ] **Step 9: Verify the real retry path from a fresh device log**

After the operator fails one command and holds the mic again, run:

```bash
DEST=$(mktemp -d /private/tmp/phrover-talk-retry-log-XXXXXX)
xcrun devicectl device copy from \
  --device C40C8EA8-E545-5B47-ADEA-CD8118AE844C \
  --domain-type appDataContainer \
  --domain-identifier bot.coy.phrover \
  --source Documents \
  --destination "$DEST"
rg "speech_audio_session_activated|speech_capture_started|mission_motion_failed" \
  "$DEST/phrover-runtime.log" | tail -n 20
```

Expected: a second `speech_audio_session_activated` and `speech_capture_started
capture_id=2` occur after the first terminal `mission_motion_failed` event.

- [ ] **Step 10: Commit only the implementation files**

```bash
git add \
  examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift \
  examples/PhroverOperator/PhroverOperatorUITests/PhroverOperatorUITests.swift
git commit -m "Keep Talk retry control accessible"
```
