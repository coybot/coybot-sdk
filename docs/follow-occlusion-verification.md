# Follow occlusion recovery

## Device reproduction

November 20, 2026: `iphone15pro-target-occlusion.log`, captured from iPhone 15 Pro,
contains records from 21:21:25Z to 21:41:50Z. Ten-degree recovery segments
accepted approximately three to four degrees of measured motion. Repeated
stop/observe operations consumed the fixed ten-second recovery deadline.
A rejected hip confidence of 0.286 also illustrates why partial detections
must remain useful as search hints without becoming verified positions.

## Invariants and implementation checks

- Search segment completion requires measured yaw within two degrees of the
  requested segment. A controller regression replays a four-degree response to
  a ten-degree request and requires another correction.
- Last reliable target movement determines which side is searched first.
  Compute both bearings from the same rover position so rover yaw changes
  cannot masquerade as person movement.
- Partial observations may choose search order only. They cannot replace the
  reliable position, become translation authority, or renew the loss deadline.
  Reject stale, future, wrong-generation, invalid, and ambiguous hints.
- Episode identity, reliable anchor, and deadline remain immutable across
  measured stage and segment updates.
- Full verified observations still pass ordinary association, confirmed-stop,
  source-generation, and depth admission before translation resumes.

## Concurrency model proposal

A small TLA+ model could use `phase`, `owner`, `generation`, `deadline`,
`stopConfirmed`, `verifiedTarget`, and `hint` as state variables. Actions:
`Lose`, `Hint`, `Turn`, `VerifiedMatch`, `StopAck`, `Reset`, `Cancel`, `Expire`.
Check that hints never enable translation or extend the deadline; stale
owners/generations never send; cancellation and expiry require a confirmed
stop before any new owner can move. This is a proposed model, not an executed
model check or proof of production correctness.

Physical lateral exits, occlusion, crossing people, motor response and clearance
still require device acceptance. Spatial association is not identity recognition.

## Regression reproduction

Before the fix:

```sh
xcodebuild test -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverSDKTests \
  -destination 'platform=iOS Simulator,id=EEA52712-371D-4FF6-B8EF-A2C78319D57F' \
  -only-testing:PhroverKitTests/NavigationFollowScanDiagnosticsTests/testRecoveryTenDegreeSegmentDoesNotFinishAfterOnlyFourDegrees
```

Result: one failed test. `XCTAssertGreaterThanOrEqual` reported one command
instead of at least two: “A small initial response must not end the ten-degree
search segment.” Result bundle:
`Test-PhroverSDKTests-2026.11.20_21-16-27--0800.xcresult`.

After the fix, the full RoverNavTests, PhroverKitTests and PhroverCloudTests
selection passed **1,740/1,740**, zero failures and skips, in
`Test-PhroverSDKTests-2026.11.20_21-21-26--0800.xcresult`.
Generated cases cover both lateral directions over 24 world headings and
freshness/generation/ambiguity combinations. Coordinator tests check that the
hint affects the actual first recovery request without adding translation goals.
Existing cancellation, stop-failure, deadline, native-admission and source-reset
regressions also ran in this suite.

The first targeted run caught a hint timestamp older than its anchor; the
reproduction (`generation=4, time=0.9, count=1`, anchor time 1, now 1.3) is retained
in the generated cases. The old diagnostic fixture used a fixed 2.86-degree
overshoot, outside the new two-degree segment corridor; its measured overshoot
is now 1.15 degrees while still checking that diagnostics record actual yaw.
