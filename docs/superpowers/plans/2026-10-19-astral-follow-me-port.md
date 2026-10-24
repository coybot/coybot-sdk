# Astral Follow-Me Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring Astral SDK's current Phrover follow-me behavior and continuity fixes into Coybot's SDK, then update Coybot's SDK pin.

**Architecture:** Keep Coybot's existing SDK and Silent Search history. Port only the follow-me feature series from Astral commit `9b24917` through `6eb91e4`, resolving shared-file changes against Coybot SDK `4fa61b5`. A dedicated follow navigation engine preserves the public Coybot controller's operation-token ownership and generic navigation. Keep the old `FollowController` API until call sites and tests prove it can be removed safely.

**Tech Stack:** Swift 6, SwiftPM, Xcode iOS simulator, SwiftUI, Git submodules.

**Spec:** Astral SDK `docs/superpowers/specs/2026-09-29-follow-me-design.md` and `docs/superpowers/specs/2026-10-12-continuous-follow-coasting-tolerant-tracking.md`.

## Global Constraints

- Preserve Coybot-specific navigation, voice, and Silent Search behavior.
- Keep production and remote repositories unchanged until the final review and required approval.
- Carry the four verified continuity regressions from Astral commit `6eb91e4`.

## Review Focus

- A low-confidence verified person retains bearing evidence below the full-track threshold.
- Bearing-only motion expires at the configured maximum even while bearings continue.
- A fresh full person observation can reacquire after a camera turn and stale screen box.
- A live goal update is refused once arrival shutdown begins.
- Follow cancellation and navigation ownership still work with Coybot's existing mission flow.

### Task 1: Establish test evidence

- [x] Run Coybot's existing follow and policy tests on the unmodified SDK.
- [x] Add Astral's live-goal regression and verify it fails against Coybot's missing follow API.

### Task 2: Port the follow implementation

- [x] Bring over the final follow-me models, perception, association, coordinator, navigation helpers, and Kalman filter.
- [x] Merge shared navigation, rover control, detector, mission, and app changes without discarding Coybot behavior.
- [x] Exclude unrelated Astral Silent Search scanner-diagnostic changes from the port.
- [x] Run focused follow, navigation, and mission tests; resolve integration failures.

### Task 3: Verify and prepare delivery

- [x] Run the complete regular SDK test suite and PhroverOperator simulator build.
- [x] Run the new Talk model and operator UI simulator tests.
- [x] Inspect the final diff and security/concurrency-sensitive paths.
- [ ] Advance Coybot's SDK gitlink to the verified SDK commit after authorization for commit/push.
