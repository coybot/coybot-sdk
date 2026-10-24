# In-place follow search implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox syntax. Do not commit or push: user has authorized implementation/build/install, not publication.

**Goal:** Search for and align with a person using bounded in-place turns; assess forward clearance only before translation.
**Architecture:** Preserve the existing search/acquire/align/clearance sequence, but give in-place search a distinct, explicit source of initial surrounding-clearance evidence. An app confirmation supplies a finite, revocable, location- and AR-session-bound clearance assertion for unknown sectors. Known depth hazards, missing/stale/malformed depth, source health and stop/transport fencing remain authoritative. Search must not substitute a translating recovery arc. Existing general mission safety remains unchanged.
**Tech Stack:** Swift, SwiftUI, ARKit, XCTest, Xcode device/simulator.
**Spec:** User approved the intended sequence from /private/tmp/coybot-search-coverage-20261023-latest/analysis.md. Product clarification offered clear-start confirmation versus previously observed clearance. Recommended confirmation is the current explicit implementation assumption, announced to user; user can steer it. This is a UI assertion made on the phone, not agent permission to move the rover.

## Global Constraints
- Work only in /private/tmp/coybot-sdk-follow-port-20261019 on port/astral-follow-me-continuity. Preserve all prior uncommitted fixes and .superpowers assets.
- No actual rover movement tests. No Git commit/push.
- 30 cm minimum total forward stopping threshold and dynamic braking requirement stay unchanged.
- Search uses equal/opposite wheel commands and existing <=80ms host burst cap, acknowledged stops and fresh-frame observation between steps. Never translate to bootstrap searching.
- Unknown surrounding space is not sensor-proven clear. Explicit operator confirmation supplies scoped clearance; current known obstacles still veto motion.
- Confirmation must be revoked by cancellation/Stop, background/leave, session replacement, AR tracking reset/loss, expiry, or movement from approved origin. Never turn a previous confirmation into a new-session grant, and never renew its expiry per pulse.
- Initial conservative grant: 60 seconds maximum from confirmation, at most 5 cm observed displacement from approved rover origin, same AR source generation; user confirms people and objects at least 50 cm away from rover. These are observed/software bounds, not physical guarantees. Normal search sweep/attempt limits still apply. Expiry requires a new explicit confirmation; no auto-confirm.
- Confirmation authorizes initial searching and alignment at that origin only; no forward ready signal/pursuit or later reacquisition after translation inherits it.
- Retain raw-depth 250ms expiry and transport-time rechecks. UI confirmation must not resume a stopped, cancelled or replaced submission.
- Prefer small typed interfaces and existing task-local/context patterns; no blanket global rotation bypass, no treating unobserved map cells as free, no invented freshness.

## Review Focus
- Stop/background/new command while confirmation is pending: no delayed movement or stale approval.
- Same grant reused after translation, reset, expiry or new session: rejected at planning AND last transport authorization.
- New visible obstacle during approved search: rejects/halts; depth missing/stale remains unavailable.
- No person yet: only zero-translation pulses; fresh stopped detection before next pulse; acquiring person cancels search immediately.
- Forward movement after target: retains sensor coverage, stopping threshold, readiness/path authorization, and cannot use operator rotation clearance.

### Task 1: Integrated operator-cleared in-place search
**Files:** SDK FollowMe request context/coordinator/router/adapters, Nav FollowNavigationController and NavigationController bridge, Perception DepthSafety evaluator/models only as necessary; app ConversationViewModel/ConversationView; targeted corresponding SDK/app tests. Add focused typed helper file if needed; update Xcode project only if file discovery requires it.
**Interfaces:** Explicit app-confirmed clearance -> session-scoped follow request evidence -> rotation-specific depth decision -> existing bounded transport fence. Implementer must propose exact API and lifecycle before production edits; parent settles any conflicts.
- [ ] Read existing APIs and tests; document exact interface/lifecycle in report.
- [ ] Add real regression coverage: blind forward camera + approved clear origin permits bounded pure rotation; same without approval stays stopped; depth obstacle/stale invalidation rejects; search never falls back to arc; stopped fresh frame gates next scan.
- [ ] Add UI/router tests for confirm, cancel, Stop/reconfigure/background races and no confirmation inheritance.
- [ ] Run new tests RED against existing behavior (build errors are not evidence).
- [ ] Implement smallest integrated change, preserving previous fixes and user 30 cm selection.
- [ ] Run focused SDK and app tests GREEN; self-review; report commands/results/limits. Do not run full suites concurrently with parent.

### Task 2: Independent review and delivery
**Files:** Validation artifacts only unless fix findings are returned to Task 1 implementer.
- [ ] Parent prepares incremental and cumulative review packages; independent safety/concurrency review.
- [ ] Resolve substantive findings with targeted red/green proofs.
- [ ] Parent runs full SDK + app suites, builds iPhone app, installs and launches. No physical follow command.
- [ ] Inspect final diff and report exact verification and physical acceptance limits; remain uncommitted.
