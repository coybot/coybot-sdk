# Astral stashed follow fixes: comparison with Coybot

## Sources

- Astral committed port source: `6eb91e4`.
- Astral working-change snapshots: `bfaa867` and `afa1fba` (October 9).
- Astral local specs: `2026-10-09-follow-alignment-acceptance.md`,
  `2026-10-09-follow-pose-evidence-ordering.md`, and
  `2026-10-08-follow-distance-and-steering.md`.
- Coybot baseline: `b79c285`, plus the subsequent device-driven working fixes.

The committed source and current Astral `main` use 0.05 radians for alignment.
The stashes contain the tested 5° acceptance change. A comparison against only
committed branches therefore misses this fix.

## Restored in this change

- Inclusive 5° follow alignment acceptance and a shared coordinator default for
  post-stop validation and ready admission.
- Astral's planner, real navigation adapter, and admission boundary regressions,
  adapted to current Coybot tests and supplemented with the latest device trace.
- Existing correction/overshoot and transport tests retain their original intent
  with worked values outside the new acceptance window.

## Other stash differences identified

These are follow-up reconciliation items, not claims of complete stash parity:

| Area | Current Coybot comparison |
| --- | --- |
| Reordered pose evidence | Current fix already reconciles complete ingress archives and checks every retained control sample. It also independently validates every control collection clock. Astral additionally deduplicates/source-sorts the control collection and compares source identity/clock labels. |
| Live follow goals | Newer continuous-follow implementation already updates live goals and fences arrival/ownership. The older stash is not a drop-in replacement. |
| Distance hysteresis | Stashed 1.20 m stand-off, 1.35 m resume, 1.28 m moving-stop threshold and 8 cm pursuit arrival margin are absent. Current defaults remain 1.50/1.75 m with 20 cm pursuit arrival. |
| Holding-direction correction | Stashed fresh-post-stop, verified 8° trigger / 20° bounded centering is absent. |
| Moving-goal progress | Current follow drive resets its progress watchdog on live goal updates; the stash measures actual approach across poses without resetting on retarget. This needs reconciliation with continuous pursuit and its tests. |
| Verification grace | Current code has acquisition grace plus newer bearing/coasting continuation. The stash's stopped following/holding grace is not equivalent. |

Further reconciliation should use the current coordinator/controller/transport
seams, preserving continuous pursuit, freshness, ownership and confirmed-stop
behavior rather than applying the old stash wholesale.

## Verification

- New acceptance, recorded-response, real-adapter and ready-admission regressions
  failed under the old 0.05-radian policy before the production change.
- Full Swift simulator suite: 1,360 passed, zero failures/skips/retries, with
  parallel testing disabled to avoid the previously observed shared-log interference.
- Correctness/spec and standards reviews: no remaining findings.
- Signed device build succeeded; installed on iPhone 15 Pro at 21:04 PDT on
  October 27. Physical follow-me acceptance remains to be checked on the rover.
