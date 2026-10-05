# Apple Intelligence Timeout Design

## Problem

`HybridBrain` currently cancels the on-device Apple Intelligence request after
1.5 seconds even when no cloud brain can take over. Device logs show that the
system model is available, but cold `LanguageModelSession` inference regularly
needs longer than that deadline. The cancellation therefore reports a timeout
and sends supported commands to the deterministic offline parser instead of
allowing Apple Intelligence to answer.

## Behavior

- Apple Intelligence remains the primary brain.
- When cloud fallback is absent or the device is offline, `HybridBrain` waits
  for the on-device result without its own primary-stage timeout. The existing
  12-second `MissionAgent` decision timeout remains the overall safety bound.
- When cloud fallback is present and online, the primary-stage timeout is eight
  seconds. A timeout then starts cloud fallback as it does today.
- A fresh `LanguageModelSession` remains scoped to each decision so model
  conversation state cannot leak between missions.
- The deterministic offline object parser remains the final fallback when both
  configured brain stages fail.

## Telemetry

Existing `mission_brain_selected` events remain compatible. Successful local
decisions continue to report `brain=on_device reason=primary`; online timeout
fallback continues to report `reason=on_device_timed_out`.

## Tests

- A local response that takes longer than the configured primary timeout must
  still succeed when no cloud brain is available.
- The same behavior must hold when a cloud brain exists but connectivity is
  unavailable.
- With an online cloud brain, the configured timeout must still cancel the
  delayed local stage and invoke cloud fallback promptly.
- Existing cancellation and selection telemetry tests must remain green.

## Scope

This change does not alter prompts, object grounding, model-session state,
navigation, LiDAR safety, or cloud request behavior.
