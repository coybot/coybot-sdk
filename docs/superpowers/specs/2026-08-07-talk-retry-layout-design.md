# Talk Retry Layout Design

## Problem

After a voice mission fails, `ConversationView` displays a command-result card. The
fixed vertical layout then pushes the microphone control under the floating tab bar.
The screen says `Ready`, but the tab bar intercepts the next hold gesture, so no new
speech capture starts.

Device evidence confirms this is a UI interaction failure: the first capture completed,
the mission returned to idle, and no second `speech_capture_started` event was logged.

## Behavior

- A failed command remains visible for diagnosis.
- Camera, navigation diagnostics, transcript, and command result occupy a scrollable
  upper region.
- The phase label and push-to-talk microphone occupy a fixed lower region above the
  tab bar.
- Holding the microphone while the agent is `Ready` starts a fresh speech capture.
- Releasing the microphone finishes that capture using the existing `SpeechIn` flow.
- A navigation failure is not automatically replayed because it may represent a safety
  stop. The operator explicitly retries by speaking again.

## Implementation

Refactor `ConversationView` into two layout regions:

1. A `ScrollView` containing the live camera panel, partial transcript, and last-command
   card.
2. A stable voice-control region containing the phase label and microphone gesture.

Remove the negative microphone offset. Give the microphone an explicit stable frame,
content shape, and accessibility identifier so its hit target cannot collapse or move
behind the tab bar when command content grows.

No changes are required in `SpeechIn`: its completed capture already returns to idle,
deactivates the audio session, and can start a new capture.

## Failure Handling

Speech authorization and capture-start errors continue to use the existing status and
runtime logging. The failed-command card remains present while the next capture begins,
then the existing `LastCommandState` event flow replaces it with the new listening draft.

## Verification

- Add a UI regression check that the push-to-talk control remains present and hittable
  when the Talk screen contains a failed-command card.
- Run the focused iOS test, then the relevant PhroverKit suite.
- Build and install the app on the connected iPhone.
- On-device validation: fail one mission, wait for `Ready`, hold the mic again, and
  confirm a new `speech_capture_started` event appears.
