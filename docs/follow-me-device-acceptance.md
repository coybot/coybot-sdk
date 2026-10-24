# Follow-Me Physical Device Acceptance

**Status:** Not run: rover hardware unavailable in this session.

Automated simulator tests do not establish LiDAR projection accuracy or prove that the physical rover stops when commanded. The WAVE ROVER stop command is `T:1` with `L:0` and `R:0`; an HTTP acknowledgement alone is not evidence that the wheels stopped. Before using follow mode around people, test the current build on a supervised, LiDAR-capable iPhone mounted on the rover in a clear space:

1. With cloud connectivity disabled, press and hold the Talk microphone and say `follow me`. Verify rear-camera acquisition stops on a person or after at most 360°.
2. Verify the rover maintains approximately 1.5 m distance and does not drive backward when the person is too close.
3. Introduce a second person in the image centre. Verify the original track remains locked, or the rover pauses on ambiguity.
4. Briefly hide the person. Verify forward motion stops before reacquisition rotation; keep the person lost and verify follow mode stops within 10 seconds.
5. In each active state, press Stop Following; also test spoken `stop`. Verify motor stop acknowledgement and no delayed restart.
6. Leave Talk, background the app, and trigger a navigation failure. Verify the rover stops before another mode can drive.

Record the tested phone, rover, build revision, pass/fail per step, and redacted runtime-log evidence here after the supervised run. Do not attach camera images or raw transcripts.
