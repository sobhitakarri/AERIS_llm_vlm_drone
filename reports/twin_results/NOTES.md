## 8. How this run was made, and what it found

All runs: 12 October 2026, LiteWing airframe, `scene_lab`, twin with rendering
on an RTX 3050 laptop (about 0.6-0.7x real time). Videos are time-lapses of
roughly 2-3x simulated time; the simulation clock is on screen in each one.

### Language model used

The project's usual models (`gemini-3.5` to `3.8-flash`) had used up the free
tier's daily quota (20 requests per model) in earlier test runs the same day,
so the recorded run used models that still had quota:

| Run | Model | Result | Kept as |
|---|---|---|---|
| Earlier the same day, not recorded on video | gemini-3.6-flash | 8/8 | `metrics/benchmark_earlier_gemini-3.6-flash.json` |
| Attempt 1 | gemini-flash-lite-latest | 5/8, three flights to the wrong object | `metrics/benchmark_attempt1_before_format_check.json`, `logs/04_language_missions_attempt1.log` |
| Attempt 2, after the format checks below | gemini-flash-lite-latest | 4/8: three refused, one flight to the wrong object | `metrics/benchmark_attempt2_flash-lite.json`, `logs/04_language_missions_attempt2.log` |
| **Reported run (section 4, videos)** | gemini-robotics-er-2-preview | 6/8 on the first pass, 8/8 after cases 1 and 2 were re-run | `metrics/benchmark.json` |
| Replan test (section 5) | gemini-3-flash-preview | pass | `metrics/replan.json` |

Cases 1 and 2 of the reported run were refused on the first pass because
every model call returned 503 or timed out; no flight took place. They were
re-run a few minutes later and are marked `rerun` in `benchmark.json`. Each
case has one scored flight; there are no repeats, so these are single
samples, not averages.

`gemini-flash-lite-latest` is not suitable for this task. It answered with
nine-number arrays, mixed scales such as `[7.32, 143, 198, 770]`, repeated
keys and a negative coordinate, and once boxed the cardboard box when asked
for the yellow crate. The depth check cannot catch that last kind of error:
the box was on a real object, only the wrong one.

### Faults found by the recorded runs, and fixed

| Fault | Effect seen | Fix |
|---|---|---|
| Gyro bias calibration had not converged when the vehicle took off, and kept adapting while the rotors spun up on the pad | Heading drifted about 2.4 deg per minute of flight. After a few missions the position estimate was 18-29 cm out, the camera fixes were refused as too large, and the vehicle hit the partition wall | Bias is the average over a boot window, then only trimmed slowly while still (`+aeris/Sensors.m`). Yaw after the 90 degree step in validation went from 88.2 to 90.1 deg |
| A position fix larger than 12 cm was refused however far the vehicle had flown, so a drifted estimate could never recover | As above | The gate widens with distance flown since the last fix; heading error is measured from each leg and fed back (`backend/vision/scene_service.py`) |
| A model request with no deadline | One overloaded model blocked a mission for several minutes | 30 s deadline, retry on 503 / timeout, then the next model (`backend/models/gemini_provider.py`) |
| Malformed model answers were parsed loosely | A repeated key silently took the second box; a stray bracket discarded a correct answer | First value wins, trailing bracket ignored, a box that is not four non-negative same-scale numbers is asked for again (up to three times) |
| Return-home requested the instant a mission finished was rejected as "mission running" | The next mission started from mid-air | The request waits up to 3 s for the mission thread to finish (`backend/api/routes.py`) |

Unit tests for these are in `backend/tests/test_live_run_fixes.py`.

### Limits of these results

* Estimator error at the scoring moment was 1.0-4.5 cm in the reported run,
  against 0.3-2.3 cm in the earlier run. Over the table edge the height
  estimate was about 6 cm out during the replan test.
* In case 6 the stand-off point is next to the partition wall, so the planner
  holds the vehicle at 1.5 m. The heading is correct, but from that height
  the cone sits at the bottom edge of the onboard camera's view.
* The link check (section 3) flies without the backend, so there are no
  camera fixes: the 4-8 cm arrival errors there are the onboard estimator and
  the arrival tolerance alone.
* The real-time factors in section 2 include model compilation on the first
  of the three runs and vary from run to run.
* The local Qwen models were not tested: with Unreal holding the GPU, one
  Qwen2-VL detection did not finish within 60 s on this PC.
* The dashboard screenshot was taken by a headless browser, which is why its
  simulation-speed chip reads x0.00.
