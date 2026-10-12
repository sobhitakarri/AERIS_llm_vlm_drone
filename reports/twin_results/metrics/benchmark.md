# Twin flight benchmark

2026-10-12 16:06:24 · LiteWing ESP32-S3 · scene_lab · model gemini-robotics-er-2-preview · **8/8 passed**

Scored against simulator ground truth: true vehicle position while hovering versus true object position.

| id | prompt | status | passed | target_error_cm | standoff_error_cm | heading_error_deg | estimator_error_cm | sim_time_s | path_m | plan_latency_s |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Find the cardboard box and hover above it | done | True | 2.8 |  |  | 4.5 | 13.2 | 3.64 | 4.42 |
| 2 | Find the red bottle and hover above it for 4 seconds | done | True | 0.8 |  |  | 4.4 | 18.7 | 5.65 | 12.48 |
| 3 | Inspect the blue cube from 0.5 m away | done | True |  | 2.1 | 1.3 | 1.0 | 19.2 | 5.88 | 7.11 |
| 4 | Find the steel drum and hover above its center | done | True | 2.8 |  |  | 2.5 | 15.8 | 4.62 | 3.82 |
| 5 | Go to the yellow crate | done | True | 2.8 |  |  | 3.4 | 11.2 | 3.1 | 3.44 |
| 6 | Find the traffic cone and hover facing it | done | True |  | 2.9 | 0.1 | 4.4 | 10.0 | 2.89 | 6.18 |
| 7 | Take off to 0.8 m and fly in a square | done | True |  |  |  |  | 23.0 | 8.17 | 0.06 |
| 8 | Find the purple elephant and hover above it | refused | True |  |  |  |  |  |  | 3.79 |
