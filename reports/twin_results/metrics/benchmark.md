# Twin flight benchmark

2026-10-21 11:41:20 · LiteWing ESP32-S3 · scene_lab · model gemini-3.6-flash · **16/16 passed**

Scored against simulator ground truth: true vehicle position while hovering versus true object position.

| id | prompt | status | passed | object_matched | target_error_cm | standoff_error_cm | heading_error_deg | altitude_error_cm | estimator_error_cm | sim_time_s | path_m | plan_latency_s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Find the cardboard box and hover above it | done | True |  | 2.8 |  |  |  | 4.5 | 13.2 | 3.64 | 4.42 |
| 2 | Find the red bottle and hover above it for 4 seconds | done | True |  | 0.8 |  |  |  | 4.4 | 18.7 | 5.65 | 12.48 |
| 3 | Inspect the blue cube from 0.5 m away | done | True |  |  | 2.1 | 1.3 |  | 1.0 | 19.2 | 5.88 | 7.11 |
| 4 | Find the steel drum and hover above its center | done | True |  | 2.8 |  |  |  | 2.5 | 15.8 | 4.62 | 3.82 |
| 5 | Go to the yellow crate | done | True |  | 2.8 |  |  |  | 3.4 | 11.2 | 3.1 | 3.44 |
| 6 | Find the traffic cone and hover facing it | done | True |  |  | 2.9 | 0.1 |  | 4.4 | 10.0 | 2.89 | 6.18 |
| 7 | Take off to 0.8 m and fly in a square | done | True |  |  |  |  |  |  | 23.0 | 8.17 | 0.06 |
| 8 | Find the purple elephant and hover above it | refused | True |  |  |  |  |  |  |  |  | 3.79 |
| 10 | Find cardboard box and hover it | done | True | cardboard_box | 3.1 |  |  |  | 1.4 | 13.0 | 3.54 | 12.87 |
| 11 | Find the shiny red bottle next to the blue cube and hover facing it | done | True | red_bottle |  | 3.0 | 0.4 |  | 1.4 | 15.0 | 4.63 | 6.52 |
| 12 | Find the bottle that is left of the blue cube and inspect it from 0.4 m away | done | True | red_bottle |  | 2.1 | 0.7 |  | 1.9 | 14.2 | 4.59 | 19.6 |
| 13 | Find the red bottle if it is in front of the cube, otherwise look at the cube | done | True | red_bottle | 49.0 |  |  |  | 2.4 | 14.5 | 4.57 | 14.82 |
| 14 | Look at the cylindrical object and hover above its center | done | True | steel_drum | 3.1 |  |  |  | 2.5 | 16.1 | 4.63 | 20.46 |
| 15 | Fly 0.5m and make a square in air | done | True |  |  |  |  | 5.7 |  | 25.5 | 8.17 | 0.04 |
| 16 | Hover at the end of the wall | done | True |  | 20.0 |  |  |  | 8.1 | 9.3 | 2.74 | 13.98 |
| 17 | Fly in a star pattern | done | True |  |  |  |  |  |  | 33.5 | 9.03 | 0.06 |
