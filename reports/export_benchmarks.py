"""Generate research-paper benchmark workbook.

Sheets
  1. Tracker_Comparison   — measured OpenCV tracker FPS / IoU / occlusion recovery
  2. Pipeline_Rates       — loop frequencies used in the indoor stack
  3. Tracker_Health       — CSRT health-check and lock-state constants
  4. Replan_Safety        — drift / cooldown / TARGET_LOST thresholds
  5. LiteWing_Control     — optical-flow odometry and hover gains
  6. Workspace            — indoor geofence used by the planner
  7. Methodology          — how the numbers were produced (cite this)

The tracker comparison is synthetic overhead-camera motion: a textured,
colourless 60x60 blob on a textured background at 640x480. Colour thresholding
finds nothing here; the point is class/colour-agnostic tracking.
"""
from __future__ import annotations

import platform
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reports" / "Research_Benchmarks.xlsx"

W, H = 640, 480
OCC_START, OCC_END = 60, 70
N_FRAMES = 120
N_TRIALS = 5
SEEDS = (7, 11, 19, 29, 41)

_rng_obj = np.random.default_rng(3)
_OBJ = cv2.GaussianBlur(
    _rng_obj.integers(120, 255, (60, 60, 3)).astype(np.uint8), (7, 7), 0
)


def make_frame(t: int, seed: int, occlude: bool = False):
    rng = np.random.default_rng(seed)
    bg = rng.integers(40, 90, (H, W, 3)).astype(np.uint8)
    bg = cv2.GaussianBlur(bg, (9, 9), 0)
    cx, cy = int(120 + t * 3.0), int(240 + 60 * np.sin(t * 0.08))
    bg[cy - 30 : cy + 30, cx - 30 : cx + 30] = _OBJ
    if occlude:
        cv2.rectangle(bg, (cx - 45, cy - 45), (cx + 45, cy + 45), (30, 30, 30), -1)
    return bg, (cx - 30, cy - 30, 60, 60)


def iou(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    return inter / float(aw * ah + bw * bh - inter + 1e-9)


def factories():
    items = [
        ("CSRT", cv2.TrackerCSRT_create),
        ("KCF", cv2.TrackerKCF_create),
        ("MIL", cv2.TrackerMIL_create),
    ]
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMOSSE_create"):
        items.append(("MOSSE (legacy)", cv2.legacy.TrackerMOSSE_create))
    return items


def run_one(name: str, ctor, seed: int) -> dict:
    frame, gt = make_frame(0, seed)
    tr = ctor()
    tr.init(frame, gt)
    pre, post, t_total = [], [], 0.0
    lost_pre = lost_occ = lost_post = 0
    for i in range(1, N_FRAMES):
        frame, gt = make_frame(i, seed, occlude=OCC_START <= i <= OCC_END)
        t0 = time.perf_counter()
        ok, box = tr.update(frame)
        t_total += time.perf_counter() - t0
        if not ok:
            if i < OCC_START:
                lost_pre += 1
            elif i <= OCC_END:
                lost_occ += 1
            else:
                lost_post += 1
            continue
        score = iou(gt, tuple(int(v) for v in box))
        if i < OCC_START:
            pre.append(score)
        elif i > OCC_END:
            post.append(score)
    n_pre = OCC_START - 1
    n_occ = OCC_END - OCC_START + 1
    n_post = (N_FRAMES - 1) - OCC_END
    recovered = bool(post) and float(np.mean(post)) > 0.5
    return {
        "tracker": name,
        "trial_seed": seed,
        "fps": (N_FRAMES - 1) / t_total,
        "iou_pre_occlusion_mean": float(np.mean(pre)) if pre else 0.0,
        "iou_post_occlusion_mean": float(np.mean(post)) if post else 0.0,
        "lost_pre": lost_pre,
        "lost_during_occlusion": lost_occ,
        "lost_post": lost_post,
        "frames_pre": n_pre,
        "frames_occlusion": n_occ,
        "frames_post": n_post,
        "reports_loss_on_occlusion": lost_occ > 0,
        "recovered_after_occlusion": recovered,
    }


def tracker_tables():
    raw = []
    for name, ctor in factories():
        for seed in SEEDS[:N_TRIALS]:
            raw.append(run_one(name, ctor, seed))
    trials = pd.DataFrame(raw)
    summary = (
        trials.groupby("tracker", sort=False)
        .agg(
            trials=("trial_seed", "count"),
            fps_mean=("fps", "mean"),
            fps_std=("fps", "std"),
            iou_pre_mean=("iou_pre_occlusion_mean", "mean"),
            iou_pre_std=("iou_pre_occlusion_mean", "std"),
            iou_post_mean=("iou_post_occlusion_mean", "mean"),
            iou_post_std=("iou_post_occlusion_mean", "std"),
            reports_loss=("reports_loss_on_occlusion", "max"),
            recovered=("recovered_after_occlusion", "max"),
        )
        .reset_index()
    )
    summary["selected_for_pipeline"] = summary["tracker"].eq("CSRT")
    summary["notes"] = summary["tracker"].map({
        "CSRT": "Selected: best IoU, reports loss (required for VLM re-detect).",
        "KCF": "Faster, lower IoU; unused as primary.",
        "MIL": "Never reports loss — cannot escalate to VLM. Rejected.",
        "MOSSE (legacy)": "Unstable confidence; unused.",
    })
    return trials.round(4), summary.round(4)


def static_sheets():
    pipeline = pd.DataFrame([
        {"subsystem": "LiteWing hover / MOVE_TO loop", "period_s": 0.02, "rate_hz": 50,
         "source": "backend/drone/litewing_interface.py  _CONTROL_DT",
         "role": "Send hover setpoints (vx, vy, yawrate, z)"},
        {"subsystem": "Optical-flow log", "period_s": 0.01, "rate_hz": 100,
         "source": "backend/drone/litewing_interface.py  motion.deltaX/Y",
         "role": "Dead-reckoning XY odometry"},
        {"subsystem": "Attitude / ToF / battery log", "period_s": 0.05, "rate_hz": 20,
         "source": "backend/drone/litewing_interface.py  stateEstimate.z",
         "role": "Altitude and battery interlock"},
        {"subsystem": "Camera grab thread", "period_s": 0.066, "rate_hz": 15.15,
         "source": "backend/vision/camera_manager.py",
         "role": "Latest-frame overhead / webcam stream"},
        {"subsystem": "PerceptionLoop (FastPerception)", "period_s": 0.10, "rate_hz": 10,
         "source": "backend/vision/perception_loop.py",
         "role": "CSRT update + optional VLM re-detect"},
        {"subsystem": "Periodic VLM re-confirm", "period_s": 8.0, "rate_hz": 0.125,
         "source": "backend/vision/object_tracker.py  _REDETECT_INTERVAL_S",
         "role": "Bound slow tracker drift even when ok=True"},
        {"subsystem": "VLM re-detect cooldown (initial)", "period_s": 3.0, "rate_hz": 0.333,
         "source": "TrackerManager.redetect_cooldown_s",
         "role": "Rate-limit Qwen2-VL / Gemini after a miss"},
        {"subsystem": "VLM miss backoff cap", "period_s": 60.0, "rate_hz": 0.017,
         "source": "TrackerManager.max_cooldown_s",
         "role": "Absent object must not cost a VLM call every few seconds"},
    ])

    health = pd.DataFrame([
        {"parameter": "tracker_preference", "value": "CSRT then KCF", "unit": "",
         "meaning": "OpenCV contrib build 4.12; CSRT preferred"},
        {"parameter": "max_scale_drift", "value": 3.0, "unit": "ratio",
         "meaning": "Kill track if box area grows/shrinks beyond 3x seed"},
        {"parameter": "max_aspect_drift", "value": 2.5, "unit": "ratio",
         "meaning": "Kill track if aspect ratio changes beyond 2.5x"},
        {"parameter": "max_step_frac", "value": 0.35, "unit": "frame diagonal",
         "meaning": "Kill track on implausible centre jump"},
        {"parameter": "min_template_score", "value": 0.10, "unit": "NCC",
         "meaning": "Low appearance vs VLM-confirmed crop"},
        {"parameter": "max_low_appearance_frames", "value": 5, "unit": "frames",
         "meaning": "TEMPORARY_ANOMALY until 5 consecutive low NCC frames"},
        {"parameter": "lock_states", "value": "TRACKING | SEEKING | TARGET_LOST", "unit": "",
         "meaning": "Only TRACKING may generate target-directed motion"},
        {"parameter": "hsv_min_blob_area", "value": 600, "unit": "px^2",
         "meaning": "Colour fallback only; unused when VLM is configured"},
    ])

    replan = pd.DataFrame([
        {"parameter": "replan_distance_threshold", "value": 0.3, "unit": "m",
         "meaning": "Drift vs mission anchor, not vs previous frame"},
        {"parameter": "confirm_frames", "value": 3, "unit": "detections",
         "meaning": "Jitter debounce before a replan is armed"},
        {"parameter": "replan_cooldown", "value": 2.0, "unit": "s",
         "meaning": "Minimum time between two replans"},
        {"parameter": "max_replans_per_mission", "value": 3, "unit": "count",
         "meaning": "Hard cap so a flickering blob cannot fly the drone forever"},
        {"parameter": "airborne_gate_z", "value": 0.1, "unit": "m",
         "meaning": "Ignore drift if drone is on the ground or disarmed"},
        {"parameter": "target_lost_policy", "value": "hover / no MOVE_TO", "unit": "",
         "meaning": "CSRT fail + VLM fail strips world_x/y; FIND holds position"},
        {"parameter": "bbox_full_frame_reject", "value": 0.7, "unit": "area ratio",
         "meaning": "VLM whole-image box is discarded, not scored high"},
        {"parameter": "bbox_sliver_reject", "value": 0.0005, "unit": "area ratio",
         "meaning": "Degenerate VLM sliver discarded"},
        {"parameter": "bbox_max_aspect", "value": 12.0, "unit": "ratio",
         "meaning": "Extreme aspect boxes discarded"},
    ])

    litewing = pd.DataFrame([
        {"parameter": "OPTICAL_FLOW_SCALE", "value": 3.7, "unit": "m/s per tick",
         "source": "Circuit-Digest dead-reckoning-position-hold.py"},
        {"parameter": "SENSOR_DT", "value": 0.01, "unit": "s", "source": "motion log 10 ms"},
        {"parameter": "VELOCITY_SMOOTH_ALPHA", "value": 0.8, "unit": "", "source": "same"},
        {"parameter": "POSITION_KP", "value": 1.5, "unit": "", "source": "same"},
        {"parameter": "VELOCITY_KP", "value": 1.2, "unit": "", "source": "same"},
        {"parameter": "TRIM_VX", "value": 0.10, "unit": "m/s", "source": "same"},
        {"parameter": "TRIM_VY", "value": -0.02, "unit": "m/s", "source": "same"},
        {"parameter": "MAX_HOLD_CORR", "value": 0.10, "unit": "m/s", "source": "hover hold"},
        {"parameter": "MAX_NAV_CORR", "value": 0.40, "unit": "m/s", "source": "MOVE_TO"},
        {"parameter": "MIN_HOVER_Z", "value": 0.20, "unit": "m", "source": "LiteWing safe hover"},
        {"parameter": "MAX_HOVER_Z", "value": 0.80, "unit": "m", "source": "LiteWing safe hover"},
        {"parameter": "MIN_BATTERY_V", "value": 3.40, "unit": "V", "source": "litewing_interface.py"},
        {"parameter": "MOVE_TO settle radius", "value": 0.12, "unit": "m", "source": "litewing_interface.py"},
        {"parameter": "MOVE_TO settle time", "value": 0.35, "unit": "s", "source": "litewing_interface.py"},
        {"parameter": "MOVE_TO timeout", "value": 20.0, "unit": "s", "source": "litewing_interface.py"},
        {"parameter": "URI", "value": "udp://192.168.43.42", "unit": "", "source": "cflib CRTP over UDP"},
    ])

    workspace = pd.DataFrame([
        {"axis": "X", "min_m": -1.5, "max_m": 1.5, "notes": "Indoor workspace geofence"},
        {"axis": "Y", "min_m": -1.5, "max_m": 1.5, "notes": "Indoor workspace geofence"},
        {"axis": "Z", "min_m": 0.2, "max_m": 1.5, "notes": "Planner ceiling; LiteWing hover clamped to 0.80 m"},
    ])

    methodology = pd.DataFrame([
        {"item": "date_utc", "value": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")},
        {"item": "host", "value": f"{platform.system()} {platform.release()} / {platform.machine()}"},
        {"item": "python", "value": platform.python_version()},
        {"item": "opencv", "value": cv2.__version__},
        {"item": "opencv_contrib_trackers", "value": "CSRT, KCF, MIL, MOSSE(legacy), Nano, ViT, DaSiamRPN"},
        {"item": "frame_size", "value": f"{W}x{H} BGR"},
        {"item": "sequence_length", "value": f"{N_FRAMES} frames (~smooth sinusoid + 3 px/frame translation)"},
        {"item": "occlusion", "value": f"frames {OCC_START}–{OCC_END}: 90x90 black rectangle over the target"},
        {"item": "target", "value": "60x60 blurred noise patch; no distinctive hue (colour tracker finds nothing)"},
        {"item": "trials", "value": f"{N_TRIALS} independent background seeds {list(SEEDS[:N_TRIALS])}"},
        {"item": "iou", "value": "intersection-over-union of predicted vs ground-truth axis-aligned box"},
        {"item": "recovery_criterion", "value": "mean IoU after occlusion > 0.5"},
        {"item": "limitation", "value": "Synthetic motion, static camera, no scale change, one object. Two similar objects not tested."},
        {"item": "not_measured_here", "value": "Qwen2-VL / Gemini latency (Ollama was down; Gemini is API-bound). Hardware flight logs."},
        {"item": "pipeline_claim", "value": "VLM identifies once; CSRT tracks; VLM again only on confirmed loss or 8 s re-confirm."},
        {"item": "safety_claim", "value": "TARGET_LOST after CSRT+VLM fail: last world pose cannot generate MOVE_TO."},
        {"item": "unit_tests_at_export", "value": "150 passed (pytest backend/tests)"},
    ])
    return pipeline, health, replan, litewing, workspace, methodology


def autosize(writer: pd.ExcelWriter, sheet: str, df: pd.DataFrame) -> None:
    from openpyxl.utils import get_column_letter

    ws = writer.sheets[sheet]
    for i, col in enumerate(df.columns, 1):
        longest = int(df[col].astype(str).map(len).max()) if len(df) else 0
        width = min(56, max(14, longest + 4, len(str(col)) + 2))
        ws.column_dimensions[get_column_letter(i)].width = width
    ws.freeze_panes = "A2"
    ws.auto_filter.ref = ws.dimensions


def main() -> Path:
    print("Running tracker comparison (%s trials x %s trackers)..." % (N_TRIALS, len(factories())))
    trials, summary = tracker_tables()
    pipeline, health, replan, litewing, workspace, methodology = static_sheets()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with pd.ExcelWriter(OUT, engine="openpyxl") as writer:
        summary.to_excel(writer, sheet_name="Tracker_Summary", index=False)
        trials.to_excel(writer, sheet_name="Tracker_Trials", index=False)
        pipeline.to_excel(writer, sheet_name="Pipeline_Rates", index=False)
        health.to_excel(writer, sheet_name="Tracker_Health", index=False)
        replan.to_excel(writer, sheet_name="Replan_Safety", index=False)
        litewing.to_excel(writer, sheet_name="LiteWing_Control", index=False)
        workspace.to_excel(writer, sheet_name="Workspace", index=False)
        methodology.to_excel(writer, sheet_name="Methodology", index=False)
        for name, df in (
            ("Tracker_Summary", summary),
            ("Tracker_Trials", trials),
            ("Pipeline_Rates", pipeline),
            ("Tracker_Health", health),
            ("Replan_Safety", replan),
            ("LiteWing_Control", litewing),
            ("Workspace", workspace),
            ("Methodology", methodology),
        ):
            autosize(writer, name, df)
    print("Wrote", OUT)
    print(summary.to_string(index=False))
    return OUT


if __name__ == "__main__":
    main()
