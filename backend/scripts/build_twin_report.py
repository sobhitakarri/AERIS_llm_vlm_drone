"""
Builds the twin results report from the outputs of the test commands.

Reads   reports/twin_results/metrics/benchmark.json, replan.json
        cache/twin_validation/metrics_*.json, flight_*.png
        reports/twin_results/logs/*.log
Writes  reports/twin_results/README.md, metrics/*.csv, metrics/summary.json,
        images/path_caseNN.png, images/benchmark_errors.png

    py -3 backend/scripts/build_twin_report.py

Nothing is measured here: every number in the report is copied from a file a
test wrote. A missing input shows up as "not run", not as a number.
"""
import csv
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
OUT = REPO_ROOT / "reports" / "twin_results"
VAL = REPO_ROOT / "cache" / "twin_validation"

VALIDATION_RUNS = [("litewing_truth", "LiteWing, controller on ground truth"),
                   ("litewing_onboard", "LiteWing, onboard estimator"),
                   ("x500_onboard", "X500, onboard estimator")]


def video_md(rel: str, label: str) -> list:
    """Markdown for a test video. GitHub does not play an .mp4 from the
    repository inline, so the README shows an animated GIF preview that links
    to the full video. Without ffmpeg it is a plain link."""
    src = OUT / rel
    if not src.is_file():
        return []
    gif = OUT / "previews" / (src.stem + ".gif")
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg and (not gif.is_file() or gif.stat().st_mtime < src.stat().st_mtime):
        gif.parent.mkdir(parents=True, exist_ok=True)
        vf = "fps=5,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer:bayer_scale=4"
        subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", str(src), "-vf", vf, str(gif)])
    if gif.is_file():
        return [f"[![{label}](previews/{gif.name})]({rel})", "", f"Preview above; full video: [{rel}]({rel})", ""]
    return [f"Video: [{rel}]({rel})", ""]


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None


def scene_axes(ax, runtime: dict) -> None:
    room = (runtime.get("room") or {}).get("size_m", [6, 5, 3])
    ws = runtime.get("workspace") or {}
    ax.add_patch(Rectangle((-room[0] / 2, -room[1] / 2), room[0], room[1], fill=False, lw=1.5, color="#444"))
    if ws:
        ax.add_patch(Rectangle((ws["x"][0], ws["y"][0]), ws["x"][1] - ws["x"][0], ws["y"][1] - ws["y"][0],
                               fill=False, lw=0.8, ls="--", color="#999"))
    for name, o in ((runtime.get("ground_truth") or {}).get("objects") or {}).items():
        sx, sy = o["size_m"][0], o["size_m"][1]
        if abs(o.get("yaw_deg", 0)) % 180 > 45:
            sx, sy = sy, sx
        ax.add_patch(Rectangle((o["pos"][0] - sx / 2, o["pos"][1] - sy / 2), sx, sy, color="#c9d3df", ec="#7d8996", lw=0.6))
        ax.text(o["pos"][0], o["pos"][1], name.replace("_", "\n"), fontsize=5.5, ha="center", va="center", color="#39424e")
    home = (runtime.get("home") or {}).get("pos", [0, 0])
    ax.plot(home[0], home[1], "s", color="#222", ms=6)
    ax.text(home[0], home[1] - 0.16, "home", fontsize=6, ha="center", va="top")
    ax.set_xlim(-room[0] / 2 - 0.1, room[0] / 2 + 0.1)
    ax.set_ylim(-room[1] / 2 - 0.1, room[1] / 2 + 0.1)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def path_plot(r: dict, runtime: dict, dest: Path) -> None:
    fig, ax = plt.subplots(figsize=(6.4, 5.4), dpi=130)
    scene_axes(ax, runtime)
    pts = r.get("path") or []
    if pts:
        ax.plot([p[0] for p in pts], [p[1] for p in pts], "-", color="#1f77b4", lw=1.6, label="true flight path")
    if r.get("object_true"):
        ax.plot(*r["object_true"][:2], "x", color="#d62728", ms=10, mew=2, label="true object position")
    if r.get("hover_true"):
        ax.plot(*r["hover_true"][:2], "o", mfc="none", color="#2ca02c", ms=11, mew=2, label="true hover position")
    ax.set_title(f"Case {r['id']}: {r['prompt']}", fontsize=9)
    if pts or r.get("object_true"):
        ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(dest)
    plt.close(fig)


def error_plot(results: list, dest: Path) -> None:
    rows = [(f"case {r['id']}", r.get("target_error_cm", r.get("standoff_error_cm")), r.get("estimator_error_cm"))
            for r in results if r.get("target_error_cm", r.get("standoff_error_cm")) is not None]
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(7, 3.4), dpi=130)
    x = range(len(rows))
    ax.bar([i - 0.2 for i in x], [r[1] for r in rows], 0.4, label="position error at the target", color="#1f77b4")
    ax.bar([i + 0.2 for i in x], [r[2] or 0 for r in rows], 0.4, label="estimator error at that moment", color="#ff7f0e")
    ax.axhline(15, color="#d62728", lw=0.8, ls="--", label="pass limit (15 cm)")
    ax.set_xticks(list(x))
    ax.set_xticklabels([r[0] for r in rows])
    ax.set_ylabel("cm")
    ax.legend(fontsize=7)
    ax.set_title("Language missions scored against simulator ground truth", fontsize=9)
    fig.tight_layout()
    fig.savefig(dest)
    plt.close(fig)


def fmt(v, digits: int = 1) -> str:
    if v is None or v == "":
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def unit_test_line(log: Path) -> str:
    if not log.is_file():
        return "not run"
    m = re.findall(r"=+ (.*?) in [\d.]+s.* =+", log.read_text(encoding="utf-8", errors="replace"))
    return m[-1] if m else "no summary line in the log"


def main() -> int:
    runtime = load(REPO_ROOT / "cache" / "twin_runtime.json") or {}
    bench = load(OUT / "metrics" / "benchmark.json")
    replan = load(OUT / "metrics" / "replan.json")
    (OUT / "images").mkdir(parents=True, exist_ok=True)
    (OUT / "metrics").mkdir(parents=True, exist_ok=True)

    md = ["# AERIS digital twin: results and metrics", "",
          f"Generated {time.strftime('%Y-%m-%d %H:%M')} by `backend/scripts/build_twin_report.py` from the files in "
          "`metrics/` and `logs/`. Every number below was written by a test run; nothing is typed in by hand.", ""]
    summary = {"generated": time.strftime("%Y-%m-%d %H:%M:%S")}

    # ------------------------------------------------------------ unit tests
    line = unit_test_line(OUT / "logs" / "01_unit_tests.log")
    summary["unit_tests"] = line
    md += ["## 1. Unit tests", "", "`py -3 -m pytest backend/tests -v` (no simulator needed)", "",
           f"Result: **{line}**. Full output: [logs/01_unit_tests.log](logs/01_unit_tests.log)", ""]

    # ------------------------------------------------------------ validation
    md += ["## 2. Twin flight validation (MATLAB only, no Python, no language model)", "",
           "`validate_aeris_twin` flies take-off, a square of four waypoints, a 90 degree yaw step and a landing on the "
           "Simscape Multibody plant. Log: [logs/02_twin_validation.log](logs/02_twin_validation.log)", ""]
    keys = [("hover_throttle", "Hover throttle", 2), ("hover_rotor_rpm", "Hover rotor speed (rpm)", 0),
            ("takeoff_settle_s", "Take-off settle to 5 cm (s)", 2), ("takeoff_overshoot_cm", "Take-off overshoot (cm)", 1),
            ("tracking_rmse_cm", "Tracking RMSE (cm)", 1), ("tracking_max_cm", "Tracking max error (cm)", 1),
            ("waypoint_error_cm", "Waypoint arrival error (cm)", 1), ("max_tilt_deg", "Max tilt (deg)", 1),
            ("yaw_step_final_deg", "Yaw after 90 deg step (deg)", 1), ("estimator_rmse_cm", "Estimator RMSE (cm)", 1),
            ("estimator_final_xy_cm", "Estimator final xy drift (cm)", 1), ("landed", "Landed", 0),
            ("landing_xy_error_cm", "Landing xy error (cm)", 1), ("battery_used_pct", "Battery used (%)", 2),
            ("min_voltage", "Minimum voltage (V)", 2), ("realtime_factor", "Real-time factor, no rendering", 2)]
    runs = [(tag, title, load(VAL / f"metrics_{tag}.json")) for tag, title in VALIDATION_RUNS]
    md += ["| Metric | " + " | ".join(t for _, t, _ in runs) + " |", "|---|" + "---|" * len(runs)]
    val_rows = []
    for key, label, digits in keys:
        cells = []
        for _, _, m in runs:
            v = (m or {}).get(key)
            if m is None:
                cells.append("not run")
            elif isinstance(v, list):
                cells.append(" / ".join(f"{x:.{digits}f}" for x in v))
            else:
                cells.append(fmt(v, digits))
        md.append(f"| {label} | " + " | ".join(cells) + " |")
        val_rows.append([label] + cells)
    md.append("")
    with open(OUT / "metrics" / "validation.csv", "w", newline="", encoding="utf-8") as fh:
        csv.writer(fh).writerows([["metric"] + [t for _, t, _ in runs]] + val_rows)
    summary["validation"] = {tag: m for tag, _, m in runs}
    for tag, title, m in runs:
        src = VAL / f"flight_{tag}.png"
        if m is not None:
            shutil.copyfile(VAL / f"metrics_{tag}.json", OUT / "metrics" / f"validation_{tag}.json")
        if src.is_file():
            shutil.copyfile(src, OUT / "images" / f"validation_{tag}.png")
            md += [f"**{title}**", "", f"![{title}](images/validation_{tag}.png)", ""]

    # ------------------------------------------------------------- link test
    md += ["## 3. Link check (Python to twin, no language model)", "",
           "`py -3 backend/scripts/twin_smoke_test.py --record` checks telemetry, the camera streams, the marker "
           "calibration of the overhead camera and a short scripted flight. The backend is not running, so the "
           "vehicle flies on its onboard estimator alone, with no camera fixes.", ""]
    smoke = OUT / "logs" / "03_link_smoke_test.log"
    if smoke.is_file():
        lines = [ln for ln in smoke.read_text(encoding="utf-8", errors="replace").splitlines() if ln and not ln.startswith("[")]
        md += ["```text", *lines, "```", ""]
        summary["link_check"] = smoke.read_text(encoding="utf-8", errors="replace").strip().splitlines()[-1]
    else:
        md += ["Not run.", ""]
    md += video_md("videos/case00_link_smoke_flight.mp4", "Link check flight")
    if (OUT / "images" / "case00_link_smoke_flight.jpg").is_file():
        md += ["![Link check flight](images/case00_link_smoke_flight.jpg)", ""]
    cams = [c for c in ("overhead", "fpv", "chase") if (OUT / "images" / f"camera_{c}.jpg").is_file()]
    if cams:
        extra = [(n, f) for n, f in (("overhead depth", "camera_overhead_depth.jpg"),
                                     ("depth proposals and a grounded target", "perception_depth_proposals.jpg"))
                 if (OUT / "images" / f).is_file()]
        heads = cams + [n for n, _ in extra]
        md += ["| " + " | ".join(heads) + " |", "|" + "---|" * len(heads),
               "| " + " | ".join([f"![{c}](images/camera_{c}.jpg)" for c in cams] + [f"![{n}](images/{f})" for n, f in extra]) + " |", ""]

    # -------------------------------------------------------------- missions
    md += ["## 4. Language missions on the twin", "",
           "`py -3 backend/scripts/run_twin_benchmark.py --record` sends each prompt to the running backend. "
           "A case passes when the vehicle's true position while hovering is within 15 cm of where the prompt "
           "asks it to be relative to the object's true position (and within 15 degrees of facing it, for "
           "inspection cases). Ground truth is used for scoring only; the planner and the perception stack never "
           "see it.", ""]
    if bench:
        meta, results = bench["meta"], bench["results"]
        summary["benchmark"] = meta
        reruns = [str(r["id"]) for r in results if r.get("rerun")]
        md += [f"Run {meta['created']}, airframe {meta.get('airframe')}, scene {meta.get('scene')}, model "
               f"{meta.get('model')}: **{meta['passed']}/{meta['total']} passed**. Each case has "
               f"{'one scored flight' if max(r.get('repeat', 1) for r in results) == 1 else 'several scored flights'}."
               + (f" Cases {', '.join(reruns)} were re-run; section 7 says why." if reruns else ""), "",
               "Log: [logs/04_language_missions.log](logs/04_language_missions.log), "
               "data: [metrics/benchmark.json](metrics/benchmark.json), [metrics/benchmark.csv](metrics/benchmark.csv)", ""]
        cols = [("id", "Case"), ("prompt", "Prompt"), ("status", "Status"), ("passed", "Pass"),
                ("target_error_cm", "Target error (cm)"), ("standoff_error_cm", "Stand-off error (cm)"),
                ("heading_error_deg", "Heading error (deg)"), ("estimator_error_cm", "Estimator error (cm)"),
                ("clearance_m", "Clearance (m)"), ("sim_time_s", "Flight time (s, sim)"), ("path_m", "Path (m)"),
                ("plan_latency_s", "Plan latency (s, wall)"), ("replans", "Replans"), ("rerun", "Re-run")]
        md += ["| " + " | ".join(c[1] for c in cols) + " |", "|" + "---|" * len(cols)]
        for r in results:
            md.append("| " + " | ".join(fmt(r.get(k), 2 if k.endswith("_m") else 1) for k, _ in cols) + " |")
        md.append("")
        with open(OUT / "metrics" / "benchmark.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow([k for k, _ in cols] + ["skills", "reasoning", "error"])
            for r in results:
                w.writerow([r.get(k, "") for k, _ in cols] + [" ".join(r.get("skills", [])), r.get("reasoning", ""), r.get("error", "")])
        error_plot(results, OUT / "images" / "benchmark_errors.png")
        if (OUT / "images" / "benchmark_errors.png").is_file():
            md += ["![Errors per case](images/benchmark_errors.png)", ""]
        for r in results:
            md += [f"### Case {r['id']}: \"{r['prompt']}\"", "",
                   f"Result: **{'PASS' if r['passed'] else 'FAIL'}** ({r.get('status')}). "
                   f"Plan: {' > '.join(r.get('skills', [])) or 'none'}.", ""]
            if r.get("reasoning"):
                md += [f"> {str(r['reasoning']).strip()}", ""]
            if r.get("error"):
                md += [f"Error reported: {r['error']}", ""]
            if r.get("video"):
                md += video_md(r["video"], f"Case {r['id']}")
            imgs = []
            if r.get("still") and (OUT / r["still"]).is_file():
                imgs.append(f"![Case {r['id']} still]({r['still']})")
            if r.get("path"):
                path_plot(r, runtime, OUT / "images" / f"path_case{r['id']:02d}.png")
                imgs.append(f"![Case {r['id']} path](images/path_case{r['id']:02d}.png)")
            md += imgs and ["\n\n".join(imgs), ""] or []
    else:
        md += ["Not run.", ""]

    # ---------------------------------------------------------------- replan
    md += ["## 5. Replanning when the target moves", "",
           "`py -3 backend/scripts/twin_replan_test.py --record` starts a hover-above mission, moves the target "
           "while the vehicle is on station and checks that the vehicle ends up over the object's new true position.", ""]
    if replan:
        summary["replan"] = replan
        md += ["| Target | Moved (cm) | Replans | Final error to new true position (cm) | Pass |", "|---|---|---|---|---|",
               f"| {replan['target']} | {fmt(replan['moved_cm'])} | {replan['replans']} | {fmt(replan['final_error_cm'])} | "
               f"{fmt(replan['passed'])} |", "",
               "Log: [logs/05_replan_test.log](logs/05_replan_test.log)", ""]
        md += video_md(replan["video"], "Replan test")
        if (OUT / replan["still"]).is_file():
            md += [f"![Replan test]({replan['still']})", ""]
    else:
        md += ["Not run.", ""]

    # ------------------------------------------------------------- dashboard
    shots = sorted((OUT / "images").glob("dashboard*.png"))
    if shots:
        md += ["## 6. Ground station", ""] + [f"![{s.stem}](images/{s.name})\n" for s in shots]

    notes = OUT / "NOTES.md"
    if notes.is_file():
        md += [notes.read_text(encoding="utf-8").strip(), ""]

    (OUT / "README.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    (OUT / "metrics" / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Report written to {OUT / 'README.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
