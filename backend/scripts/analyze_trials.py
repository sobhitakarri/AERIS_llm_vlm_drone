"""
Statistics for the evaluation matrix written by run_twin_benchmark.py.

    py -3 backend/scripts/analyze_trials.py
    py -3 backend/scripts/analyze_trials.py reports/twin_results/metrics/trials.csv --reference full
    py -3 backend/scripts/analyze_trials.py a.csv b.csv --by-scenario

Per mode and disturbance: success rate with a Wilson 95% interval, median and
interquartile range of position error and latencies, and replanning counts.
Each mode is compared with the reference mode on the same trials (same seed,
scenario, disturbance and wording) with an exact McNemar test, which uses only
the trials where the two disagree. A safety table counts what reached the
executor that should not have.

Writes <metrics>/analysis.md and analysis_cells.csv next to the first CSV.
"""
import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CSV = REPO_ROOT / "reports" / "twin_results" / "metrics" / "trials.csv"
Z95 = 1.959964


# ------------------------------------------------------------------- stats
def wilson(k: int, n: int, z: float = Z95) -> tuple:
    """Wilson score interval for k successes in n trials."""
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def mcnemar_exact(b: int, c: int) -> float:
    """Two-sided exact McNemar p-value from the discordant pair counts."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def median_iqr(values: list) -> tuple:
    vals = sorted(v for v in values if v is not None and not math.isnan(v))
    if not vals:
        return (None, None, None, 0)
    if len(vals) == 1:
        return (vals[0], vals[0], vals[0], 1)
    q1, med, q3 = statistics.quantiles(vals, n=4, method="inclusive")
    return (med, q1, q3, len(vals))


# -------------------------------------------------------------------- data
def _num(v):
    if v in (None, ""):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _bool(v):
    return str(v).strip().lower() in ("true", "1", "yes")


def load(paths: list) -> list:
    rows = []
    for p in paths:
        with open(p, newline="", encoding="utf-8") as f:
            rows.extend(csv.DictReader(f))
    for r in rows:
        r["passed"] = _bool(r.get("passed"))
        # One position error per trial, whichever the task defines.
        r["position_error_cm"] = _num(r.get("target_error_cm"))
        if r["position_error_cm"] is None:
            r["position_error_cm"] = _num(r.get("standoff_error_cm"))
    return rows


def pair_key(r: dict) -> tuple:
    return (r.get("seed"), r.get("scenario"), r.get("disturbance"), r.get("prompt"))


# ---------------------------------------------------------------- analysis
def cells(rows: list, by_scenario: bool) -> list:
    groups = {}
    for r in rows:
        key = (r["mode"], r["disturbance"]) + ((r["scenario"],) if by_scenario else ())
        groups.setdefault(key, []).append(r)
    out = []
    for key, rs in groups.items():
        n, k = len(rs), sum(r["passed"] for r in rs)
        lo, hi = wilson(k, n)
        err = median_iqr([r["position_error_cm"] for r in rs])
        lat = median_iqr([_num(r.get("plan_latency_s")) for r in rs])
        rep = median_iqr([_num(r.get("replan_latency_s")) for r in rs])
        out.append({
            "mode": key[0], "disturbance": key[1], "scenario": key[2] if by_scenario else "",
            "n": n, "passed": k, "rate": k / n, "ci_low": lo, "ci_high": hi,
            "error_median_cm": err[0], "error_q1_cm": err[1], "error_q3_cm": err[2], "error_n": err[3],
            "plan_latency_median_s": lat[0], "plan_latency_q1_s": lat[1], "plan_latency_q3_s": lat[2],
            "replan_latency_median_s": rep[0], "replan_latency_n": rep[3],
            "replans_mean": statistics.fmean([_num(r.get("replans")) or 0.0 for r in rs]),
            "cache_hit_rate": sum(_bool(r.get("cache_hit")) for r in rs) / n,
            "malformed_per_trial": statistics.fmean([_num(r.get("n_malformed")) or 0.0 for r in rs]),
            # Among trials with a look-alike in the scene: flown to the look-alike.
            "wrong_object": sum(_bool(r.get("wrong_object")) for r in rs),
            "wrong_object_n": sum(r.get("wrong_object") not in (None, "") for r in rs),
        })
    order = {m: i for i, m in enumerate(dict.fromkeys(r["mode"] for r in rows))}
    return sorted(out, key=lambda c: (order[c["mode"]], c["disturbance"], str(c["scenario"])))


def paired(rows: list, reference: str) -> list:
    """Each mode against the reference on the same trials, per disturbance."""
    ref = {}
    for r in rows:
        if r["mode"] == reference:
            ref[pair_key(r)] = r["passed"]
    out = []
    modes = [m for m in dict.fromkeys(r["mode"] for r in rows) if m != reference]
    for mode in modes:
        for dist in dict.fromkeys(r["disturbance"] for r in rows):
            both = [(ref[pair_key(r)], r["passed"]) for r in rows
                    if r["mode"] == mode and r["disturbance"] == dist and pair_key(r) in ref]
            if not both:
                continue
            b = sum(1 for a, o in both if a and not o)       # reference passed, mode failed
            c = sum(1 for a, o in both if o and not a)       # mode passed, reference failed
            out.append({
                "mode": mode, "disturbance": dist, "pairs": len(both),
                "ref_rate": sum(a for a, _ in both) / len(both), "mode_rate": sum(o for _, o in both) / len(both),
                "ref_only": b, "mode_only": c, "p": mcnemar_exact(b, c),
            })
    return out


def safety(rows: list) -> list:
    out = []
    for mode in dict.fromkeys(r["mode"] for r in rows):
        rs = [r for r in rows if r["mode"] == mode]
        absent = [r for r in rs if r.get("kind") == "absent"]
        removed = [r for r in rs if r.get("disturbance") == "remove_target"]
        out.append({
            "mode": mode, "trials": len(rs),
            "validator_rejected": sum(r.get("validator_valid") == "False" for r in rs),
            "flown_despite_validator": sum(_bool(r.get("validator_overridden")) for r in rs),
            "clipped_requests": int(sum(_num(r.get("n_clipped")) or 0 for r in rs)),
            "absent_object_flown": sum(r.get("status") not in ("refused", "") for r in absent),
            "absent_trials": len(absent),
            "stale_target_hover": sum(_bool(r.get("stale_hover")) for r in removed),
            "removed_target_completed": sum(r.get("status") == "done" for r in removed),
            "removed_trials": len(removed),
            "targets_lost_events": int(sum(_num(r.get("targets_lost")) or 0 for r in rs)),
        })
    return out


# ------------------------------------------------------------------ report
def _f(v, digits=1, pct=False):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "–"
    return f"{100 * v:.{digits}f}%" if pct else f"{v:.{digits}f}"


def _p(p: float) -> str:
    return "<0.001" if p < 0.001 else f"{p:.3f}"


def report(rows: list, cell_rows: list, pairs: list, safe: list, reference: str, sources: list) -> str:
    runs = sorted({r.get("run_id") for r in rows if r.get("run_id")})
    lines = [
        "# Evaluation analysis\n",
        f"{len(rows)} trials from {len(runs)} run(s): {', '.join(runs) or '–'}. Sources: "
        + ", ".join(Path(s).name for s in sources) + ".\n",
        "Success rates carry Wilson 95% intervals. Errors and latencies are median [IQR]. "
        "Position error is the hover error for hover tasks and the stand-off error for view tasks.\n",
        "## Success by mode and disturbance\n",
        "| mode | disturbance" + (" | scenario" if any(c["scenario"] for c in cell_rows) else "")
        + " | passed / n | rate [95% CI] | position error cm | plan latency s | replan latency s | replans/trial | wrong object | cache hits |",
    ]
    by_sc = any(c["scenario"] for c in cell_rows)
    lines.append("|---|---|" + ("---|" if by_sc else "") + "---|---|---|---|---|---|---|---|")
    for c in cell_rows:
        lines.append(
            f"| {c['mode']} | {c['disturbance']}" + (f" | {c['scenario']}" if by_sc else "")
            + f" | {c['passed']}/{c['n']} | {_f(c['rate'], 0, True)} [{_f(c['ci_low'], 0, True)}, {_f(c['ci_high'], 0, True)}]"
            + f" | {_f(c['error_median_cm'])} [{_f(c['error_q1_cm'])}, {_f(c['error_q3_cm'])}]"
            + f" | {_f(c['plan_latency_median_s'], 2)} [{_f(c['plan_latency_q1_s'], 2)}, {_f(c['plan_latency_q3_s'], 2)}]"
            + f" | {_f(c['replan_latency_median_s'], 2)} | {_f(c['replans_mean'], 2)}"
            + (f" | {c['wrong_object']}/{c['wrong_object_n']}" if c["wrong_object_n"] else " | –")
            + f" | {_f(c['cache_hit_rate'], 0, True)} |"
        )
    lines += [
        f"\n## Paired comparison with `{reference}`\n",
        "Same trials in both modes. `ref only`: the reference passed and this mode failed; `mode only`: the reverse. "
        "Exact two-sided McNemar test on those discordant pairs.\n",
        "| mode | disturbance | pairs | reference rate | mode rate | ref only | mode only | p |",
        "|---|---|---|---|---|---|---|---|",
    ]
    if not pairs:
        lines.append(f"| – | – | 0 | – | – | – | – | no `{reference}` trials to pair with |")
    for p in pairs:
        lines.append(f"| {p['mode']} | {p['disturbance']} | {p['pairs']} | {_f(p['ref_rate'], 0, True)} | "
                     f"{_f(p['mode_rate'], 0, True)} | {p['ref_only']} | {p['mode_only']} | {_p(p['p'])} |")
    lines += [
        "\n## Safety\n",
        "`clipped` counts coordinates outside the flight envelope that were clamped before the validator saw "
        "the plan. `absent flown`: a mission flown for an object that does not exist. `stale hover`: holding "
        "station where a removed target used to be.\n",
        "| mode | trials | validator rejected | flown despite validator | clipped | absent flown | stale hover | removed target, mission completed | target-lost events |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for s in safe:
        lines.append(
            f"| {s['mode']} | {s['trials']} | {s['validator_rejected']} | {s['flown_despite_validator']} | "
            f"{s['clipped_requests']} | {s['absent_object_flown']}/{s['absent_trials']} | "
            f"{s['stale_target_hover']}/{s['removed_trials']} | {s['removed_target_completed']}/{s['removed_trials']} | "
            f"{s['targets_lost_events']} |"
        )
    small = [c for c in cell_rows if c["n"] < 10]
    if small:
        lines.append(f"\n{len(small)} cell(s) have fewer than 10 trials; their intervals are wide.")
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="*", help=f"trial CSVs (default {DEFAULT_CSV})")
    ap.add_argument("--reference", default="full", help="mode the others are compared with")
    ap.add_argument("--by-scenario", action="store_true", help="split the success table by scenario")
    ap.add_argument("--include-outages", action="store_true",
                    help="keep trials in which every model request failed (left out by default)")
    ap.add_argument("--out", help="output folder (default: next to the first CSV)")
    args = ap.parse_args()
    paths = [Path(p) for p in args.csv] or [DEFAULT_CSV]
    missing = [p for p in paths if not p.is_file()]
    if missing:
        print("Not found:", ", ".join(map(str, missing)))
        return 1
    rows = load(paths)
    outages = [r for r in rows if _bool(r.get("model_unavailable"))]
    if outages and not args.include_outages:
        rows = [r for r in rows if not _bool(r.get("model_unavailable"))]
    if not rows:
        print("No trials in the input.")
        return 1
    cell_rows = cells(rows, args.by_scenario)
    pairs = paired(rows, args.reference)
    safe = safety(rows)
    out = Path(args.out) if args.out else paths[0].parent
    out.mkdir(parents=True, exist_ok=True)
    md = report(rows, cell_rows, pairs, safe, args.reference, [str(p) for p in paths])
    if outages:
        md += (f"\n{len(outages)} trial(s) in which every model request failed (quota or overload) were "
               + ("kept" if args.include_outages else "left out; `--include-outages` keeps them") + ".\n")
    (out / "analysis.md").write_text(md, encoding="utf-8")
    with open(out / "analysis_cells.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(cell_rows[0]))
        w.writeheader()
        w.writerows(cell_rows)
    print(f"{len(rows)} trials, {len(cell_rows)} cells. Written {out / 'analysis.md'} and analysis_cells.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
