"""The statistics behind the evaluation tables, checked against known values."""
import math

import pytest

from backend.scripts import analyze_trials as an
from backend.scripts import run_twin_benchmark as bench


def test_wilson_interval_matches_published_values():
    lo, hi = an.wilson(8, 10)
    assert lo == pytest.approx(0.490, abs=1e-3) and hi == pytest.approx(0.943, abs=1e-3)
    lo, hi = an.wilson(0, 20)
    assert lo == 0.0 and hi == pytest.approx(0.161, abs=1e-3)
    assert all(math.isnan(v) for v in an.wilson(0, 0))


def test_exact_mcnemar():
    assert an.mcnemar_exact(0, 0) == 1.0
    assert an.mcnemar_exact(0, 6) == pytest.approx(2 / 64)
    assert an.mcnemar_exact(3, 3) == 1.0
    assert an.mcnemar_exact(1, 9) == pytest.approx(2 * 11 / 1024)


def test_median_iqr():
    assert an.median_iqr([None, 4.0]) == (4.0, 4.0, 4.0, 1)
    med, q1, q3, n = an.median_iqr([1.0, 2.0, 3.0, 4.0, 5.0])
    assert (med, q1, q3, n) == (3.0, 2.0, 4.0, 5)


def _trial(mode, seed, passed, dist="none", scenario=2, **kw):
    return {"run_id": "r1", "seed": seed, "mode": mode, "scenario": scenario, "kind": kw.pop("kind", "above"),
            "disturbance": dist, "prompt": f"p{scenario}", "passed": passed, "status": kw.pop("status", "done"),
            "target_error_cm": kw.pop("err", 3.0), "plan_latency_s": 4.0, **kw}


def test_end_to_end_on_harness_csv(tmp_path):
    rows = []
    for seed in range(10):
        rows.append(_trial("full", seed, True, "move_target", replan_latency_s=1.5, replans=1))
        rows.append(_trial("open_loop", seed, seed < 2, "move_target", err=30.0))
    rows.append(_trial("full", 0, True, scenario=8, kind="absent", status="refused"))
    rows.append(_trial("no_depth_check", 0, False, scenario=8, kind="absent", status="done"))
    rows.append(_trial("full", 0, True, "remove_target", status="failed"))
    rows.append(_trial("open_loop", 0, False, "remove_target", stale_hover=True, n_clipped=2))
    path = tmp_path / "trials.csv"
    bench.write_rows(path, rows, append=False)          # the harness's own writer

    data = an.load([path])
    cells = {(c["mode"], c["disturbance"]): c for c in an.cells(data, False)}
    full = cells[("full", "move_target")]
    assert full["n"] == 10 and full["passed"] == 10 and full["replan_latency_median_s"] == 1.5
    ol = cells[("open_loop", "move_target")]
    assert ol["passed"] == 2 and ol["error_median_cm"] == 30.0

    pair = next(p for p in an.paired(data, "full") if p["mode"] == "open_loop" and p["disturbance"] == "move_target")
    assert (pair["pairs"], pair["ref_only"], pair["mode_only"]) == (10, 8, 0)
    assert pair["p"] == pytest.approx(an.mcnemar_exact(8, 0))

    safe = {s["mode"]: s for s in an.safety(data)}
    assert safe["no_depth_check"]["absent_object_flown"] == 1 and safe["full"]["absent_object_flown"] == 0
    assert safe["open_loop"]["stale_target_hover"] == 1 and safe["open_loop"]["clipped_requests"] == 2

    md = an.report(data, an.cells(data, False), an.paired(data, "full"), an.safety(data), "full", [str(path)])
    assert "| open_loop | move_target | 10 | 100% | 20% | 8 | 0 | 0.008 |" in md


def test_model_outages_are_flagged_and_left_out(tmp_path):
    outage = {"kind": "vlm", "attempts": [{"error": "429 RESOURCE_EXHAUSTED", "raw": None}]}
    answered = {"kind": "vlm", "attempts": [{"error": None, "raw": '{"label": "none"}'}]}
    assert bench.trace_columns({"model_calls": [outage]}, {}, None)["model_unavailable"] is True
    assert bench.trace_columns({"model_calls": [outage, answered]}, {}, None)["model_unavailable"] is False
    rows = [_trial("full", 0, True), _trial("open_loop", 0, False, status="refused", model_unavailable=True)]
    path = tmp_path / "t.csv"
    bench.write_rows(path, rows, append=False)
    import sys
    from unittest import mock
    with mock.patch.object(sys, "argv", ["x", str(path), "--out", str(tmp_path)]):
        assert an.main() == 0
    md = (tmp_path / "analysis.md").read_text(encoding="utf-8")
    assert "1 trial(s) in which every model request failed" in md and "| open_loop |" not in md.split("## Paired")[0]
