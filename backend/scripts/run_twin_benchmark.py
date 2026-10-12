"""
Flight benchmark against the digital twin.

Sends natural-language prompts to a running backend (`python -m backend.main
--drone twin`) and scores each mission against the twin's ground truth:
where the vehicle truly was while it hovered, versus where the object the
prompt refers to truly is. The ground truth is used here and nowhere else.

    py -3 backend/scripts/run_twin_benchmark.py
    py -3 backend/scripts/run_twin_benchmark.py --only 1 3 --repeats 3
    py -3 backend/scripts/run_twin_benchmark.py --record

Results: <out>/metrics/benchmark.json and benchmark.md. With --record, one
video and one still per test case in <out>/videos and <out>/images.
"""
import argparse
import asyncio
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
DEFAULT_OUT = REPO_ROOT / "reports" / "twin_results"

# `expect` names the scene object the prompt refers to, for scoring only.
# kind: above = hover over it, view = stand off and face it, shape = no target.
SCENARIOS = [
    {"id": 1, "prompt": "Find the cardboard box and hover above it", "expect": "cardboard_box", "kind": "above"},
    {"id": 2, "prompt": "Find the red bottle and hover above it for 4 seconds", "expect": "red_bottle", "kind": "above"},
    {"id": 3, "prompt": "Inspect the blue cube from 0.5 m away", "expect": "blue_cube", "kind": "view", "standoff": 0.5},
    {"id": 4, "prompt": "Find the steel drum and hover above its center", "expect": "steel_drum", "kind": "above"},
    {"id": 5, "prompt": "Go to the yellow crate", "expect": "yellow_crate", "kind": "above"},
    {"id": 6, "prompt": "Find the traffic cone and hover facing it", "expect": "traffic_cone", "kind": "view", "standoff": 0.5},
    {"id": 7, "prompt": "Take off to 0.8 m and fly in a square", "expect": None, "kind": "shape"},
    {"id": 8, "prompt": "Find the purple elephant and hover above it", "expect": None, "kind": "absent"},
]


def api(base: str, path: str, body: dict = None, timeout: float = 180.0) -> dict:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method="POST" if body is not None or path in POSTS else "GET",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


POSTS = {"/api/home", "/api/land", "/api/abort"}


def slug(scenario: dict) -> str:
    words = "".join(c if c.isalnum() else " " for c in scenario["prompt"].lower()).split()
    return f"case{scenario['id']:02d}_" + "_".join(words[:6])


async def fly(base: str, ws_url: str, scenario: dict, truth_objects: dict, rec=None, still=None) -> dict:
    import websockets

    result = {"id": scenario["id"], "prompt": scenario["prompt"], "kind": scenario["kind"]}
    t_wall = time.time()
    reply = await asyncio.to_thread(api, base, "/api/command", {"command": scenario["prompt"]})
    result["plan_latency_s"] = round(time.time() - t_wall, 2)
    result["accepted"] = bool(reply.get("success"))
    result["reasoning"] = reply.get("reasoning") or reply.get("error") or "; ".join(reply.get("errors", []))
    result["skills"] = [s["skill"] for s in reply.get("skills", [])]
    if not result["accepted"]:
        result["status"] = "refused"
        result["passed"] = scenario["kind"] == "absent"     # refusing an absent target is the right answer
        if rec:
            rec.note = "REFUSED: " + str(result["reasoning"])[:34]
            await asyncio.sleep(4.0)
            rec.save_still(still)
        return result

    samples, hover, path_len, last = [], [], 0.0, None
    sim_t0, hover_t0, still_done = None, None, False
    async with websockets.connect(ws_url) as ws:
        deadline = time.time() + 300
        seen_running = False
        while time.time() < deadline:
            m = json.loads(await ws.recv())
            mon, twin = m.get("monitor") or {}, m.get("twin") or {}
            tr = twin.get("truth")
            if tr:
                p = (tr["x"], tr["y"], tr["z"])
                if last is not None:
                    path_len += math.dist(p, last)
                last = p
                samples.append(p)
                if mon.get("current_skill") == "HOVER" and mon.get("status") == "running":
                    hover.append((p, tr.get("yaw", 0.0), m["state"]["x"], m["state"]["y"], m["state"]["z"]))
                    hover_t0 = hover_t0 if hover_t0 is not None else twin.get("sim_time") or 0.0
                    if rec and not still_done and (twin.get("sim_time") or 0.0) - hover_t0 > 1.5:
                        still_done = rec.save_still(still)
                else:
                    hover_t0 = None
            if sim_t0 is None and twin.get("sim_time") is not None:
                sim_t0 = twin["sim_time"]
            sim_t = twin.get("sim_time")
            if mon.get("status") == "running":
                seen_running = True
            elif seen_running or time.time() - t_wall > 8:
                result["status"] = mon.get("status")
                result["error"] = m.get("last_error")
                result["replans"] = m.get("replan_count", 0)
                break
        else:
            result["status"] = "timeout"
    result["sim_time_s"] = round((sim_t or 0) - (sim_t0 or 0), 1)
    result["path_m"] = round(path_len, 2)
    result["path"] = [[round(c, 3) for c in p] for p in samples[::5]]
    if rec and not still_done:
        rec.save_still(still)

    ok = result["status"] == "done"
    if hover:
        (hx, hy, hz), hyaw, ex, ey, ez = hover[-1]
        result["estimator_error_cm"] = round(100 * math.dist((hx, hy, hz), (ex, ey, ez)), 1)
        result["hover_true"] = [round(hx, 3), round(hy, 3), round(hz, 3)]
        obj = truth_objects.get(scenario.get("expect") or "")
        if obj:
            result["object_true"] = [round(c, 3) for c in obj["pos"]]
            ox, oy = obj["pos"][0], obj["pos"][1]
            top = obj["pos"][2] + obj["size_m"][2]
            dist = math.hypot(hx - ox, hy - oy)
            if scenario["kind"] == "above":
                result["target_error_cm"] = round(100 * dist, 1)
                result["clearance_m"] = round(hz - top, 2)
                ok = ok and dist < 0.15 and hz > top
            elif scenario["kind"] == "view":
                want = scenario.get("standoff", 0.5)
                bearing = math.degrees(math.atan2(oy - hy, ox - hx))
                result["standoff_error_cm"] = round(100 * abs(dist - want), 1)
                result["heading_error_deg"] = round(abs((hyaw - bearing + 180) % 360 - 180), 1)
                ok = ok and abs(dist - want) < 0.15 and result["heading_error_deg"] < 15
    if scenario["kind"] == "absent":
        ok = False                                           # a mission was flown to something that is not there
    result["passed"] = bool(ok)
    return result


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8000")
    ap.add_argument("--only", type=int, nargs="*", help="scenario ids to run")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="results folder")
    ap.add_argument("--record", action="store_true", help="record a video and a still per test case")
    ap.add_argument("--merge", action="store_true", help="with --only: replace those cases in the existing results, keep the rest")
    args = ap.parse_args()
    out = Path(args.out)
    ws_url = args.base.replace("http", "ws") + "/ws/telemetry"

    cfg = api(args.base, "/api/config")
    if cfg.get("drone_backend") != "twin" or not cfg.get("drone_connected"):
        print("The backend is not connected to the digital twin.")
        return 1
    from backend.drone.twin_interface import load_runtime
    truth_objects = (load_runtime().get("ground_truth") or {}).get("objects") or {}

    chosen = [s for s in SCENARIOS if not args.only or s["id"] in args.only]
    results = []
    for rep in range(args.repeats):
        for sc in chosen:
            print(f"[{sc['id']}] {sc['prompt']}", flush=True)
            rec, still, name = None, None, slug(sc) + (f"_r{rep + 1}" if args.repeats > 1 else "")
            if args.record:
                from backend.scripts.twin_recorder import HttpSource, Recorder
                still = out / "images" / f"{name}.jpg"
                rec = Recorder(out / "videos" / f"{name}.mp4", f"Case {sc['id']}", HttpSource(args.base),
                               caption=f'"{sc["prompt"]}"').start()
            r = await fly(args.base, ws_url, sc, truth_objects, rec, still)
            r["repeat"] = rep + 1
            results.append(r)
            print("     ", "PASS" if r["passed"] else "FAIL", {k: v for k, v in r.items() if k.endswith(("_cm", "_deg", "_m")) or k in ("status", "error")}, flush=True)
            if r["accepted"]:
                if rec:
                    rec.note = "returning home"
                home = await asyncio.to_thread(api, args.base, "/api/home", None, 240.0)
                r["returned_home"] = bool(home.get("success"))
                if not r["returned_home"]:
                    print("      return home failed:", home.get("error"), flush=True)
                await asyncio.sleep(1.0)
            if rec:
                rec.stop()
                r["video"], r["still"] = f"videos/{name}.mp4", f"images/{name}.jpg"

    OUT_DIR = out / "metrics"
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if args.only and args.merge and (OUT_DIR / "benchmark.json").is_file():
        # Re-running some cases: keep the others from the previous run.
        done = {r["id"] for r in results}
        old = json.loads((OUT_DIR / "benchmark.json").read_text(encoding="utf-8"))["results"]
        for r in results:
            r["rerun"] = True
        results = sorted([r for r in old if r["id"] not in done] + results, key=lambda r: r["id"])
    meta = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "model": cfg.get("model_name"), "airframe": cfg.get("robot_name"),
            "scene": cfg.get("scene"), "passed": sum(r["passed"] for r in results), "total": len(results)}
    (OUT_DIR / "benchmark.json").write_text(json.dumps({"meta": meta, "results": results}, indent=2), encoding="utf-8")
    cols = ["id", "prompt", "status", "passed", "target_error_cm", "standoff_error_cm", "heading_error_deg",
            "estimator_error_cm", "sim_time_s", "path_m", "plan_latency_s"]
    lines = [f"# Twin flight benchmark\n\n{meta['created']} · {meta['airframe']} · {meta['scene']} · model {meta['model']} · "
             f"**{meta['passed']}/{meta['total']} passed**\n",
             "Scored against simulator ground truth: true vehicle position while hovering versus true object position.\n",
             "| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for r in results:
        lines.append("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |")
    (OUT_DIR / "benchmark.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\n{meta['passed']}/{meta['total']} passed. Results in {OUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
