"""
Closed-loop replanning test on the digital twin.

Starts a hover-above mission, moves the target while the vehicle is on
station, and checks that the system notices, replans and ends up over the
object's new true position.

    py -3 backend/scripts/twin_replan_test.py
    py -3 backend/scripts/twin_replan_test.py --target red_bottle --dx 0.25 --dy 0.0
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


def api(base, path, body=None, post=False):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method="POST" if (body is not None or post) else "GET",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=240) as r:
        return json.loads(r.read())


async def main() -> int:
    import websockets
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8000")
    ap.add_argument("--target", default="red_bottle")
    ap.add_argument("--dx", type=float, default=-0.20)
    ap.add_argument("--dy", type=float, default=0.30)
    ap.add_argument("--moves", type=int, default=1, help="number of successive nudges")
    ap.add_argument("--out", default=str(REPO_ROOT / "reports" / "twin_results"), help="results folder")
    ap.add_argument("--record", action="store_true", help="record a video and a still of the test")
    args = ap.parse_args()

    from backend.drone.twin_interface import load_runtime
    obj = ((load_runtime().get("ground_truth") or {}).get("objects") or {}).get(args.target)
    if not obj:
        print(f"'{args.target}' is not in the scene.")
        return 1
    x0, y0, z0 = obj["pos"]
    label = args.target.replace("_", " ")

    out, rec = Path(args.out), None
    prompt = f"Find the {label} and hover above it for 25 seconds"
    if args.record:
        from backend.scripts.twin_recorder import HttpSource, Recorder
        rec = Recorder(out / "videos" / "case09_replan_moved_target.mp4", "Case 9  replan", HttpSource(args.base),
                       caption=f'"{prompt}" - target moved mid-hover').start()
    reply = api(args.base, "/api/command", {"command": prompt})
    if not reply.get("success"):
        print("refused:", reply.get("error") or reply.get("errors"))
        if rec:
            rec.stop()
        return 1
    print("plan:", reply["reasoning"])

    moved, new_xy, replans, hover_truth = 0, (x0, y0), 0, None
    t_moved = None
    async with websockets.connect(args.base.replace("http", "ws") + "/ws/telemetry") as ws:
        hover_since = None
        deadline = time.time() + 420
        while time.time() < deadline:
            m = json.loads(await ws.recv())
            mon, twin = m.get("monitor") or {}, m.get("twin") or {}
            replans = m.get("replan_count", 0)
            sim_t = twin.get("sim_time") or 0.0
            if mon.get("current_skill") == "HOVER" and mon.get("status") == "running":
                hover_since = hover_since if hover_since is not None else sim_t
                if twin.get("truth"):
                    hover_truth = (twin["truth"]["x"], twin["truth"]["y"])
                # Nudge the target a few seconds into each hover.
                if moved < args.moves and sim_t - hover_since > 3.0 and (t_moved is None or sim_t - t_moved > 4.0):
                    moved += 1
                    new_xy = (x0 + moved * args.dx, y0 + moved * args.dy)
                    api(args.base, "/api/twin/object", {"name": args.target, "x": new_xy[0], "y": new_xy[1], "z": z0})
                    t_moved = sim_t
                    print(f"t={sim_t:.1f}s  moved {label} to ({new_xy[0]:.2f}, {new_xy[1]:.2f})")
                    if rec:
                        rec.note = f"target moved {math.hypot(args.dx, args.dy) * 100:.0f} cm at t={sim_t:.0f}s"
            else:
                hover_since = None
            if mon.get("status") in ("done", "error", "aborted") and moved:
                print("mission", mon.get("status"), "| error:", m.get("last_error"))
                break
    err = math.hypot(hover_truth[0] - new_xy[0], hover_truth[1] - new_xy[1]) if hover_truth else float("nan")
    print(f"replans: {replans}   final hover is {err * 100:.1f} cm from the object's new true position")
    if rec:
        rec.save_still(out / "images" / "case09_replan_moved_target.jpg")
    api(args.base, "/api/home", post=True)
    api(args.base, "/api/twin/object", {"name": args.target, "x": x0, "y": y0, "z": z0})
    ok = replans >= 1 and err < 0.15
    if rec:
        rec.stop()
    (out / "metrics").mkdir(parents=True, exist_ok=True)
    (out / "metrics" / "replan.json").write_text(json.dumps({
        "prompt": prompt, "target": args.target, "moved_cm": round(100 * moved * math.hypot(args.dx, args.dy), 1),
        "moves": moved, "replans": replans, "final_error_cm": round(err * 100, 1), "passed": bool(ok),
        "video": "videos/case09_replan_moved_target.mp4", "still": "images/case09_replan_moved_target.jpg",
    }, indent=2), encoding="utf-8")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
