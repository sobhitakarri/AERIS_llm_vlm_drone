"""
Gate tests on the digital twin: "fly through the gate with the dog" and five
more, one scene each (matlab/twin/config/scene_gates_<n>.json, written by
make_gate_scenes.py).

Start the twin on the scene, then run this script (no backend needed):

    # MATLAB:  run_aeris_twin('litewing', 'scene_gates_1')
    py -3 backend/scripts/run_gate_tests.py --test 1 --record

The script flies one test on whatever scene the twin is running. With
--launch it starts MATLAB itself for each requested test and stops it after.
Scoring uses the plant's true position: a pass is a crossing of the correct
gate's plane inside its opening, and no crossing of another gate.
"""
import argparse
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from backend.drone.twin_interface import TwinInterface  # noqa: E402
from backend.models.gemini_provider import GeminiProvider  # noqa: E402
from backend.planning.gate_nav import GateSpec, GateVLM, gate_skills, locate_gate  # noqa: E402
from backend.scripts.twin_recorder import Recorder, TwinSource  # noqa: E402

TESTS = json.loads((REPO_ROOT / "matlab" / "twin" / "config" / "gate_tests.json").read_text(encoding="utf-8"))
MATLAB = r"C:\Program Files\MATLAB\R2026b\bin\matlab.exe"
CRUISE_Z = 0.5
OPENING_HALF_W = 0.35
OPENING_Z = (0.20, 0.85)


class OverlaySource(TwinSource):
    """Twin frames with the vision model's choice drawn on the onboard view."""

    def __init__(self, twin):
        super().__init__(twin)
        self.choice = None

    def frame(self, cam):
        f = super().frame(cam)
        if f is None or cam != "fpv" or self.choice is None:
            return f
        for i, g in enumerate(self.choice.gates):
            if g.get("box"):
                u0, v0, u1, v1 = g["box"]
                chosen = i == self.choice.index
                cv2.rectangle(f, (u0, v0), (u1, v1), (60, 220, 60) if chosen else (160, 160, 160), 2 if chosen else 1)
                if chosen:
                    cv2.putText(f, "target", (u0, max(12, v0 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 220, 60), 1, cv2.LINE_AA)
        return f


class Crossings:
    """Samples the plant's true position and logs each crossing of a gate plane."""

    def __init__(self, twin, gates):
        self.twin, self.gates = twin, gates
        self.events, self.path = [], []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start()
        return self

    def stop(self):
        self._stop.set()
        self._t.join(timeout=3)

    def _run(self):
        prev = None
        while not self._stop.is_set():
            tr = self.twin.truth
            if tr and "x" in tr:
                cur = (tr["x"], tr["y"], tr["z"])
                self.path.append(cur)
                if prev is not None:
                    for g in self.gates:
                        if (prev[0] - g["x"]) * (cur[0] - g["x"]) < 0:
                            f = (g["x"] - prev[0]) / (cur[0] - prev[0])
                            y = prev[1] + f * (cur[1] - prev[1])
                            z = prev[2] + f * (cur[2] - prev[2])
                            inside = abs(y - g["y"]) < OPENING_HALF_W and OPENING_Z[0] < z < OPENING_Z[1]
                            self.events.append({"gate": g["name"], "y_off": round(y - g["y"], 3), "z": round(z, 3), "through_opening": inside})
                prev = cur
            time.sleep(0.05)


def run_test(t: dict, args, provider) -> dict:
    out = Path(args.out)
    tag = f"gate{t['id']}_{t['prompt'].lower().replace(' ', '_')[:40].strip('_')}"
    twin = TwinInterface()
    twin.connect()
    if not twin.wait_for_telemetry(args.wait):
        return {"id": t["id"], "error": "no telemetry from the twin"}
    scene = (twin.runtime.get("scene") or {}).get("name") if isinstance(twin.runtime.get("scene"), dict) else twin.runtime.get("scene")
    print(f"[{t['id']}] twin scene: {scene}; instruction: {t['prompt']}")
    hx, hy = twin.home_position()
    if math.dist((twin.state.x, twin.state.y), (hx, hy)) > 0.3 or twin.state.is_armed:
        twin.restart()                       # every test starts on the home pad
        t0 = time.time()
        while time.time() - t0 < 60 and (math.dist((twin.truth.get("x", 99), twin.truth.get("y", 99)), (hx, hy)) > 0.3 or twin.state.is_armed):
            time.sleep(0.5)
        time.sleep(4.0)                      # estimator boot window
    gt = twin.ground_truth()
    gates = [dict(g, **{"x": gt[g["name"]]["pos"][0], "y": gt[g["name"]]["pos"][1]}) for g in t["gates"] if g["name"] in gt]
    if len(gates) != len(t["gates"]):
        return {"id": t["id"], "error": f"twin is not running {t['scene']} (gates missing from ground truth)"}

    src = OverlaySource(twin)
    rec = Recorder(out / "videos" / f"{tag}.mp4", f"Gate test {t['id']}", src, caption=f"\"{t['prompt']}\"",
                   main_cam="fpv").start() if args.record else None
    res = {"id": t["id"], "prompt": t["prompt"], "scene": t["scene"], "answer": t["answer"]}
    cross = Crossings(twin, gates).start()
    try:
        ok = twin.takeoff(CRUISE_Z)
        if rec:
            rec.note = "taking off"
        twin.hover(1.5)
        frame = twin.get_camera_frame("fpv")
        cv2.imwrite(str(out / "images" / f"{tag}_view.jpg"), frame)
        if rec:
            rec.note = "asking the vision-language model"
        t0 = time.time()
        choice = GateVLM(provider).choose(frame, t["prompt"])
        res["vlm_s"] = round(time.time() - t0, 1)
        if choice is None or choice.index < 0:
            res.update(status="refused", reason=(choice.reason if choice else "no model answered"))
            if rec:
                rec.note = "no gate chosen"
        else:
            src.choice = choice
            pose = {"x": twin.state.x, "y": twin.state.y, "z": twin.state.z, "yaw": twin.state.yaw}
            gate = locate_gate(choice.box, twin.camera_info("fpv"), pose, GateSpec())
            res.update(model=choice.model, chosen_picture=choice.picture, reason=choice.reason,
                       gates_seen=[(g["picture"], g["shape"]) for g in choice.gates], located=gate)
            ws = twin.workspace()
            skills = gate_skills(gate, pose, GateSpec(), ws)
            res["plan"] = [s.model_dump() if hasattr(s, "model_dump") else s.dict() for s in skills]
            if rec:
                rec.note = f"{choice.picture[:28]} -> gate at ({gate['x']:+.2f}, {gate['y']:+.2f})"
            for s in skills:
                p = s.params
                if s.skill == "MOVE_TO":
                    src.choice = None            # the boxes belong to the view from the home pad
                    ok = twin.move_to(p["x"], p["y"], p["z"], velocity=p.get("speed", 0.4)) and ok
                elif s.skill == "HOVER":
                    twin.hover(p["t"])
        time.sleep(0.3)
        cross.stop()
        res["crossings"] = cross.events
        right = [e for e in cross.events if e["gate"] == t["answer"] and e["through_opening"]]
        wrong = [e for e in cross.events if e["gate"] != t["answer"] and abs(e["y_off"]) < 0.45]   # through another gate's frame
        res["passed"] = bool(right) and not wrong
        res.setdefault("status", "flew")
        if rec:
            rec.note = ("PASS: correct gate" if res["passed"] else "FAIL") + f"  (true crossings: {len(cross.events)})"
            time.sleep(1.0)
            rec.save_still(out / "images" / f"{tag}.jpg")
        twin.land()
    finally:
        cross.stop()
        if rec:
            rec.stop()
    twin.disconnect()
    print(json.dumps({k: v for k, v in res.items() if k != "plan"}, indent=1))
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", type=int, nargs="*", help="test ids (default: the one the running scene belongs to)")
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--launch", action="store_true", help="start and stop MATLAB for each test")
    ap.add_argument("--wait", type=float, default=300.0)
    ap.add_argument("--out", default=str(REPO_ROOT / "reports" / "twin_results" / "gates"))
    args = ap.parse_args()
    out = Path(args.out)
    (out / "videos").mkdir(parents=True, exist_ok=True)
    (out / "images").mkdir(parents=True, exist_ok=True)
    provider = GeminiProvider()
    results_file = out / "gate_results.json"
    results = json.loads(results_file.read_text(encoding="utf-8")) if results_file.exists() else []
    for t in TESTS:
        if args.test and t["id"] not in args.test:
            continue
        proc = None
        if args.launch:
            proc = subprocess.Popen([MATLAB, "-r", f"cd('{REPO_ROOT / 'matlab' / 'twin'}'); run_aeris_twin('litewing','{t['scene']}')"],
                                    creationflags=0x00000008 | 0x00000200)   # DETACHED_PROCESS | NEW_PROCESS_GROUP
        try:
            r = run_test(t, args, provider)
            results = [x for x in results if x.get("id") != r["id"]] + [r]
        finally:
            if proc is not None:
                try:
                    tw = TwinInterface(); tw.connect(); tw._send("STOP"); time.sleep(2); tw.disconnect()
                except Exception:
                    pass
                time.sleep(5)
                proc.terminate()
        (out / "gate_results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    ok = sum(1 for r in results if r.get("passed"))
    print(f"\n{ok}/{len(results)} passed")
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
