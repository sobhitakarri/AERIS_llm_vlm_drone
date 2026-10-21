"""
Flight benchmark against the digital twin.

Sends natural-language prompts to a running backend (`python -m backend.main
--drone twin`) and scores each mission against the twin's ground truth:
where the vehicle truly was while it hovered, versus where the object the
prompt refers to truly is. The ground truth is used here and nowhere else.

    py -3 backend/scripts/run_twin_benchmark.py
    py -3 backend/scripts/run_twin_benchmark.py --only 1 3 --seeds 3
    py -3 backend/scripts/run_twin_benchmark.py --record

Evaluation matrix: run modes x scenarios x disturbances x seeds.

    py -3 backend/scripts/run_twin_benchmark.py --modes full open_loop scripted \
        --disturbances none move_target delay_250 --seeds 10 --randomize --paraphrase

A seed fixes the object layout, the prompt wording and the disturbance, and
is the same for every mode, so modes are compared on identical trials. The
plan cache stays on; each row's cache_hit says whether it answered.

Baseline inputs come from the scene file's nominal layout, as a prior map
written before the flight would: `scripted` flies to the nominal positions
and `no_vlm` gives the nominal map to the LLM. With --randomize the real
layout differs from that map, which is what those baselines are tested on.

Results: <out>/metrics/trials.csv (one row per trial) and trials.jsonl (the
same rows with each mission's full trace). A plain run (mode full, no
disturbance, no randomisation) also writes benchmark.json and benchmark.md.
With --record, one video and one still per trial in <out>/videos and <out>/images.
"""
import argparse
import asyncio
import csv
import json
import math
import random
import re
import sys
import time
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
DEFAULT_OUT = REPO_ROOT / "reports" / "twin_results"
SCENE_DIR = REPO_ROOT / "matlab" / "twin" / "config"

# `expect` names the scene object the prompt refers to, for scoring only.
# kind: above = hover over it, view = stand off and face it, shape = no target.
# `obj` marks scenarios whose wording may be varied with --paraphrase;
# `fixed_layout` ones depend on where objects stand relative to each other.
# `avoid` names a look-alike: hovering at it instead is a wrong-object result.
# `scenes` limits a scenario to the scenes where its answer holds (default: all).
SCENARIOS = [
    {"id": 1, "prompt": "Find the cardboard box and hover above it", "expect": "cardboard_box", "kind": "above", "obj": "cardboard box"},
    {"id": 2, "prompt": "Find the red bottle and hover above it for 4 seconds", "expect": "red_bottle", "kind": "above", "obj": "red bottle", "avoid": "green_bottle"},
    {"id": 3, "prompt": "Inspect the blue cube from 0.5 m away", "expect": "blue_cube", "kind": "view", "standoff": 0.5, "obj": "blue cube"},
    {"id": 4, "prompt": "Find the steel drum and hover above its center", "expect": "steel_drum", "kind": "above", "obj": "steel drum"},
    {"id": 5, "prompt": "Go to the yellow crate", "expect": "yellow_crate", "kind": "above", "obj": "yellow crate"},
    {"id": 6, "prompt": "Find the traffic cone and hover facing it", "expect": "traffic_cone", "kind": "view", "standoff": 0.5, "obj": "traffic cone"},
    {"id": 7, "prompt": "Take off to 0.8 m and fly in a square", "expect": None, "kind": "shape", "shape": "square", "alt": 0.8},
    {"id": 8, "prompt": "Find the purple elephant and hover above it", "expect": None, "kind": "absent", "obj": "purple elephant"},
    # Harder wording. Case 9 is the moved-target replan test (twin_replan_test.py).
    {"id": 10, "prompt": "Find cardboard box and hover it", "expect": "cardboard_box", "kind": "above"},
    {"id": 11, "prompt": "Find the shiny red bottle next to the blue cube and hover facing it", "expect": "red_bottle", "kind": "view", "standoff": 0.5, "avoid": "green_bottle"},
    {"id": 12, "prompt": "Find the bottle that is left of the blue cube and inspect it from 0.4 m away", "expect": "red_bottle", "kind": "view", "standoff": 0.4, "fixed_layout": True, "scenes": ["scene_lab"]},
    # Seen from home (facing +x) the bottle is nearer than the cube, so the bottle branch applies.
    {"id": 13, "prompt": "Find the red bottle if it is in front of the cube, otherwise look at the cube", "expect": "red_bottle", "kind": "near", "avoid": "blue_cube", "fixed_layout": True},
    # Bottle, drum and extinguisher are all cylinders: any of them is a correct reading.
    {"id": 14, "prompt": "Look at the cylindrical object and hover above its center", "expect": ["steel_drum", "red_bottle", "fire_extinguisher", "green_bottle"], "kind": "above"},
    {"id": 15, "prompt": "Fly 0.5m and make a square in air", "expect": None, "kind": "shape", "shape": "square", "alt": 0.5},
    # The wall runs along y from -2.35 to -1.15; only the -1.15 end is inside the geofence.
    {"id": 16, "prompt": "Hover at the end of the wall", "expect": None, "kind": "point", "point": [-0.45, -1.15], "tol": 0.30},
    {"id": 17, "prompt": "Fly in a star pattern", "expect": None, "kind": "shape", "shape": "star"},
    # Wrong-object grounding (scene_distractor): a look-alike is present and is a real object,
    # so the depth check cannot reject it. Only choosing the right one passes.
    {"id": 20, "prompt": "Find the red bottle and hover above it", "expect": "red_bottle", "kind": "above", "avoid": "green_bottle", "scenes": ["scene_distractor"]},
    {"id": 21, "prompt": "Find the green bottle and hover above it", "expect": "green_bottle", "kind": "above", "avoid": "red_bottle", "scenes": ["scene_distractor"]},
    # The green bottle is 0.44 m from the laptop, the red one 0.81 m: true from any viewpoint.
    {"id": 22, "prompt": "Find the bottle next to the laptop and hover above it", "expect": "green_bottle", "kind": "above", "avoid": "red_bottle", "fixed_layout": True, "scenes": ["scene_distractor"]},
    {"id": 23, "prompt": "Find the red box and hover above it", "expect": "red_box", "kind": "above", "avoid": "red_bottle", "scenes": ["scene_distractor"]},
]

PHRASINGS = {
    "above": [
        "Find the {obj} and hover above it",
        "Hover over the {obj}",
        "Locate the {obj} and hover on top of it",
        "Fly to the {obj} and hold position above it",
    ],
    "absent": [
        "Find the {obj} and hover above it",
        "Hover over the {obj}",
        "Locate the {obj} and hover on top of it",
    ],
    "view": [
        "Inspect the {obj} from {d} m away",
        "Look at the {obj} from {d} m away",
        "Go to the {obj} and face it from {d} m away",
    ],
}

MODES = ["full", "scripted", "open_loop", "no_vlm", "no_depth_check", "no_replan", "safety_shadow"]
DISTURBANCES = ["none", "move_target", "remove_target", "delay_100", "delay_250", "delay_500"]

# Where movable objects may stand in a randomised layout (world frame, m),
# and the objects each must keep clear of.
LAYOUT_REGIONS = {
    "red_bottle": {"x": (0.98, 2.12), "y": (0.98, 1.52), "z": 0.75, "clear_of": ["blue_cube", "laptop", "green_bottle"], "gap": 0.25},
    "traffic_cone": {"x": (-0.10, 1.00), "y": (-1.50, -0.35), "z": 0.0, "clear_of": ["steel_drum", "yellow_crate"], "gap": 0.45},
}
HIDE_Z = -1.0                 # remove_target: below the floor, out of every camera
MOVE_RANGE_M = (0.25, 0.40)   # move_target: how far the target is pushed mid-hover
DISTURBED_HOVER_S = 10        # hover long enough for a moved target to be noticed
COMMAND_TIMEOUT_S = 1200      # longest a plan may take, through every model fallback

CSV_COLUMNS = [
    "run_id", "trial", "seed", "scene", "mode", "scenario", "kind", "disturbance", "prompt", "paraphrased", "randomized",
    "accepted", "status", "passed", "refused_reason",
    "object_matched", "target_error_cm", "standoff_error_cm", "heading_error_deg", "altitude_error_cm",
    "estimator_error_cm", "grounding_error_cm", "wrong_object", "stale_hover", "clearance_m",
    "sim_time_s", "path_m", "plan_latency_s",
    "disturbance_sim_t", "first_replan_sim_t", "replan_latency_s", "replans", "replans_suppressed", "targets_lost",
    "planner_source", "intent", "cache_hit", "grounding", "llm_model", "vlm_model",
    "n_llm_calls", "n_vlm_calls", "n_model_attempts", "n_malformed", "model_unavailable",
    "t_intent_s", "t_goal_s", "t_frame_s", "t_object_search_s", "t_actions_s", "t_llm_plan_s", "t_validate_s", "t_plan_total_s",
    "validator_valid", "validator_errors", "validator_overridden", "n_clipped",
    "telemetry_mode", "mission_id", "returned_home", "twin_restarted",
]


def api(base: str, path: str, body: dict = None, timeout: float = 180.0) -> dict:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method="POST" if body is not None or path in POSTS else "GET",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


POSTS = {"/api/home", "/api/land", "/api/abort", "/api/twin/restart"}


def slug(scenario: dict) -> str:
    words = "".join(c if c.isalnum() else " " for c in scenario["prompt"].lower()).split()
    return f"case{scenario['id']:02d}_" + "_".join(words[:6])


def primary_target(scenario: dict):
    expect = scenario.get("expect")
    return expect if isinstance(expect, str) else None


# ------------------------------------------------------------- trial design
def nominal_scene(scene: str = "scene_lab") -> dict:
    """The scene file's layout: the prior map the baselines are given."""
    scene = json.loads((SCENE_DIR / f"{scene}.json").read_text(encoding="utf-8"))
    return {o["name"]: o for o in scene["objects"]}


def random_layout(rng: random.Random, nominal: dict) -> dict:
    """New positions for the movable objects, clear of their neighbours."""
    out = {}
    for name, reg in LAYOUT_REGIONS.items():
        if name not in nominal:
            continue
        for _ in range(200):
            x, y = rng.uniform(*reg["x"]), rng.uniform(*reg["y"])
            if all(math.hypot(x - nominal[o]["pos"][0], y - nominal[o]["pos"][1]) >= reg["gap"]
                   for o in reg["clear_of"] if o in nominal):
                out[name] = [round(x, 3), round(y, 3), reg["z"]]
                break
    return out


def moved_position(rng: random.Random, name: str, start: list) -> list:
    """Where move_target pushes the target: a short hop that stays in its region."""
    reg = LAYOUT_REGIONS.get(name)
    for _ in range(200):
        d, a = rng.uniform(*MOVE_RANGE_M), rng.uniform(0, 2 * math.pi)
        x, y = start[0] + d * math.cos(a), start[1] + d * math.sin(a)
        if reg is None or (reg["x"][0] <= x <= reg["x"][1] and reg["y"][0] <= y <= reg["y"][1]):
            return [round(x, 3), round(y, 3), start[2]]
    return [round(start[0] + MOVE_RANGE_M[0], 3), start[1], start[2]]


def phrase(rng: random.Random, scenario: dict, paraphrase: bool) -> tuple:
    templates = PHRASINGS.get(scenario["kind"])
    if not paraphrase or not scenario.get("obj") or not templates:
        return scenario["prompt"], False
    text = rng.choice(templates).format(obj=scenario["obj"], d=scenario.get("standoff", 0.5))
    return text, text != scenario["prompt"]


def with_long_hover(prompt: str) -> str:
    if re.search(r"\d+(?:\.\d+)?\s*(?:s|sec|secs|seconds?)\b", prompt):
        return re.sub(r"\d+(?:\.\d+)?\s*(?:s|sec|secs|seconds?)\b", f"{DISTURBED_HOVER_S} seconds", prompt, count=1)
    return f"{prompt}, for {DISTURBED_HOVER_S} seconds"


def applicable(mode: str, scenario: dict, disturbance: str, movable: set) -> str:
    """Empty when the trial can run; otherwise why it is skipped."""
    if mode == "scripted" and scenario["kind"] == "absent":
        return "a script has no waypoint for an object that does not exist"
    if mode == "scripted" and scenario["kind"] == "above" and not primary_target(scenario):
        return "ambiguous target: no single scripted waypoint"
    if disturbance in ("move_target", "remove_target"):
        tgt = primary_target(scenario)
        if not tgt or tgt not in movable or scenario["kind"] not in ("above", "view", "near"):
            return "target is not a movable object"
    return ""


def scripted_skills(scenario: dict, nominal: dict, home: list) -> list:
    """B0: the waypoints an engineer would write for the nominal layout."""
    from backend.planning.shape_paths import closed_move_skills, shape_corners
    kind = scenario["kind"]
    if kind == "shape":
        z = scenario.get("alt", 0.8)
        moves = [s.model_dump() for s in closed_move_skills(z, shape_corners(scenario.get("shape", "square")))]
        return [{"skill": "TAKEOFF", "params": {"z": z}}] + moves + [{"skill": "HOVER", "params": {"t": 2.0}}]
    if kind == "point":
        x, y = scenario["point"]
        return [{"skill": "TAKEOFF", "params": {"z": 0.6}},
                {"skill": "MOVE_TO", "params": {"x": x, "y": y, "z": 0.6}},
                {"skill": "HOVER", "params": {"t": 3.0}}]
    obj = nominal[primary_target(scenario)]
    ox, oy = obj["pos"][0], obj["pos"][1]
    top = obj["pos"][2] + obj["size_m"][2]
    hover_t = 3.0
    m = re.search(r"(\d+(?:\.\d+)?)\s*(?:s|sec|secs|seconds?)\b", scenario.get("_prompt", scenario["prompt"]))
    if m:
        hover_t = float(m.group(1))
    if kind == "view":
        d = scenario.get("standoff", 0.5)
        ax, ay = home[0] - ox, home[1] - oy
        n = math.hypot(ax, ay) or 1.0
        vx, vy = ox + d * ax / n, oy + d * ay / n
        z = round(max(0.6, top + 0.10), 2)
        yaw = math.degrees(math.atan2(oy - vy, ox - vx))
        return [{"skill": "TAKEOFF", "params": {"z": z}},
                {"skill": "MOVE_TO", "params": {"x": round(vx, 3), "y": round(vy, 3), "z": z}},
                {"skill": "ROTATE", "params": {"yaw": round(yaw, 1)}},
                {"skill": "HOVER", "params": {"t": hover_t}}]
    z = round(max(0.6, top + 0.35), 2)
    return [{"skill": "TAKEOFF", "params": {"z": 0.6}},
            {"skill": "MOVE_TO", "params": {"x": ox, "y": oy, "z": z}},
            {"skill": "HOVER", "params": {"t": hover_t}}]


def prior_map(nominal: dict) -> list:
    return [{"label": n, "x": o["pos"][0], "y": o["pos"][1], "z": round(o["pos"][2] + o["size_m"][2], 3)}
            for n, o in nominal.items()]


# ------------------------------------------------------------------- flight
class Disturber:
    """Applies one disturbance at its moment in the mission."""

    def __init__(self, base: str, kind: str, target: str, start_pos: list, rng: random.Random):
        self.base, self.kind, self.target, self.start = base, kind, target, start_pos
        self.new_pos = moved_position(rng, target, start_pos) if kind == "move_target" and target else None
        self.sim_t = None
        self._hover_t0 = None

    def tick(self, skill: str, status: str, sim_t: float) -> None:
        if self.sim_t is not None or status != "running" or self.kind not in ("move_target", "remove_target"):
            return
        if self.kind == "move_target":
            # A second into the first hover over the target.
            if skill == "HOVER":
                self._hover_t0 = sim_t if self._hover_t0 is None else self._hover_t0
                if sim_t - self._hover_t0 >= 1.0:
                    self._fire(self.new_pos)
            else:
                self._hover_t0 = None
        elif skill in ("MOVE_TO", "FIND", "INSPECT"):
            # On the way to the target, after the plan was made from it.
            self._fire([self.start[0], self.start[1], HIDE_Z])

    def _fire(self, pos: list) -> None:
        api(self.base, "/api/twin/object", {"name": self.target, "x": pos[0], "y": pos[1], "z": pos[2]})
        self.sim_t = self._now_sim_t
        print(f"      disturbance {self.kind}: {self.target} -> ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})", flush=True)

    _now_sim_t = None


async def fly(base: str, ws_url: str, scenario: dict, request: dict, disturber=None, rec=None, still=None) -> dict:
    import websockets

    result = {"id": scenario["id"], "prompt": request["command"], "kind": scenario["kind"]}
    t_wall = time.time()
    # Planning can sit through model fallbacks (30 s deadline each, retried on
    # 503/504); giving up sooner would leave the backend to fly unobserved.
    reply = await asyncio.to_thread(api, base, "/api/command", request, COMMAND_TIMEOUT_S)
    result["plan_latency_s"] = round(time.time() - t_wall, 2)
    result["accepted"] = bool(reply.get("success"))
    result["mission_id"] = reply.get("mission_id")
    result["reasoning"] = reply.get("reasoning") or reply.get("error") or "; ".join(reply.get("errors", []))
    result["skills"] = [s["skill"] for s in reply.get("skills", [])]
    if not result["accepted"]:
        result["status"] = "refused"
        result["refused_reason"] = result["reasoning"]
        # Refusing an absent target is the right answer. (remove_target hides
        # the object only after the plan exists, so a refusal there is a miss.)
        result["passed"] = scenario["kind"] == "absent"
        if rec:
            rec.note = "REFUSED: " + str(result["reasoning"])[:34]
            await asyncio.sleep(4.0)
            rec.save_still(still)
        return result

    samples, hover, path_len, last = [], [], 0.0, None
    sim_t0, sim_t, hover_t0, still_done = None, None, None, False
    async with websockets.connect(ws_url) as ws:
        deadline = time.time() + 300
        seen_running = False
        while time.time() < deadline:
            m = json.loads(await ws.recv())
            mon, twin = m.get("monitor") or {}, m.get("twin") or {}
            result.setdefault("telemetry_mode", m.get("run_mode"))
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
            if disturber is not None and sim_t is not None:
                disturber._now_sim_t = sim_t
                await asyncio.to_thread(disturber.tick, mon.get("current_skill"), mon.get("status"), sim_t)
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
    result["_hover"], result["_samples"] = hover, samples
    if rec and not still_done:
        rec.save_still(still)
    return result


def score(scenario: dict, result: dict, truth_objects: dict, disturbance: str, start_truth: dict) -> None:
    """Pass/fail and errors against ground truth taken after the flight."""
    hover, samples = result.pop("_hover", []), result.pop("_samples", [])
    if result["status"] == "refused":
        return
    ok = result["status"] == "done"
    if hover:
        (hx, hy, hz), hyaw, ex, ey, ez = hover[-1]
        result["estimator_error_cm"] = round(100 * math.dist((hx, hy, hz), (ex, ey, ez)), 1)
        result["hover_true"] = [round(hx, 3), round(hy, 3), round(hz, 3)]
        expect = scenario.get("expect") or []
        names = [n for n in ([expect] if isinstance(expect, str) else expect) if n in truth_objects]
        # With several acceptable objects, score against the one the vehicle went to.
        name = min(names, key=lambda n: math.hypot(hx - truth_objects[n]["pos"][0], hy - truth_objects[n]["pos"][1]), default=None)
        obj = truth_objects.get(name or "")
        if disturbance == "remove_target":
            obj = None
            old = start_truth.get(primary_target(scenario) or "")
            if old:
                result["stale_hover"] = math.hypot(hx - old["pos"][0], hy - old["pos"][1]) < 0.15
        if obj:
            result["object_matched"] = name
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
            elif scenario["kind"] == "near":
                # Went to the intended object, not the alternative named in the prompt.
                alt = truth_objects.get(scenario.get("avoid") or "")
                alt_dist = math.hypot(hx - alt["pos"][0], hy - alt["pos"][1]) if alt else math.inf
                result["target_error_cm"] = round(100 * dist, 1)
                ok = ok and dist < 0.6 and dist < alt_dist
        look_alike = truth_objects.get(scenario.get("avoid") or "")
        if obj and look_alike and scenario["kind"] in ("above", "view"):
            # Went to the look-alike rather than the object the prompt meant.
            d_alt = math.hypot(hx - look_alike["pos"][0], hy - look_alike["pos"][1])
            result["wrong_object"] = d_alt < math.hypot(hx - obj["pos"][0], hy - obj["pos"][1]) and d_alt < 0.6
        if scenario["kind"] == "point":
            px, py = scenario["point"]
            d = math.hypot(hx - px, hy - py)
            result["target_error_cm"] = round(100 * d, 1)
            ok = ok and d < scenario.get("tol", 0.3)
    elif scenario["kind"] in ("near", "point"):
        ok = False                                           # never held station anywhere
    if scenario["kind"] == "shape" and scenario.get("alt") and samples:
        top = max(p[2] for p in samples)
        result["altitude_error_cm"] = round(100 * abs(top - scenario["alt"]), 1)
        ok = ok and abs(top - scenario["alt"]) < 0.10
    if scenario["kind"] == "absent":
        ok = False                                           # a mission was flown to something that is not there
    if disturbance == "remove_target":
        # The target vanished after planning: completing the mission as if it
        # were still there is the failure; stopping or refusing is correct.
        ok = result["status"] != "done"
    result["passed"] = bool(ok)


def trace_columns(tr: dict, truth_at_plan: dict, expect) -> dict:
    """Flatten a mission trace into CSV columns."""
    if not tr:
        return {}
    calls = tr.get("model_calls") or []
    attempts = [a for c in calls for a in c.get("attempts") or []]
    events = tr.get("events") or []
    stages = tr.get("stages") or {}
    planner = tr.get("planner") or {}
    validation = tr.get("validation") or {}
    first_replan = next((e for e in events if e["kind"] == "replan"), None)
    out = {
        "planner_source": planner.get("source"), "intent": planner.get("intent"),
        "cache_hit": planner.get("cache_hit"), "grounding": planner.get("grounding"),
        "llm_model": ";".join(sorted({c.get("model") or "" for c in calls if c["kind"] == "llm"})) or None,
        "vlm_model": ";".join(sorted({c.get("model") or "" for c in calls if c["kind"] == "vlm"})) or None,
        "n_llm_calls": sum(c["kind"] == "llm" for c in calls),
        "n_vlm_calls": sum(c["kind"] == "vlm" for c in calls),
        "n_model_attempts": len(attempts),
        "n_malformed": sum(1 for a in attempts if a.get("error")),
        # Every request failed (quota, overload): an outage, not the system's answer.
        "model_unavailable": bool(attempts) and all(a.get("error") and not a.get("raw") for a in attempts),
        "validator_valid": validation.get("valid"),
        "validator_errors": "; ".join(validation.get("errors") or []) or None,
        "validator_overridden": any(e["kind"] == "validator_overridden" for e in events),
        # Requests outside the envelope that were clamped before the validator saw them.
        "n_clipped": sum(e["kind"] == "workspace_clip" for e in events),
        "replans": sum(e["kind"] == "replan" for e in events),
        "replans_suppressed": sum(e["kind"] == "replan_suppressed" for e in events),
        "targets_lost": sum(e["kind"] == "target_lost" for e in events),
        "first_replan_sim_t": first_replan.get("sim_t") if first_replan else None,
    }
    for name in ("intent", "goal", "frame", "object_search", "actions", "llm_plan", "validate", "plan_total"):
        out[f"t_{name}_s"] = stages.get(name)
    gxy = planner.get("grounded_xy")
    obj = truth_at_plan.get(expect) if isinstance(expect, str) else None
    if gxy and gxy[0] is not None and obj:
        out["grounding_error_cm"] = round(100 * math.hypot(gxy[0] - obj["pos"][0], gxy[1] - obj["pos"][1]), 1)
    return out


# ---------------------------------------------------------------- utilities
def truth(base: str) -> dict:
    try:
        return (api(base, "/api/eval/truth", timeout=10) or {}).get("objects") or {}
    except Exception:
        return {}


def place(base: str, positions: dict) -> None:
    for name, pos in positions.items():
        api(base, "/api/twin/object", {"name": name, "x": pos[0], "y": pos[1], "z": pos[2]})


async def go_home(base: str, home: list) -> tuple:
    """Back on the pad. Restarts the twin when the vehicle cannot get there
    (a failed mission lands where it is), so every trial starts at home."""
    ok = False
    try:
        ok = bool((await asyncio.to_thread(api, base, "/api/home", None, 240.0)).get("success"))
    except Exception as exc:
        print("      return home failed:", exc, flush=True)
    st = (await asyncio.to_thread(api, base, "/api/telemetry", None, 10.0)).get("state") or {}
    if ok and math.hypot(st.get("x", 0) - home[0], st.get("y", 0) - home[1]) < 0.3 and st.get("z", 1) < 0.1:
        return True, False
    print("      vehicle not on the home pad: restarting the twin", flush=True)
    await restart_twin(base, home)
    return ok, True


async def restart_twin(base: str, home: list) -> None:
    """Restart the simulation (vehicle on the pad, estimator and objects reset)
    and wait until it is back and the scene camera has calibrated again."""
    before = ((await asyncio.to_thread(api, base, "/api/telemetry", None, 10.0)).get("twin") or {}).get("sim_time") or 0.0
    await asyncio.to_thread(api, base, "/api/twin/restart")
    await asyncio.sleep(5.0)
    t_end = time.time() + 240
    while time.time() < t_end:
        await asyncio.sleep(2.0)
        try:
            t = await asyncio.to_thread(api, base, "/api/telemetry", None, 10.0)
            s = t.get("state") or {}
            sim_t = (t.get("twin") or {}).get("sim_time")
            cal = (await asyncio.to_thread(api, base, "/api/scene", None, 20.0)).get("calibration") or {}
            # A new run: the clock started again, so calibration below is the new run's.
            restarted = sim_t is not None and (sim_t < before or before < 1.0)
            if (restarted and t.get("connected") and math.hypot(s.get("x", 9) - home[0], s.get("y", 9) - home[1]) < 0.3
                    and cal.get("calibrated") and cal.get("depth")):
                await asyncio.sleep(2.0)
                return
        except Exception:
            pass
    raise RuntimeError("The twin did not come back after a restart.")


def write_rows(csv_path: Path, rows: list, append: bool) -> None:
    new = not (append and csv_path.is_file())
    with open(csv_path, "w" if new else "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        if new:
            w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in CSV_COLUMNS})


# --------------------------------------------------------------------- main
async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8000")
    ap.add_argument("--only", type=int, nargs="*", help="scenario ids to run")
    ap.add_argument("--modes", nargs="*", default=["full"], choices=MODES)
    ap.add_argument("--disturbances", nargs="*", default=["none"], choices=DISTURBANCES)
    ap.add_argument("--seeds", "--repeats", type=int, default=1, dest="seeds", help="trials per cell")
    ap.add_argument("--seed-offset", type=int, default=0, help="first seed, to extend an earlier run")
    ap.add_argument("--randomize", action="store_true", help="random positions for the movable objects")
    ap.add_argument("--paraphrase", action="store_true", help="vary the wording of the prompt")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="results folder")
    ap.add_argument("--csv", help="trial CSV (default <out>/metrics/trials.csv)")
    ap.add_argument("--append", action="store_true", help="append to the CSV instead of replacing it")
    ap.add_argument("--record", action="store_true", help="record a video and a still per trial")
    ap.add_argument("--reset-each-trial", action="store_true",
                    help="restart the twin before every trial, so no trial inherits estimator drift")
    ap.add_argument("--merge", action="store_true", help="plain run with --only: replace those cases in benchmark.json, keep the rest")
    args = ap.parse_args()
    out = Path(args.out)
    ws_url = args.base.replace("http", "ws") + "/ws/telemetry"
    out_dir = out / "metrics"
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = Path(args.csv) if args.csv else out_dir / "trials.csv"

    # The link drops for a few seconds whenever the twin starts a new run.
    cfg, t_end = api(args.base, "/api/config"), time.time() + 180
    while cfg.get("drone_backend") == "twin" and not cfg.get("drone_connected") and time.time() < t_end:
        time.sleep(3.0)
        cfg = api(args.base, "/api/config")
    if cfg.get("drone_backend") != "twin" or not cfg.get("drone_connected"):
        print("The backend is not connected to the digital twin.")
        return 1
    home = cfg.get("home") or [0.0, 0.0]
    movable = set(cfg.get("movable_objects") or [])
    scene = cfg.get("scene") or "scene_lab"
    if not (SCENE_DIR / f"{scene}.json").is_file():
        print(f"Scene '{scene}' has no file in {SCENE_DIR}.")
        return 1
    nominal = nominal_scene(scene)
    nominal_movable = {n: list(nominal[n]["pos"]) for n in movable if n in nominal}
    run_id = time.strftime("%Y%m%d_%H%M%S")
    plain = args.modes == ["full"] and args.disturbances == ["none"] and not args.randomize and not args.paraphrase

    chosen = [s for s in SCENARIOS if (not args.only or s["id"] in args.only) and scene in s.get("scenes", [scene])]
    if not chosen:
        print(f"No scenarios for scene '{scene}'" + (f" among {args.only}" if args.only else "") + ".")
        return 1
    results, trial = [], 0
    jsonl = open(out_dir / f"trials_{run_id}.jsonl", "w", encoding="utf-8")
    try:
        for seed in range(args.seed_offset, args.seed_offset + args.seeds):
            for sc in chosen:
                rng_layout = random.Random(f"layout-{seed}-{sc['id']}")
                layout = random_layout(rng_layout, nominal) if args.randomize and not sc.get("fixed_layout") else {}
                for dist in args.disturbances:
                    for mode in args.modes:
                        skip = applicable(mode, sc, dist, movable)
                        if skip:
                            print(f"[{sc['id']}] {mode}/{dist} seed {seed}: skipped ({skip})", flush=True)
                            continue
                        trial += 1
                        # Same seed, same trial: identical wording and disturbance in every mode.
                        rng = random.Random(f"trial-{seed}-{sc['id']}-{dist}")
                        prompt, paraphrased = phrase(rng, sc, args.paraphrase)
                        if dist == "move_target":
                            prompt = with_long_hover(prompt)
                        request = {"command": prompt, "mode": mode}
                        if mode == "scripted":
                            request["skills"] = scripted_skills({**sc, "_prompt": prompt}, nominal, home)
                        if mode == "no_vlm":
                            request["known_objects"] = prior_map(nominal)

                        if args.reset_each_trial:
                            # Every trial from a fresh estimator: otherwise drift
                            # built up in earlier flights carries into later ones.
                            await restart_twin(args.base, home)
                        await asyncio.to_thread(place, args.base, {**nominal_movable, **layout})
                        await asyncio.sleep(1.5)                 # depth map and tracker see the new layout
                        start_truth = await asyncio.to_thread(truth, args.base)
                        tgt = primary_target(sc)
                        disturber = None
                        if dist in ("move_target", "remove_target") and tgt in start_truth:
                            disturber = Disturber(args.base, dist, tgt, list(start_truth[tgt]["pos"]), rng)
                        if dist.startswith("delay_"):
                            await asyncio.to_thread(api, args.base, "/api/twin/link_delay", {"ms": float(dist.split("_")[1])})

                        print(f"[{sc['id']}] {mode}/{dist} seed {seed}: {prompt}", flush=True)
                        rec, still, name = None, None, slug(sc) + ("" if plain and args.seeds == 1 else f"_{mode}_{dist}_s{seed}")
                        if args.record:
                            from backend.scripts.twin_recorder import HttpSource, Recorder
                            still = out / "images" / f"{name}.jpg"
                            rec = Recorder(out / "videos" / f"{name}.mp4", f"Case {sc['id']} [{mode}]", HttpSource(args.base),
                                           caption=f'"{prompt}"').start()
                        try:
                            r = await fly(args.base, ws_url, sc, request, disturber, rec, still)
                        finally:
                            if dist.startswith("delay_"):
                                await asyncio.to_thread(api, args.base, "/api/twin/link_delay", {"ms": 0.0})
                        end_truth = await asyncio.to_thread(truth, args.base)
                        score(sc, r, end_truth or start_truth, dist, start_truth)
                        tr = await asyncio.to_thread(api, args.base, "/api/trace", None, 10.0)
                        if r.get("mission_id") and tr.get("id") != r["mission_id"]:
                            tr = {}
                        r.update({k: v for k, v in trace_columns(tr, start_truth, tgt).items() if v is not None or k not in r})
                        r.update({
                            "run_id": run_id, "trial": trial, "seed": seed, "scene": scene, "mode": mode, "scenario": sc["id"],
                            "disturbance": dist, "paraphrased": paraphrased, "randomized": bool(layout),
                            "layout": layout or None,
                        })
                        if disturber is not None and disturber.sim_t is not None:
                            r["disturbance_sim_t"] = round(disturber.sim_t, 2)
                            if r.get("first_replan_sim_t") is not None:
                                r["replan_latency_s"] = round(r["first_replan_sim_t"] - disturber.sim_t, 2)
                        print("     ", "PASS" if r["passed"] else "FAIL",
                              {k: v for k, v in r.items() if k.endswith(("_cm", "_deg")) or k in ("status", "error", "planner_source")}, flush=True)
                        if r["accepted"]:
                            if rec:
                                rec.note = "returning home"
                            r["returned_home"], r["twin_restarted"] = await go_home(args.base, home)
                            await asyncio.sleep(1.0)
                        if rec:
                            rec.stop()
                            r["video"], r["still"] = f"videos/{name}.mp4", f"images/{name}.jpg"
                        results.append(r)
                        jsonl.write(json.dumps({**r, "trace": tr}, default=str) + "\n")
                        jsonl.flush()
                        write_rows(csv_path, [r], append=args.append or len(results) > 1)
    finally:
        jsonl.close()
        try:
            api(args.base, "/api/twin/link_delay", {"ms": 0.0})
            place(args.base, nominal_movable)
        except Exception:
            pass

    if not results:
        print("No trials ran.")
        return 1
    meta = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "run_id": run_id, "model": cfg.get("model_name"),
            "airframe": cfg.get("robot_name"), "scene": cfg.get("scene"), "modes": args.modes,
            "disturbances": args.disturbances, "seeds": args.seeds, "randomize": args.randomize,
            "paraphrase": args.paraphrase,
            "passed": sum(r["passed"] for r in results), "total": len(results)}
    if plain:
        write_plain_report(out_dir, meta, results, args)
    write_matrix_summary(out_dir / f"matrix_{run_id}.md", meta, results)
    print(f"\n{meta['passed']}/{meta['total']} passed. Trials in {csv_path}")
    return 0


def write_plain_report(out_dir: Path, meta: dict, results: list, args) -> None:
    if args.only and args.merge and (out_dir / "benchmark.json").is_file():
        # Re-running some cases: keep the others from the previous run.
        done = {r["id"] for r in results}
        old = json.loads((out_dir / "benchmark.json").read_text(encoding="utf-8"))["results"]
        for r in results:
            r["rerun"] = True
        results = sorted([r for r in old if r["id"] not in done] + results, key=lambda r: r["id"])
        meta = {**meta, "passed": sum(r["passed"] for r in results), "total": len(results)}
    (out_dir / "benchmark.json").write_text(json.dumps({"meta": meta, "results": results}, indent=2, default=str), encoding="utf-8")
    cols = ["id", "prompt", "status", "passed", "object_matched", "target_error_cm", "standoff_error_cm", "heading_error_deg",
            "altitude_error_cm", "estimator_error_cm", "sim_time_s", "path_m", "plan_latency_s"]
    lines = [f"# Twin flight benchmark\n\n{meta['created']} · {meta['airframe']} · {meta['scene']} · model {meta['model']} · "
             f"**{meta['passed']}/{meta['total']} passed**\n",
             "Scored against simulator ground truth: true vehicle position while hovering versus true object position.\n",
             "| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for r in results:
        lines.append("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |")
    (out_dir / "benchmark.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_matrix_summary(path: Path, meta: dict, results: list) -> None:
    """Pass counts per mode and disturbance. Statistics belong to the analysis step."""
    cells = {}
    for r in results:
        c = cells.setdefault((r["mode"], r["disturbance"]), [0, 0])
        c[0] += bool(r["passed"])
        c[1] += 1
    modes = list(dict.fromkeys(r["mode"] for r in results))
    dists = list(dict.fromkeys(r["disturbance"] for r in results))
    lines = [f"# Evaluation matrix {meta['run_id']}\n",
             f"{meta['created']} · model {meta['model']} · seeds {meta['seeds']} · randomize {meta['randomize']} · "
             f"paraphrase {meta['paraphrase']}\n",
             "Passed / trials.\n",
             "| mode | " + " | ".join(dists) + " |", "|---|" + "---|" * len(dists)]
    for m in modes:
        row = [f"{cells[(m, d)][0]}/{cells[(m, d)][1]}" if (m, d) in cells else "–" for d in dists]
        lines.append(f"| {m} | " + " | ".join(row) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
