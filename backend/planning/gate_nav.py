"""
Gate navigation: "fly through the gate with the dog".

One request to the vision-language model on the onboard (forward) camera:
it describes every gate's picture, picks the one the instruction refers to
(reasoning about content, e.g. which food is healthy, what 2+4 is) and boxes
that gate's picture. The box is turned into a position in metres from camera
geometry alone: the picture's known width gives range, and the known height
of the picture centre gives a second range from the viewing elevation. The
plan is then approach point -> through the opening -> a point beyond it.

Nothing here reads where the gates really are. What it does assume, as a
real deployment would be told: picture width and height above the floor, the
opening's height, and that gates face the direction the vehicle is facing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from backend.core.logger import get_logger
from backend.models.gemini_provider import GEMINI_MODEL_CANDIDATES, _extract_json
from backend.schemas.mission import SkillPrimitive
from backend.vision.bbox_convert import to_pixels as bbox_to_pixels

logger = get_logger("GateNav")


@dataclass
class GateSpec:
    """What the operator knows about the gates' dimensions (metres)."""
    picture_width: float = 0.50
    picture_center_z: float = 1.25
    opening_center_z: float = 0.525
    approach_m: float = 0.75
    exit_m: float = 0.85


@dataclass
class GateChoice:
    index: int
    picture: str
    reason: str
    gates: List[dict] = field(default_factory=list)      # every gate: picture, shape, box (px)
    box: Optional[tuple] = None                           # chosen picture box (u0, v0, u1, v1)
    model: str = ""
    raw: str = ""


PROMPT = (
    "You are the vision system of a small drone. The image is its forward camera. It shows several gates "
    "in a row. Each gate has a framed picture on top, a coloured outline shape under the picture, and an "
    "opening below. Instruction: \"{instruction}\".\n"
    "1. List every gate left to right: what its picture shows and its shape.\n"
    "2. Choose the one gate that fulfils the instruction. Reason about content when needed (which food is "
    "healthy, what a sum equals, what or who is shown). Choose exactly one.\n"
    "Return ONLY JSON: {{\"gates\": [{{\"picture\": <string>, \"shape\": <string>, "
    "\"picture_box_2d\": [ymin, xmin, ymax, xmax]}}], \"choice\": <0-based index into gates>, \"reason\": <string>}} "
    "with boxes normalised to 0-1000 and tight around the picture only. If no gate fits, use choice -1."
)


class GateVLM:
    def __init__(self, provider):
        self.provider = provider

    def choose(self, frame_bgr: np.ndarray, instruction: str) -> Optional[GateChoice]:
        import cv2
        from google.genai import types
        client = self.provider.client
        if client is None:
            logger.warning("No Gemini client: gate not chosen.")
            return None
        _, jpg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        part = types.Part.from_bytes(data=jpg.tobytes(), mime_type="image/jpeg")
        h, w = frame_bgr.shape[:2]
        prompt = PROMPT.format(instruction=instruction)
        models = [self.provider.model_name] + [m for m in GEMINI_MODEL_CANDIDATES if m != self.provider.model_name]
        for name in models:
            try:
                r = client.models.generate_content(
                    model=name, contents=[part, prompt],
                    config=types.GenerateContentConfig(response_mime_type="application/json"))
                raw = getattr(r, "text", "") or ""
                data = _extract_json(raw)
                if isinstance(data, list):
                    data = data[0] if data else {}
                choice = self._parse(data, w, h)
                choice.model, choice.raw = name, " ".join(raw.split())
                self.provider.model_name = name
                logger.info(f"[{name}] gate {choice.index}: {choice.picture} - {choice.reason}")
                return choice
            except Exception as exc:
                logger.warning(f"Gate VLM {name} failed: {str(exc)[:120]}")
        return None

    @staticmethod
    def _parse(data: dict, w: int, h: int) -> GateChoice:
        gates = []
        for g in data.get("gates") or []:
            box = None
            b = g.get("picture_box_2d")
            if isinstance(b, (list, tuple)) and len(b) == 4:
                px = bbox_to_pixels(b, w, h, order="yxyx", assume="norm1000")
                box = tuple(int(v) for v in px) if px else None
            gates.append({"picture": g.get("picture", ""), "shape": g.get("shape", ""), "box": box})
        idx = int(data.get("choice", -1))
        if idx < 0 or idx >= len(gates) or gates[idx]["box"] is None:
            return GateChoice(index=-1, picture="", reason=str(data.get("reason", "")), gates=gates)
        return GateChoice(index=idx, picture=gates[idx]["picture"], reason=str(data.get("reason", "")),
                          gates=gates, box=gates[idx]["box"])


def pixel_ray(u: float, v: float, cam: dict, pose: dict, mount_fwd: float = 0.03, tilt_down_deg: float = 20.0):
    """World-frame camera origin and unit ray for pixel (u, v)."""
    psi = math.radians(pose["yaw"])
    p = math.radians(tilt_down_deg)
    b = np.array([math.cos(psi), math.sin(psi), 0.0])
    right = np.array([math.sin(psi), -math.cos(psi), 0.0])
    fwd = b * math.cos(p) + np.array([0, 0, -1.0]) * math.sin(p)
    up = b * math.sin(p) + np.array([0, 0, 1.0]) * math.cos(p)
    xc = (u - cam["cx"]) / cam["fx"]
    yc = (v - cam["cy"]) / cam["fy"]
    ray = fwd + xc * right - yc * up
    ray /= np.linalg.norm(ray)
    origin = np.array([pose["x"], pose["y"], pose["z"]]) + b * mount_fwd
    return origin, ray


def locate_gate(box: tuple, cam: dict, pose: dict, spec: GateSpec, tilt_down_deg: float = 20.0) -> dict:
    """Gate position (x, y) from one picture box."""
    u0, v0, u1, v1 = box
    uc, vc = (u0 + u1) / 2.0, (v0 + v1) / 2.0
    origin, ray = pixel_ray(uc, vc, cam, pose, tilt_down_deg=tilt_down_deg)
    axis = pixel_ray(cam["cx"], cam["cy"], cam, pose, tilt_down_deg=tilt_down_deg)[1]
    # depth along the optical axis from the picture's apparent width, then along the ray
    depth = cam["fx"] * spec.picture_width / max(8.0, float(u1 - u0))
    t_width = depth / max(1e-3, float(ray @ axis))
    # range from the known height of the picture centre
    t_height = (spec.picture_center_z - origin[2]) / ray[2] if ray[2] > 0.05 else None
    # the height-based range does not depend on how tightly the model boxed the picture
    t = t_width if t_height is None or not (0.5 < t_height < 6.0) else 0.25 * t_width + 0.75 * t_height
    p = origin + t * ray
    return {"x": float(p[0]), "y": float(p[1]), "range_width": float(t_width),
            "range_height": None if t_height is None else float(t_height), "range": float(t)}


def gate_skills(gate: dict, pose: dict, spec: GateSpec, ws: dict) -> List[SkillPrimitive]:
    """Approach in front of the gate, fly through the opening, continue past it."""
    psi = math.radians(pose["yaw"])
    d = np.array([math.cos(psi), math.sin(psi)])
    g = np.array([gate["x"], gate["y"]])
    z = round(min(ws["z"][1], max(ws["z"][0], spec.opening_center_z)), 2)

    def clamp(p):
        return [round(max(ws["x"][0], min(ws["x"][1], p[0])), 2), round(max(ws["y"][0], min(ws["y"][1], p[1])), 2)]

    a, b, c = clamp(g - d * spec.approach_m), clamp(g), clamp(g + d * spec.exit_m)
    return [SkillPrimitive(skill="MOVE_TO", params={"x": a[0], "y": a[1], "z": z, "speed": 0.4}),
            SkillPrimitive(skill="MOVE_TO", params={"x": b[0], "y": b[1], "z": z, "speed": 0.3}),
            SkillPrimitive(skill="MOVE_TO", params={"x": c[0], "y": c[1], "z": z, "speed": 0.3}),
            SkillPrimitive(skill="HOVER", params={"t": 2.0})]
