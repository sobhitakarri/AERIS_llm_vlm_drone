"""Code-first intent parse: dictionary / regex before any LLM call."""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from backend.planning.shape_paths import parse_shape_name, wants_closed_path


def normalize_command(command: str) -> str:
    s = (command or "").lower().strip()
    s = s.replace("take-off", "take off").replace("takeoff", "take off")
    s = re.sub(r"\s+", " ", s)
    return s


def template_key(command: str) -> str:
    return re.sub(r"-?\d+(?:\.\d+)?", "N", normalize_command(command)).strip()


@dataclass
class ParsedIntent:
    raw: str
    normalized: str
    template_key: str
    intent: str
    use_code: bool
    vision: bool
    shape: Optional[str] = None
    target: Optional[str] = None
    slots: Dict[str, float] = field(default_factory=dict)


_COMPOUND = re.compile(
    r"\b(then|otherwise|unless|after that|if it|if you|if the|"
    r"trace|letter|draw|write|patrol|spiral|racetrack|"
    r"figure eight|plus sign|laps?)\b"
)


def _is_land_only(cmd: str) -> bool:
    if not re.search(r"\bland\b|\blanding\b|\bcome down\b", cmd):
        return False
    rest = re.sub(r"\b(please|now|safely|softly|and|then)\b", " ", cmd)
    rest = re.sub(r"\b(land|landing|come down)\b", " ", rest)
    rest = re.sub(r"[^a-z0-9]+", " ", rest).strip()
    return len(rest.split()) == 0


def _is_return_only(cmd: str) -> bool:
    if not (re.search(r"\b(return|go home|come home)\b", cmd) or cmd.strip() in {"home", "rth"}):
        return False
    rest = re.sub(r"\b(please|now|and|then|to home|home)\b", " ", cmd)
    rest = re.sub(r"\b(return|go home|come home|rth)\b", " ", rest)
    rest = re.sub(r"[^a-z0-9]+", " ", rest).strip()
    return len(rest.split()) == 0


def parse_intent(command: str, context: Optional[Dict[str, Any]] = None) -> ParsedIntent:
    raw = command or ""
    cmd = normalize_command(raw)
    tmpl = template_key(raw)
    slots = _extract_slots(cmd, context)
    vision = bool(re.search(r"\b(find|inspect|look at|search|bottle|cube|object)\b", cmd))
    compound = bool(_COMPOUND.search(cmd))

    if vision and re.search(r"\b(find|inspect|look at|search)\b", cmd):
        tgt = _extract_target(cmd)
        # Conditionals / extra maneuvers need Gemini or VLM, not the FIND template.
        use_code = not compound
        return ParsedIntent(raw, cmd, tmpl, "vision", use_code, True, target=tgt, slots=slots)

    if _is_return_only(cmd):
        return ParsedIntent(raw, cmd, tmpl, "return", True, False, slots=slots)

    if _is_land_only(cmd) and not wants_closed_path(cmd):
        return ParsedIntent(raw, cmd, tmpl, "land", True, False, slots=slots)

    if wants_closed_path(cmd) and not re.search(r"\bcircl|\bround\b", cmd):
        return ParsedIntent(
            raw, cmd, tmpl, "shape", True, False,
            shape=parse_shape_name(cmd), slots=slots,
        )

    if re.search(r"\bcircl|\bround\b", cmd) and not compound:
        return ParsedIntent(raw, cmd, tmpl, "circle", True, False, slots=slots)

    if re.search(r"\b(rotate|yaw|spin|turn)\b", cmd) and not compound:
        return ParsedIntent(raw, cmd, tmpl, "rotate", True, False, slots=slots)

    # "fly to 5 meters" is altitude, not a 2D goto. Need x,y for code goto.
    if re.search(r"\b(go to|goto|move to|fly to|navigate to)\b", cmd) and "x" in slots and "y" in slots:
        return ParsedIntent(raw, cmd, tmpl, "goto", True, False, slots=slots)

    if re.search(r"\b(left|right|forward|back|backward)\b", cmd) and re.search(r"\d", cmd) and not compound:
        return ParsedIntent(raw, cmd, tmpl, "relative", True, False, slots=slots)

    if "take off" in cmd and not compound:
        return ParsedIntent(raw, cmd, tmpl, "takeoff", True, False, slots=slots)

    if re.search(r"\bhover\b|\bhold\b|\bstay\b", cmd) and not re.search(r"\bfly\b", cmd) and not compound:
        return ParsedIntent(raw, cmd, tmpl, "hover", True, False, slots=slots)

    return ParsedIntent(raw, cmd, tmpl, "unknown", False, False, slots=slots)


def _extract_target(cmd: str) -> str:
    m = re.search(r"(?:find|inspect|look at|search for)\s+(?:the\s+)?(.+)$", cmd)
    if m:
        return m.group(1).strip()
    return "object"


def _extract_slots(cmd: str, context: Optional[Dict[str, Any]]) -> Dict[str, float]:
    slots: Dict[str, float] = {}
    ctx = context or {}
    pose = ctx.get("pose") or {}

    m = re.search(r"(?:to|at|altitude|height)\s*(-?\d+(?:\.\d+)?)\s*m?", cmd)
    if m:
        slots["z"] = float(m.group(1))
    else:
        nums = [float(n) for n in re.findall(r"-?\d+(?:\.\d+)?", cmd)]
        if "take off" in cmd and nums:
            slots["z"] = nums[0]

    m = re.search(r"(?:hover|for|wait)\s*(-?\d+(?:\.\d+)?)\s*(?:s|sec|second)?", cmd)
    if m:
        slots["t"] = float(m.group(1))

    m = re.search(r"(?:rotate|yaw|spin|turn)\s*(-?\d+(?:\.\d+)?)", cmd)
    if m:
        slots["angle"] = float(m.group(1))
    elif "rotate" in cmd or "yaw" in cmd:
        slots["angle"] = 90.0

    m = re.search(
        r"(?:go to|goto|move to|fly to|navigate to)\s*"
        r"(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)",
        cmd,
    )
    if m:
        slots["x"] = float(m.group(1))
        slots["y"] = float(m.group(2))
    m = re.search(r"x\s*=\s*(-?\d+(?:\.\d+)?)", cmd)
    if m:
        slots["x"] = float(m.group(1))
    m = re.search(r"y\s*=\s*(-?\d+(?:\.\d+)?)", cmd)
    if m:
        slots["y"] = float(m.group(1))

    dist = None
    md = re.search(r"(-?\d+(?:\.\d+)?)\s*m", cmd)
    if md:
        dist = float(md.group(1))
    px, py = float(pose.get("x", 0.0)), float(pose.get("y", 0.0))
    if dist is not None:
        if re.search(r"\bleft\b", cmd):
            slots["x"] = px
            slots["y"] = py + dist
        elif re.search(r"\bright\b", cmd):
            slots["x"] = px
            slots["y"] = py - dist
        elif re.search(r"\bforward\b", cmd):
            slots["x"] = px + dist
            slots["y"] = py
        elif re.search(r"\b(back|backward)\b", cmd):
            slots["x"] = px - dist
            slots["y"] = py

    m = re.search(r"radius\s*(-?\d+(?:\.\d+)?)", cmd)
    if m:
        slots["radius"] = float(m.group(1))

    return slots
