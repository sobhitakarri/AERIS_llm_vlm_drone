"""
Local Qwen2-VL Provider via Ollama API.
Supports dual-mode:
1. LLM Mode: Natural language prompt decomposition into JSON skill primitives.
2. VLM Mode: Visual grounding of objects into bounding boxes [ymin, xmin, ymax, xmax].
"""
import base64
import json
import re
import urllib.request
import urllib.error
from typing import Optional, List, Dict, Any

from backend.models.base_llm import LLMProvider
from backend.models.base_vlm import VLMProvider
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.perception import DetectedObject, BoundingBox
from backend.vision.bbox_convert import (
    plausibility as bbox_plausibility,
    select_qwen_box,
)
from backend.core.logger import get_logger
from backend.planning.intent_router import parse_intent
from backend.planning.local_planner import plan_from_intent

logger = get_logger("QwenProvider")


class QwenProvider(LLMProvider, VLMProvider):
    """
    Local foundation model provider using Ollama running Qwen2-VL-2B GGUF.
    Endpoints used:
    - http://localhost:11434/api/chat
    """

    def __init__(
        self,
        endpoint_url: str = "http://localhost:11434",
        model_name: str = "hf.co/ggml-org/Qwen2-VL-2B-Instruct-GGUF:Q4_K_M"
    ):
        self.endpoint_url = endpoint_url.rstrip("/")
        self.model_name = model_name
        self.is_live = False
        self._check_connection()

    def _check_connection(self) -> bool:
        try:
            req = urllib.request.Request(f"{self.endpoint_url}/api/tags")
            with urllib.request.urlopen(req, timeout=3) as resp:
                data = json.loads(resp.read().decode())
                models = [m.get("name") for m in data.get("models", [])]
                logger.info(f"[Qwen Local] Ollama connected. Available models: {models}")
                if self.model_name not in models and models:
                    # Pick the first matching qwen model if exact tag varies
                    for m in models:
                        if "qwen" in m.lower():
                            self.model_name = m
                            break
                    else:
                        self.model_name = models[0]
                self.is_live = True
                logger.info(f"[Qwen Local] Active model: '{self.model_name}'")
                return True
        except Exception as e:
            logger.warning(f"[Qwen Local] Ollama check failed: {e}. Ensure 'ollama serve' is running.")
            self.is_live = False
            return False

    def _call_ollama(self, messages: List[Dict[str, Any]], format_json: bool = False) -> str:
        payload = {
            "model": self.model_name,
            "messages": messages,
            "stream": False,
            "options": {
                "temperature": 0.1,
                "num_predict": 1024,  # Raised from 512: multi-waypoint plans need more tokens
                # A single 640x480 frame costs Qwen2-VL several hundred vision
                # tokens, which overflows Ollama's small default context.
                "num_ctx": 8192,
            }
        }
        if format_json:
            payload["format"] = "json"

        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            f"{self.endpoint_url}/api/chat",
            data=body,
            headers={"Content-Type": "application/json"}
        )

        with urllib.request.urlopen(req, timeout=60) as resp:
            data = json.loads(resp.read().decode())
            return data.get("message", {}).get("content", "")

    # ------------------------------------------------------------------
    # Unified fallback for geometric commands & primitives using local_planner
    # ------------------------------------------------------------------
    def _fallback_plan(self, raw_command: str, context: Optional[dict] = None) -> Optional[List[SkillPrimitive]]:
        """
        Deterministic skill generator for well-known geometric patterns and primitives.
        Leverages shape_paths (20+ shapes) and local_planner without duplicating logic.
        """
        try:
            from backend.planning.local_planner import local_plan
            skills = local_plan(raw_command, context)
            if skills:
                return skills
        except Exception as e:
            logger.warning(f"[Qwen Local] Local planner fallback error: {e}")
        return None

    def plan(self, raw_command: str, capabilities: RobotCapabilities, context=None) -> MissionPlan:
        logger.info(f"[Qwen Local] Planning command: '{raw_command}'")

        # ── Step 1: Try deterministic intent routing BEFORE the LLM ──────────
        # intent_router + local_planner covers: shape, circle, takeoff, land,
        # hover, goto, rotate, relative-move, return — all with correct waypoints.
        # shape_paths.py supports 20+ shapes (star, heart, spiral, pentagon,
        # dodecagon, cross, L-shape, rhombus, trapezoid, parallelogram, n-gon…).
        try:
            intent = parse_intent(raw_command, context)
            logger.info(f"[Qwen Local] Intent: '{intent.intent}', use_code={intent.use_code}, shape={intent.shape!r}")
            if intent.use_code and intent.intent != "unknown":
                code_skills = plan_from_intent(intent, context)
                if code_skills:
                    shape_label = intent.shape or intent.intent
                    return MissionPlan(
                        raw_command=raw_command,
                        reasoning=(
                            f"Deterministic plan for intent='{intent.intent}'"
                            + (f", shape='{shape_label}'" if intent.shape else "")
                            + f". Generated {len(code_skills)} skill(s) via local planner."
                        ),
                        skills=code_skills,
                        source="local-planner",
                        pipeline=["intent-router", "local-planner", "shape-paths"],
                    )
        except Exception as e:
            logger.warning(f"[Qwen Local] Intent routing failed ({e}), falling through to LLM.")

        # ── Step 2: Parse altitude (for LLM + final fallback) ─────────────────
        alt = 1.0
        m = re.search(r"(\d+(?:\.\d+)?)\s*m", raw_command.lower())
        if m:
            alt = float(m.group(1))

        system_prompt = (
            "You are an autonomous UAV flight planner for an indoor quadrotor.\n"
            "Allowed skills: TAKEOFF(z, speed), LAND(), HOVER(t), MOVE_TO(x, y, z, speed), ROTATE(angle), CIRCLE(radius).\n"
            "Workspace limits: X and Y in [-1.5, 1.5] meters, Z (altitude) in [0.3, 1.5] meters.\n"
            "IMPORTANT: For geometric patterns generate ALL required waypoints.\n"
            "  - 'fly a square' → 5 MOVE_TO calls (4 corners + return to start)\n"
            "  - 'fly a circle' → use CIRCLE skill with a radius\n"
            "  - 'fly a triangle' → 4 MOVE_TO calls (3 corners + return to start)\n"
            "Example – 'take off to 1m and fly a square':\n"
            "{\n"
            '  "reasoning": "Fly a square at 1m altitude. Side length ~0.8m centred at origin. '
            'Visiting TL(-0.8,0.8), TR(0.8,0.8), BR(0.8,-0.8), BL(-0.8,-0.8) then back to start.",\n'
            '  "skills": [\n'
            '    {"skill": "TAKEOFF",  "params": {"z": 1.0, "speed": 0.4}},\n'
            '    {"skill": "MOVE_TO",  "params": {"x": -0.8, "y":  0.8, "z": 1.0, "speed": 0.4}},\n'
            '    {"skill": "MOVE_TO",  "params": {"x":  0.8, "y":  0.8, "z": 1.0, "speed": 0.4}},\n'
            '    {"skill": "MOVE_TO",  "params": {"x":  0.8, "y": -0.8, "z": 1.0, "speed": 0.4}},\n'
            '    {"skill": "MOVE_TO",  "params": {"x": -0.8, "y": -0.8, "z": 1.0, "speed": 0.4}},\n'
            '    {"skill": "MOVE_TO",  "params": {"x": -0.8, "y":  0.8, "z": 1.0, "speed": 0.4}},\n'
            '    {"skill": "HOVER",    "params": {"t": 2.0}},\n'
            '    {"skill": "LAND",     "params": {}}\n'
            "  ]\n"
            "}"
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Generate a flight plan for: {raw_command}"}
        ]

        try:
            content = self._call_ollama(messages, format_json=True)
            logger.info(f"[Qwen Local] Ollama response: {content[:200]}...")
            data = json.loads(content)
            skills = []
            for item in data.get("skills", []):
                name = item.get("skill", "").upper()
                params = item.get("params", {})
                skills.append(SkillPrimitive(skill=name, params=params))

            # --- Sanity check: did the LLM actually generate multi-waypoint paths? ---
            move_to_count = sum(1 for s in skills if s.skill == "MOVE_TO")
            needs_pattern = any(kw in raw_command.lower() for kw in ["square", "circle", "triangle", "figure"])
            if needs_pattern and move_to_count < 3:
                logger.warning(
                    f"[Qwen Local] LLM generated only {move_to_count} MOVE_TO for a geometric command. "
                    "Activating pattern fallback."
                )
                fallback_skills = self._fallback_plan(raw_command, context)
                if fallback_skills:
                    return MissionPlan(
                        raw_command=raw_command,
                        reasoning=(
                            f"{data.get('reasoning', '')} "
                            "[Deterministic fallback applied: geometric trajectory generated]"
                        ),
                        skills=fallback_skills,
                        source="qwen2-vl-local+planner-fallback",
                        pipeline=["qwen2-vl-ollama", "local-planner"]
                    )

            return MissionPlan(
                raw_command=raw_command,
                reasoning=data.get("reasoning", "Generated by local Qwen2-VL via Ollama."),
                skills=skills,
                source="qwen2-vl-local",
                pipeline=["qwen2-vl-ollama"]
            )
        except Exception as e:
            logger.warning(f"[Qwen Local] Generation error: {e}. Trying deterministic fallback.")

            # Try deterministic fallback before giving up
            pattern_skills = self._fallback_plan(raw_command, context)
            if pattern_skills:
                return MissionPlan(
                    raw_command=raw_command,
                    reasoning=f"Deterministic fallback used after Ollama error: {e}",
                    skills=pattern_skills,
                    source="local-planner-fallback",
                    pipeline=["local-planner"]
                )

            return MissionPlan(
                raw_command=raw_command,
                reasoning=f"Fallback plan generated after Ollama error: {e}",
                skills=[
                    SkillPrimitive(skill="TAKEOFF", params={"z": alt, "speed": 0.4}),
                    SkillPrimitive(skill="HOVER", params={"t": 3.0}),
                    SkillPrimitive(skill="LAND", params={})
                ],
                source="qwen-fallback"
            )

    def resolve_target(self, image_bytes: bytes, target_description: str,
                       image_width: int = 0, image_height: int = 0) -> Optional[DetectedObject]:
        """
        Multimodal visual grounding:
        Passes the camera frame to Qwen2-VL to locate target object bounding box.

        Args:
            image_bytes: Raw image bytes (JPEG/PNG).
            target_description: Natural language description of the target.
            image_width: Actual pixel width of the image (0 = auto-detect).
            image_height: Actual pixel height of the image (0 = auto-detect).
        """
        logger.info(f"[Qwen Local VLM] Grounding target: '{target_description}'...")
        b64_img = base64.b64encode(image_bytes).decode("utf-8")

        # ── Auto-detect image size if not provided ────────────────────────────
        img_w, img_h = image_width, image_height
        if img_w == 0 or img_h == 0:
            try:
                import struct
                # Fast JPEG/PNG size detection without PIL
                if image_bytes[:2] == b'\xff\xd8':  # JPEG
                    import io
                    pos = 2
                    while pos < len(image_bytes):
                        marker = image_bytes[pos:pos+2]
                        pos += 2
                        if marker in (b'\xff\xc0', b'\xff\xc2'):
                            img_h, img_w = struct.unpack('>HH', image_bytes[pos+3:pos+7])
                            break
                        length = struct.unpack('>H', image_bytes[pos:pos+2])[0]
                        pos += length
                elif image_bytes[:8] == b'\x89PNG\r\n\x1a\n':  # PNG
                    img_w, img_h = struct.unpack('>II', image_bytes[16:24])
            except Exception:
                pass
            if img_w == 0 or img_h == 0:
                img_w, img_h = 640, 480  # safe fallback
                logger.warning(f"[Qwen Local VLM] Could not detect image size, using {img_w}x{img_h}")
            else:
                logger.info(f"[Qwen Local VLM] Detected image size: {img_w}x{img_h}")

        # This GGUF was trained on Qwen2-VL's 0–1000 boxes. Asking for raw
        # pixels made it emit 0–1000 numbers that then passed the pixel
        # overflow check on tall photographs and sat on the wrong half of
        # the object. Ask for 0–1000 and decode with select_qwen_box.
        prompt = (
            f"Locate the {target_description} in this image. "
            "Reply with ONLY a bounding box on one line: [x1, y1, x2, y2] "
            "using 0-1000 normalised coordinates "
            "(x=0 left, x=1000 right, y=0 top, y=1000 bottom). "
            "Fit the box tightly around the object, not the whole image. "
            "If the object is not in the image, reply exactly: NOT_FOUND. "
            "No explanation. Just the array."
        )

        messages = [{"role": "user", "content": prompt, "images": [b64_img]}]

        try:
            content = self._call_ollama(messages)
            logger.info(f"[Qwen Local VLM] Raw output: {content[:200]}")

            if "not_found" in content.lower():
                logger.info(f"[Qwen Local VLM] '{target_description}' reported absent.")
                return None

            bbox_vals = self._parse_bbox(content)
            if not bbox_vals:
                logger.warning(f"[Qwen Local VLM] Could not parse bbox from: {content[:100]}")
                return None

            box = self._to_pixels(bbox_vals, img_w, img_h)
            if box is None:
                logger.warning(
                    "[Qwen Local VLM] Rejected implausible box %s for '%s'.",
                    bbox_vals, target_description,
                )
                return None
            u_min, v_min, u_max, v_max = box

            plausibility = self._plausibility(u_min, v_min, u_max, v_max, img_w, img_h)
            if plausibility <= 0.0:
                logger.warning(
                    "[Qwen Local VLM] '%s' box px(%d,%d)-(%d,%d) failed plausibility gate.",
                    target_description, u_min, v_min, u_max, v_max,
                )
                return None

            bbox = BoundingBox(u_min=u_min, v_min=v_min, u_max=u_max, v_max=v_max)
            logger.info(
                f"[Qwen Local VLM] '{target_description}' raw{tuple(bbox_vals)} -> "
                f"px({u_min},{v_min})-({u_max},{v_max})  plausibility={plausibility:.2f}"
            )
            return DetectedObject(
                label=target_description,
                bbox=bbox,
                confidence=round(plausibility, 2),
            )
        except Exception as e:
            logger.warning(f"[Qwen Local VLM] Grounding failed: {e}")

        return None

    _to_pixels = staticmethod(select_qwen_box)
    _plausibility = staticmethod(bbox_plausibility)

    @staticmethod
    def _parse_bbox(text: str) -> Optional[tuple]:
        """
        Robust bbox parser — handles:
        - Raw array: [472, 0, 675, 514]
        - Markdown-fenced JSON: ```json\n{"bbox": [...]}```
        - Plain JSON with bbox / bbox_2d (Qwen2.5-VL) keys, object or list
        - Qwen2-VL grounding tokens: <|box_start|>(x1,y1),(x2,y2)<|box_end|>
        """
        # 1. Strip markdown fences
        text = re.sub(r"```[a-z]*\n?", "", text).strip()

        # 2. Qwen2-VL native grounding format: (x1,y1),(x2,y2)
        m = re.search(
            r"\(\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\)\s*,?\s*"
            r"\(\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\)",
            text,
        )
        if m:
            return tuple(float(g) for g in m.groups())

        # 3. JSON, either an object or a list of detections
        try:
            j = json.loads(text)
            if isinstance(j, list) and j:
                j = j[0]
            if isinstance(j, dict):
                b = (
                    j.get("bbox_2d") or j.get("bbox")
                    or j.get("bounding_box") or j.get("box")
                )
                if b and len(b) == 4:
                    return tuple(float(v) for v in b)
        except Exception:
            pass

        # 4. Bare array anywhere in the text
        m = re.search(
            r"\[\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*"
            r"(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\]",
            text,
        )
        if m:
            return tuple(float(g) for g in m.groups())

        # Deliberately no "first four integers anywhere" fallback: it turned
        # prose like "I see 2 objects in this 640 by 480 image" into a bbox.
        return None

