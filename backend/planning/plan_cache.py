"""Disk cache of Gemini request → response → skills, plus numeric templates."""
from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

from backend.planning.intent_router import normalize_command, template_key
from backend.schemas.mission import SkillPrimitive

_DEFAULT_PATH = Path(__file__).resolve().parents[2] / "cache" / "mission_plans.json"


def _cache_path() -> Path:
    override = os.getenv("MISSION_CACHE_PATH")
    return Path(override) if override else _DEFAULT_PATH


def load_store() -> Dict[str, Any]:
    path = _cache_path()
    if not path.exists():
        return {"exact": {}, "templates": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        data.setdefault("exact", {})
        data.setdefault("templates", {})
        return data
    except (OSError, json.JSONDecodeError):
        return {"exact": {}, "templates": {}}


def save_store(store: Dict[str, Any]) -> None:
    path = _cache_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(store, indent=2), encoding="utf-8")


def lookup(command: str) -> Optional[Dict[str, Any]]:
    store = load_store()
    key = normalize_command(command)
    if key in store["exact"]:
        hit = store["exact"][key]
        hit["hit"] = "exact"
        return hit
    tmpl = template_key(command)
    if tmpl in store["templates"]:
        hit = _fill_template(store["templates"][tmpl], command)
        if hit:
            hit["hit"] = "template"
            return hit
    return None


def store_entry(
    command: str,
    request_payload: Dict[str, Any],
    response_payload: Dict[str, Any],
    skills: List[SkillPrimitive],
    *,
    allow_template: bool = True,
) -> None:
    store = load_store()
    rec = {
        "command": command,
        "request": request_payload,
        "response": response_payload,
        "skills": [s.model_dump() for s in skills],
    }
    store["exact"][normalize_command(command)] = rec
    if allow_template:
        store["templates"][template_key(command)] = rec
    save_store(store)


def _fill_template(rec: Dict[str, Any], command: str) -> Optional[Dict[str, Any]]:
    old_cmd = rec.get("command") or ""
    old_nums = [float(n) for n in re.findall(r"-?\d+(?:\.\d+)?", old_cmd)]
    new_nums = [float(n) for n in re.findall(r"-?\d+(?:\.\d+)?", command)]
    skills = [
        SkillPrimitive(skill=s.get("skill") or s.get("name") or "", params=dict(s.get("params") or {}))
        for s in rec.get("skills") or []
    ]
    if old_nums and new_nums and len(old_nums) == len(new_nums):
        mapping = list(zip(old_nums, new_nums))
        for skill in skills:
            for k, v in list(skill.params.items()):
                if isinstance(v, (int, float)):
                    for old, new in mapping:
                        if abs(float(v) - old) < 1e-6:
                            skill.params[k] = new
                            break
        rec = dict(rec)
        rec["skills"] = [s.model_dump() for s in skills]
        rec["command"] = command
        return rec
    rec = dict(rec)
    rec["skills"] = [s.model_dump() for s in skills]
    rec["command"] = command
    return rec
