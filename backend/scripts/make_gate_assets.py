"""
Builds the poster images for the twin's gate scenes (scene_gates_*.json).

Photos and logos come from Wikimedia Commons (openly licensed; credits are
written to matlab/twin/assets/gates/CREDITS.md). Digit posters are drawn here.
Run once:
    py -3 backend/scripts/make_gate_assets.py
"""
import json
import time
import urllib.parse
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "matlab" / "twin" / "assets" / "gates"
SRC = ROOT / "src"
UA = {"User-Agent": "twin-asset-fetch/0.1 (research prototype)"}
API = "https://commons.wikimedia.org/w/api.php?"
SIZE = 512

# poster name -> exact Commons title, or "?query" for the first bitmap hit
FILES = {
    "dog": "?Labrador retriever dog portrait",
    "horse": "File:Horse December 2014-1.jpg",
    "parrot": "?scarlet macaw parrot",
    "tux": "File:Linux mascot tux.png",
    "python": "File:Python logo (2025).png",
    "chrome": "File:Google Chrome icon (February 2022).svg",
    "fruit": "File:Fruit Salad Bowl.jpg",
    "pizza": "?pepperoni pizza whole",
    "burger": "File:Big Mac hamburger.jpg",
    "monalisa": "File:Mona Lisa.jpg",
    "scream": "File:The Scream Pastel.jpg",
    "pearl": "?Vermeer Girl with a Pearl Earring",
    "cowboy": "File:Grabill - The Cow Boy-edit.jpg",
    "astronaut": "File:Aldrin Apollo 11 original.jpg",
    "firefighter": "?firefighter helmet portrait",
}


def _get(url: str) -> bytes:
    for attempt in range(6):
        try:
            return urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=30).read()
        except urllib.error.HTTPError as e:
            if e.code != 429:
                raise
            time.sleep(8 * (attempt + 1))
    raise RuntimeError("rate limited: " + url)


def resolve(spec: str):
    if spec.startswith("?"):
        q = {"action": "query", "format": "json", "generator": "search", "gsrsearch": spec[1:] + " filetype:bitmap",
             "gsrnamespace": 6, "gsrlimit": 5, "prop": "imageinfo", "iiprop": "url|extmetadata", "iiurlwidth": SIZE}
    else:
        q = {"action": "query", "format": "json", "titles": spec, "prop": "imageinfo", "iiprop": "url|extmetadata", "iiurlwidth": SIZE}
    d = json.loads(_get(API + urllib.parse.urlencode(q)))
    pages = sorted((d.get("query", {}).get("pages") or {}).values(), key=lambda p: p.get("index", 0))
    for p in pages:
        ii = (p.get("imageinfo") or [None])[0]
        if ii:
            lic = (ii.get("extmetadata", {}).get("LicenseShortName") or {}).get("value", "?")
            return p["title"], lic, ii["thumburl"]
    raise RuntimeError("not found: " + spec)


def square(img: np.ndarray, name: str) -> np.ndarray:
    """Centre-crop photos; logos on transparent or white backgrounds are padded."""
    if img.ndim == 3 and img.shape[2] == 4:
        a = img[..., 3:4].astype(np.float32) / 255
        img = (img[..., :3] * a + 255 * (1 - a)).astype(np.uint8)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    h, w = img.shape[:2]
    if name in ("tux", "python", "chrome"):
        s = max(h, w)
        pad = np.full((s, s, 3), 255, np.uint8)
        pad[(s - h) // 2:(s - h) // 2 + h, (s - w) // 2:(s - w) // 2 + w] = img
        img = pad
    else:
        s = min(h, w)
        y0, x0 = (h - s) // 2, (w - s) // 2
        img = img[y0:y0 + s, x0:x0 + s]
    return cv2.resize(img, (SIZE, SIZE), interpolation=cv2.INTER_AREA)


def digit(d: str) -> np.ndarray:
    img = np.full((SIZE, SIZE, 3), 250, np.uint8)
    (tw, th), base = cv2.getTextSize(d, cv2.FONT_HERSHEY_DUPLEX, 11, 22)
    cv2.putText(img, d, ((SIZE - tw) // 2, (SIZE + th) // 2), cv2.FONT_HERSHEY_DUPLEX, 11, (15, 15, 15), 22, cv2.LINE_AA)
    return img


def main() -> None:
    SRC.mkdir(parents=True, exist_ok=True)
    credits = ["# Gate poster credits\n", "Images from Wikimedia Commons; see each file page for the full licence.\n"]
    for name, spec in FILES.items():
        out = ROOT / f"{name}.png"
        if out.exists():
            continue
        title, lic, url = resolve(spec)
        raw = _get(url)
        (SRC / f"{name}.bin").write_bytes(raw)
        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)
        cv2.imwrite(str(out), square(img, name))
        print(f"{name:12s} {title} [{lic}]")
        time.sleep(2.5)
    for p in sorted(ROOT.glob("*.png")):
        pass
    for d in "0123456789":
        cv2.imwrite(str(ROOT / f"digit_{d}.png"), digit(d))
    # Rebuild credits from the cache of titles
    meta = {}
    for name, spec in FILES.items():
        meta[name] = spec
    (ROOT / "CREDITS.md").write_text("".join(credits) + "\n".join(f"- {k}: {v}" for k, v in meta.items()) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
