"""
Generates the image assets used by the MATLAB/Unreal digital twin:
surface textures and the ArUco calibration markers listed in the scene file.

Run once (or after editing the scene's marker list):
    py -3 backend/scripts/make_twin_assets.py
"""
import json
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TWIN_DIR = REPO_ROOT / "matlab" / "twin"
TEX_DIR = TWIN_DIR / "assets" / "tex"


def _fbm(rng: np.random.Generator, size: int, octaves: int = 6, persistence: float = 0.55) -> np.ndarray:
    """Tileable fractal noise in [0, 1]."""
    out = np.zeros((size, size), np.float32)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        cells = 2 ** (o + 2)
        grid = rng.random((cells, cells), dtype=np.float32)
        grid = np.pad(grid, ((0, 1), (0, 1)), mode="wrap")
        layer = cv2.resize(grid, (size + size // cells, size + size // cells), interpolation=cv2.INTER_CUBIC)
        out += amp * layer[:size, :size]
        total += amp
        amp *= persistence
    out /= total
    return (out - out.min()) / max(float(out.max() - out.min()), 1e-6)


def _save(name: str, img: np.ndarray) -> None:
    TEX_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(TEX_DIR / name), np.clip(img, 0, 255).astype(np.uint8))


def _tint(noise: np.ndarray, dark, light) -> np.ndarray:
    dark = np.array(dark, np.float32)
    light = np.array(light, np.float32)
    return dark + noise[..., None] * (light - dark)


def make_floor(rng) -> None:
    """Sealed concrete with trowel variation, fine speckle and hairline cracks."""
    n = 1024
    base = _fbm(rng, n, 6)
    speck = rng.normal(0.0, 1.0, (n, n)).astype(np.float32)
    speck = cv2.GaussianBlur(speck, (0, 0), 0.8)
    img = _tint(0.75 * base + 0.25 * _fbm(rng, n, 3), (118, 120, 122), (168, 170, 170))
    img += speck[..., None] * 5.0
    cracks = np.zeros((n, n), np.uint8)
    for _ in range(7):
        p = rng.integers(0, n, 2).astype(np.float32)
        ang = rng.uniform(0, 2 * np.pi)
        for _ in range(rng.integers(40, 120)):
            ang += rng.normal(0, 0.25)
            q = p + 9.0 * np.array([np.cos(ang), np.sin(ang)], np.float32)
            cv2.line(cracks, tuple(int(v) % n for v in p), tuple(int(v) % n for v in q), 255, 1, cv2.LINE_AA)
            p = q
    img -= cv2.GaussianBlur(cracks, (0, 0), 0.7)[..., None].astype(np.float32) * 0.16
    _save("floor_concrete.png", img)


def make_wall(rng) -> None:
    n = 512
    img = _tint(_fbm(rng, n, 5), (205, 210, 212), (226, 230, 231))
    img += cv2.GaussianBlur(rng.normal(0, 1, (n, n)).astype(np.float32), (0, 0), 0.6)[..., None] * 2.5
    _save("wall_plaster.png", img)


def make_cardboard(rng) -> None:
    n = 512
    flute = 0.5 + 0.5 * np.sin(np.arange(n, dtype=np.float32) * 2 * np.pi / 6.0)
    noise = 0.8 * _fbm(rng, n, 5) + 0.2 * flute[None, :]
    img = _tint(noise, (92, 128, 164), (128, 168, 204))  # BGR kraft brown
    cv2.rectangle(img, (0, n // 2 - 22), (n, n // 2 + 22), (150, 190, 215), -1)  # packing tape
    cv2.putText(img, "FRAGILE", (150, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (60, 80, 110), 3, cv2.LINE_AA)
    _save("cardboard.png", img)


def make_wood(rng) -> None:
    n = 512
    y = np.linspace(0, 1, n, dtype=np.float32)[:, None]
    warp = _fbm(rng, n, 4)
    rings = 0.5 + 0.5 * np.sin((y * 38.0 + warp * 6.0) * 2 * np.pi)
    img = _tint(0.6 * rings + 0.4 * _fbm(rng, n, 5), (70, 118, 160), (120, 172, 212))
    _save("wood.png", img)


def make_metal(rng) -> None:
    n = 512
    streak = cv2.GaussianBlur(rng.random((n, n), dtype=np.float32), (0, 0), sigmaX=24, sigmaY=0.6)
    streak = (streak - streak.min()) / (streak.max() - streak.min())
    _save("metal_brushed.png", _tint(streak, (120, 122, 124), (176, 178, 180)))


def make_ceiling(rng) -> None:
    n = 512
    img = _tint(_fbm(rng, n, 4), (222, 224, 224), (240, 241, 241))
    for k in range(0, n, n // 4):
        cv2.line(img, (k, 0), (k, n), (185, 187, 187), 3)
        cv2.line(img, (0, k), (n, k), (185, 187, 187), 3)
    _save("ceiling_tile.png", img)


def make_helipad() -> None:
    n = 512
    img = np.full((n, n, 3), (52, 54, 56), np.float32)
    cv2.circle(img, (n // 2, n // 2), 225, (40, 205, 240), 14, cv2.LINE_AA)
    cv2.putText(img, "H", (n // 2 - 105, n // 2 + 112), cv2.FONT_HERSHEY_SIMPLEX, 10.0, (235, 235, 235), 30, cv2.LINE_AA)
    _save("helipad.png", img)


def make_hazard() -> None:
    n = 256
    img = np.zeros((n, n, 3), np.float32)
    xx, yy = np.meshgrid(np.arange(n), np.arange(n))
    stripe = ((xx + yy) // 32) % 2 == 0
    img[stripe] = (20, 200, 240)
    img[~stripe] = (28, 28, 30)
    _save("hazard.png", img)


def make_screen(rng) -> None:
    w, h = 512, 320
    img = np.full((h, w, 3), (46, 30, 24), np.float32)
    for row in range(18):
        y = 22 + row * 16
        x = 18 + int(rng.integers(0, 4)) * 16
        length = int(rng.integers(60, 320))
        colour = [(210, 170, 120), (140, 200, 150), (120, 160, 230), (200, 200, 200)][int(rng.integers(0, 4))]
        cv2.line(img, (x, y), (x + length, y), colour, 3)
    _save("screen.png", img)


def make_poster() -> None:
    w, h = 512, 720
    img = np.full((h, w, 3), (244, 244, 240), np.float32)
    cv2.rectangle(img, (0, 0), (w, 120), (40, 40, 200), -1)
    cv2.putText(img, "FLIGHT AREA", (38, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.9, (255, 255, 255), 5, cv2.LINE_AA)
    for i, line in enumerate(["Eye protection required", "Keep clear of the net", "Props off before entry", "Battery in LiPo bag"]):
        cv2.circle(img, (60, 210 + i * 110), 14, (40, 40, 200), -1, cv2.LINE_AA)
        cv2.putText(img, line, (95, 222 + i * 110), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (40, 40, 40), 2, cv2.LINE_AA)
    _save("poster_safety.png", img)


def make_markers(scene: dict) -> None:
    markers = scene.get("markers", {})
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, markers.get("dictionary", "DICT_4X4_50")))
    for item in markers.get("items", []):
        marker_id = int(item["id"])
        code = cv2.aruco.generateImageMarker(dictionary, marker_id, 480)
        # White quiet zone: the detector needs contrast around the black border.
        tile = cv2.copyMakeBorder(code, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
        _save(f"aruco_{marker_id}.png", cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR))


def main() -> None:
    scene = json.loads((TWIN_DIR / "config" / "scene_lab.json").read_text(encoding="utf-8"))
    rng = np.random.default_rng(7)
    make_floor(rng)
    make_wall(rng)
    make_cardboard(rng)
    make_wood(rng)
    make_metal(rng)
    make_ceiling(rng)
    make_helipad()
    make_hazard()
    make_screen(rng)
    make_poster()
    make_markers(scene)
    print(f"Wrote {len(list(TEX_DIR.glob('*.png')))} textures to {TEX_DIR}")


if __name__ == "__main__":
    main()
