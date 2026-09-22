"""Shared bbox coordinate handling for VLM grounding output.

VLMs disagree on both the ordering and the scale of bounding boxes:

- Qwen2-VL     absolute pixels, (x1, y1, x2, y2)
- Qwen2.5-VL   absolute pixels, "bbox_2d"
- Qwen-VL v1   0-1000 normalised
- Gemini       0-1000 normalised, (ymin, xmin, ymax, xmax)

Rather than trust any one convention, infer the scale from the magnitudes and
take the ordering from the caller, which knows which model it queried.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

Box = Tuple[int, int, int, int]


def to_pixels(
    vals: Sequence[float],
    img_w: int,
    img_h: int,
    order: str = "xyxy",
    assume: str = "infer",
) -> Optional[Box]:
    """Converts four raw numbers to absolute pixel (u_min, v_min, u_max, v_max).

    `assume` picks how the scale is decided:
      "infer"     guess from magnitudes. Note this is genuinely ambiguous on a
                  640x480 frame, where any value under 640 is valid both as a
                  pixel and as a 0-1000 normalised coordinate.
      "norm1000"  treat as 0-1000 normalised unless a value exceeds 1000, which
                  only pixels can. Use this when the model's convention is known
                  (Gemini) rather than leaving it to the ambiguous guess.

    Returns None when the values cannot describe a usable box under any known
    convention, which is safer than clamping nonsense into the frame.
    """
    if vals is None or len(vals) != 4:
        return None
    try:
        a, b, c, d = (float(v) for v in vals)
    except (TypeError, ValueError):
        return None

    if order == "yxyx":
        y1, x1, y2, x2 = a, b, c, d
    else:
        x1, y1, x2, y2 = a, b, c, d

    peak = max(abs(x1), abs(y1), abs(x2), abs(y2))
    if peak <= 1.0:
        scale_x, scale_y = float(img_w), float(img_h)          # 0-1 normalised
    elif assume == "norm1000" and peak <= 1000.0:
        scale_x, scale_y = img_w / 1000.0, img_h / 1000.0
    elif x1 > img_w or x2 > img_w or y1 > img_h or y2 > img_h:
        if peak <= 1000.0:
            scale_x, scale_y = img_w / 1000.0, img_h / 1000.0  # 0-1000 normalised
        else:
            return None                                       # no known convention
    else:
        scale_x = scale_y = 1.0                               # already pixels

    x1, x2 = x1 * scale_x, x2 * scale_x
    y1, y2 = y1 * scale_y, y2 * scale_y
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1

    u_min = max(0, min(int(round(x1)), img_w - 1))
    u_max = max(0, min(int(round(x2)), img_w))
    v_min = max(0, min(int(round(y1)), img_h - 1))
    v_max = max(0, min(int(round(y2)), img_h))
    if u_max <= u_min or v_max <= v_min:
        return None
    return u_min, v_min, u_max, v_max


def select_qwen_box(
    vals: Sequence[float], img_w: int, img_h: int
) -> Optional[Box]:
    """Qwen2-VL GGUF (Ollama) was trained on 0–1000 boxes.

    `infer` only switches to 0–1000 when a coordinate overflows the image.
    On a tall still (e.g. 800×1200) a 0–1000 box such as [336, 101, 661, 941]
    fits as pixels and lands on the wrong half of the object. Prefer the
    0–1000 reading unless it is implausible and the pixel reading is not.
    """
    as_px = to_pixels(vals, img_w, img_h, order="xyxy", assume="infer")
    as_n1 = to_pixels(vals, img_w, img_h, order="xyxy", assume="norm1000")
    if as_n1 is None:
        return as_px
    if as_px is None or as_px == as_n1:
        return as_n1
    p_n = plausibility(*as_n1, img_w, img_h)
    p_p = plausibility(*as_px, img_w, img_h)
    if p_n <= 0.0 and p_p > 0.0:
        return as_px
    return as_n1


def plausibility(
    u_min: int, v_min: int, u_max: int, v_max: int, img_w: int, img_h: int
) -> float:
    """Scores how believable a box is, penalising the known VLM grounding
    failure shapes. Returns 0.0 for boxes that should be discarded.

    This is deliberately not a detector confidence. Scoring by area (as this
    once did) rewards the whole-image box, which is the most common failure.
    """
    frame_area = float(max(1, img_w * img_h))
    area_ratio = ((u_max - u_min) * (v_max - v_min)) / frame_area
    if area_ratio > 0.7:
        return 0.0   # "the whole image" is the classic grounding failure
    if area_ratio < 0.0005:
        return 0.0   # degenerate sliver
    w, h = u_max - u_min, v_max - v_min
    aspect = max(w, h) / float(max(1, min(w, h)))
    if aspect > 12.0:
        return 0.0
    score = 0.75
    if area_ratio > 0.45:
        score -= 0.25
    if aspect > 6.0:
        score -= 0.15
    return max(0.1, min(0.9, score))
