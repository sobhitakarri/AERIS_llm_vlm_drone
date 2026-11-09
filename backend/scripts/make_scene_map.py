"""
Top-down layout map of the twin's scenes, drawn from matlab/twin/config/*.json.

    py -3 backend/scripts/make_scene_map.py

Writes reports/twin_results/images/scene_map.png: scene_lab and scene_distractor
side by side (room, geofence, home pad, calibration markers, cameras, objects with
heights). The same files the twin loads, so the map cannot drift from the scene.
"""
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "matlab" / "twin" / "config"
OUT = REPO / "reports" / "twin_results" / "images" / "scene_map.png"

INK, MUTED, FLOOR = "#1F2937", "#6B7280", "#F8FAFC"
INDIGO, GREEN, ROSE, AMBER = "#4F46E5", "#10B981", "#F43F5E", "#F59E0B"

FIXED_FILL = {            # objects with no colour in the scene file
    "table": "#E9D8B8", "shelf": "#CBD5E1", "partition": "#94A3B8", "chair": "#94A3B8",
    "plant": "#BBF7D0", "laptop": "#CBD5E1", "cone": "#FB923C", "extinguisher": "#F87171",
    "box": "#E7C9A0",     # cardboard
}
ROUND = {"bottle", "drum", "cone", "plant", "extinguisher", "chair"}

# Label placement (dx, dy, horizontal alignment), where the default (below the object) collides.
LABEL = {
    "red_bottle": (-0.65, 0.42, "center"), "blue_cube": (0.05, 0.62, "center"),
    "laptop": (0.1, -0.37, "center"), "green_bottle": (0.2, 0.78, "center"),
    "workbench": (-0.2, -0.48, "center"), "cardboard_box": (-0.05, -0.42, "center"),
    "small_carton": (0.5, 0.2, "left"), "fire_extinguisher": (0.18, 0.0, "left"),
    "office_chair": (0.0, -0.45, "center"), "potted_plant": (0.0, -0.4, "center"),
    "partition_wall": (-0.12, 0.35, "right"), "storage_shelf": (0.0, 0.0, "center"),
    "red_box": (0.0, -0.22, "center"), "yellow_crate": (0.0, -0.3, "center"),
    "steel_drum": (0.0, -0.31, "center"), "traffic_cone": (0.0, -0.26, "center"),
}
PILL = {"green_bottle": (0.34, 0.0)}      # (dx, dy) of the NEW tag from the object
LEADER = {"red_bottle", "blue_cube", "laptop", "green_bottle", "workbench"}


def fill_for(o: dict) -> str:
    if o["type"] == "box" and o.get("material") == "cardboard":
        return FIXED_FILL["box"]
    if "color" in o:
        r, g, b = o["color"]
        return "#%02X%02X%02X" % (round(r * 255), round(g * 255), round(b * 255))
    return FIXED_FILL.get(o["type"], "#CBD5E1")


def draw_object(ax, o: dict, new: bool) -> None:
    x, y, z = o["pos"]
    w, h = o["size_m"][0], o["size_m"][1]
    z_order = 3 + (2 if z > 0 else 0)
    ec = INDIGO if new else "#64748B"
    lw = 2.2 if new else 0.8
    if o["type"] in ROUND:
        ax.add_patch(Circle((x, y), w / 2, fc=fill_for(o), ec=ec, lw=lw, zorder=z_order))
    else:
        rect = Rectangle((x - w / 2, y - h / 2), w, h, fc=fill_for(o), ec=ec, lw=lw, zorder=z_order)
        rect.set_transform(Affine2D().rotate_deg_around(x, y, o.get("yaw_deg", 0)) + ax.transData)
        ax.add_patch(rect)
    if o.get("movable"):
        ax.add_patch(Circle((x, y), max(w, h) / 2 + 0.09, fill=False, ec=ROSE, lw=1.3, ls=(0, (3, 2)), zorder=7))


def label(ax, o: dict, new: bool) -> None:
    x, y, z = o["pos"]
    name = o["name"]
    dx, dy, ha = LABEL.get(name, (0.0, -0.3, "center"))
    height = z + o["size_m"][2]
    text = name.replace("_", " ") + ("\n" + f"top {height:.2f} m")
    tx, ty = x + dx, y + dy
    color = INDIGO if new else INK
    ax.text(tx, ty, text, fontsize=7.2, ha=ha, va="center", color=color, zorder=9,
            fontweight="bold" if new else "normal", linespacing=1.15)
    if name in LEADER:
        ax.plot([x, tx - (0.0 if ha == "center" else (0.04 if ha == "left" else -0.04))], [y, ty + (0.1 if dy < 0 else -0.1)],
                lw=0.6, color="#94A3B8", zorder=8)


def panel(ax, scene: dict, title: str, new_names: set) -> None:
    room = scene["room"]["size_m"]
    ws = scene["workspace"]
    ax.add_patch(Rectangle((-room[0] / 2, -room[1] / 2), room[0], room[1], fc=FLOOR, ec=INK, lw=2.4, zorder=1))
    ax.add_patch(Rectangle((ws["x"][0], ws["y"][0]), ws["x"][1] - ws["x"][0], ws["y"][1] - ws["y"][0],
                           fill=False, ec=GREEN, lw=1.8, ls=(0, (6, 3)), zorder=2))
    ax.text(1.3, ws["y"][0] - 0.2, f"geofence\nx ±{ws['x'][1]:.1f}  y ±{ws['y'][1]:.1f}  z {ws['z'][0]}-{ws['z'][1]} m",
            fontsize=7.5, color=GREEN, va="center", ha="center", fontweight="bold", zorder=9, linespacing=1.2)

    for m in scene["markers"]["items"]:
        s = scene["markers"]["size_m"]
        ax.add_patch(Rectangle((m["pos"][0] - s / 2, m["pos"][1] - s / 2), s, s, fc="#111827", ec="none", zorder=2))
        ax.add_patch(Rectangle((m["pos"][0] - s / 3.2, m["pos"][1] - s / 3.2), s / 1.6, s / 1.6, fc="white", ec="none", zorder=2))
        ax.text(m["pos"][0], m["pos"][1], str(m["id"]), fontsize=6.5, ha="center", va="center", color="#111827", zorder=3)

    hx, hy = scene["home"]["pos"]
    ax.add_patch(Circle((hx, hy), 0.27, fc="white", ec=INDIGO, lw=2, zorder=3))
    ax.text(hx, hy, "H", fontsize=11, fontweight="bold", color=INDIGO, ha="center", va="center", zorder=4)
    ax.text(hx, hy - 0.36, f"home ({hx:.1f}, {hy:.1f})", fontsize=7.2, color=INDIGO, ha="center", va="top", zorder=9)

    for o in scene["objects"]:
        draw_object(ax, o, o["name"] in new_names)
    for o in scene["objects"]:
        label(ax, o, o["name"] in new_names)
    for name in new_names:
        o = next(o for o in scene["objects"] if o["name"] == name)
        pdx, pdy = PILL.get(name, (0.0, 0.3))
        ax.text(o["pos"][0] + pdx, o["pos"][1] + pdy, "NEW", fontsize=6.8, fontweight="bold",
                color="white", ha="center", va="center", zorder=10,
                bbox=dict(boxstyle="round,pad=0.22,rounding_size=0.6", fc=INDIGO, ec="none"))

    # Cameras
    cam = scene["cameras"]
    ox, oy = cam["overhead"]["pos"][0], cam["overhead"]["pos"][1]
    ax.add_patch(Circle((ox, oy), 0.1, fc=INDIGO, ec="white", lw=1.5, zorder=8))
    ax.add_patch(Circle((ox, oy), 0.2, fill=False, ec=INDIGO, lw=1.2, zorder=8))
    ax.text(ox + 0.28, oy - 0.02, f"overhead RGB-D camera\n{cam['overhead']['pos'][2]:.2f} m up, sees the whole room",
            fontsize=7, color=INDIGO, va="center", zorder=9)
    cx, cy = cam["chase"]["pos"][0], cam["chase"]["pos"][1]
    lx, ly = cam["chase"]["look_at"][0], cam["chase"]["look_at"][1]
    ax.plot([cx, lx], [cy, ly], color=INDIGO, lw=0.9, ls=(0, (1, 2)), zorder=2)
    ax.add_patch(Circle((cx, cy), 0.09, fc="white", ec=INDIGO, lw=1.6, zorder=8))
    ax.text(cx + 0.15, cy, "chase camera", fontsize=7, color=INDIGO, ha="left", va="center", zorder=9)

    ax.set_xlim(-room[0] / 2 - 0.25, room[0] / 2 + 0.25)
    ax.set_ylim(-room[1] / 2 - 0.25, room[1] / 2 + 0.25)
    ax.set_aspect("equal")
    ax.set_xticks(range(-3, 4))
    ax.set_yticks(range(-2, 3))
    ax.tick_params(labelsize=7.5, colors=MUTED, length=2)
    ax.grid(color="#E5E7EB", lw=0.5, zorder=0)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_xlabel("x (m): forward", fontsize=8.5, color=MUTED)
    ax.set_ylabel("y (m): left", fontsize=8.5, color=MUTED)
    ax.set_title(title, fontsize=13, fontweight="bold", color=INK, loc="left", pad=10)


def main() -> None:
    lab = json.loads((CONFIG / "scene_lab.json").read_text(encoding="utf-8"))
    dis = json.loads((CONFIG / "scene_distractor.json").read_text(encoding="utf-8"))
    new = {o["name"] for o in dis["objects"]} - {o["name"] for o in lab["objects"]}

    fig, axes = plt.subplots(1, 2, figsize=(17, 8.6), dpi=170)
    fig.patch.set_facecolor("white")
    panel(axes[0], lab, f"scene_lab: {len(lab['objects'])} objects", set())
    panel(axes[1], dis, f"scene_distractor: {len(dis['objects'])} objects, 2 look-alikes added", new)

    room = lab["room"]["size_m"]
    handles = [
        Line2D([], [], color=INK, lw=2.4, label=f"room {room[0]:.0f} x {room[1]:.0f} x {room[2]:.0f} m"),
        Line2D([], [], color=GREEN, lw=1.8, ls=(0, (6, 3)), label="geofence (flight envelope)"),
        Line2D([], [], marker="o", mfc="white", mec=INDIGO, mew=2, color="none", ms=9, label="home pad"),
        Line2D([], [], marker="s", mfc="#111827", mec="#111827", color="none", ms=8, label="ArUco calibration marker (id)"),
        Line2D([], [], marker="o", mfc="none", mec=ROSE, mew=1.3, color="none", ms=10, label="movable object (disturbance tests)"),
        Line2D([], [], marker="o", mfc=INDIGO, mec="white", color="none", ms=9, label="camera"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=6, frameon=False, fontsize=9, labelcolor=INK, bbox_to_anchor=(0.5, 0.0))
    fig.suptitle("AERIS digital twin: indoor flight lab, top-down view", fontsize=17, fontweight="bold", color=INK, x=0.04, ha="left", y=0.975)
    fig.text(0.04, 0.925, "World frame: origin at the room centre on the floor, x forward, y left, z up. Heights are the top of each object above the floor.",
             fontsize=9.5, color=MUTED, ha="left")
    fig.tight_layout(rect=(0.0, 0.07, 1.0, 0.91))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, facecolor="white")
    print("written", OUT)


if __name__ == "__main__":
    main()
