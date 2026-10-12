"""
End-to-end smoke test of the digital twin link.

Start the twin in MATLAB (`run_aeris_twin`), then:
    py -3 backend/scripts/twin_smoke_test.py

Checks telemetry, the camera streams, marker calibration and a short flight,
and reports tracking error against the plant's true state.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from backend.drone.twin_interface import TwinInterface  # noqa: E402
from backend.vision.calibration import calibrate_overhead  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=240.0, help="seconds to wait for the twin")
    ap.add_argument("--stop", action="store_true", help="send STOP to the twin when done")
    ap.add_argument("--out", default=str(REPO_ROOT / "reports" / "twin_results"), help="results folder")
    ap.add_argument("--record", action="store_true", help="record a video of the flight")
    args = ap.parse_args()
    root = Path(args.out)
    out = root / "images"
    out.mkdir(parents=True, exist_ok=True)

    twin = TwinInterface()
    twin.connect()
    if not twin.wait_for_telemetry(args.wait):
        print("FAIL: no telemetry from the twin")
        return 1
    print(f"telemetry ok: pose=({twin.state.x:.2f}, {twin.state.y:.2f}, {twin.state.z:.2f}) "
          f"battery={twin.state.battery_v:.2f} V")

    t0 = time.time()
    frames = {}
    while time.time() - t0 < 30 and len(frames) < len(twin.camera_names()):
        for name in twin.camera_names():
            f = twin.get_camera_frame(name)
            if f is not None:
                frames[name] = f
        time.sleep(0.2)
    for name, f in frames.items():
        cv2.imwrite(str(out / f"camera_{name}.jpg"), f)
    print(f"cameras: {sorted(frames)}")
    depth = twin.get_depth_frame()
    print("depth:", None if depth is None else f"{depth.shape} range {depth.min():.2f}..{depth.max():.2f} m")

    cal = None
    if "overhead" in frames:
        cal = calibrate_overhead(frames["overhead"], twin.markers(), twin.camera_info("overhead"))
    if cal is None:
        print("FAIL: overhead calibration")
    else:
        print(f"calibration: markers {cal.marker_ids}, reprojection {cal.reproj_rms_px:.2f} px, "
              f"camera at {cal.camera_position.round(3).tolist()}")

    def err():
        tr = twin.truth
        return math.dist((twin.state.x, twin.state.y, twin.state.z), (tr.get("x", 0), tr.get("y", 0), tr.get("z", 0)))

    rec = None
    if args.record:
        from backend.scripts.twin_recorder import Recorder, TwinSource
        rec = Recorder(root / "videos" / "case00_link_smoke_flight.mp4", "Case 0  link check", TwinSource(twin),
                       caption="take-off, three waypoints, 90 deg yaw, landing (no language model)").start()
    ok = twin.takeoff(1.0)
    print(f"takeoff: {ok}  z={twin.state.z:.2f}  est-vs-truth {err() * 100:.1f} cm")
    wp = [(-0.8, -0.6, 1.0), (0.4, -0.2, 1.2), (0.0, 0.6, 0.8)]
    for x, y, z in wp:
        t1 = time.time()
        ok = twin.move_to(x, y, z, velocity=0.6) and ok
        tr = twin.truth
        print(f"move_to ({x:+.1f}, {y:+.1f}, {z:.1f}): {time.time() - t1:4.1f} s  "
              f"true=({tr.get('x', 0):+.2f}, {tr.get('y', 0):+.2f}, {tr.get('z', 0):.2f})  "
              f"true error {math.dist((x, y, z), (tr.get('x', 0), tr.get('y', 0), tr.get('z', 0))) * 100:.1f} cm")
    ok = twin.rotate(90) and ok
    print(f"rotate: yaw={twin.state.yaw:.0f} deg (true {twin.truth.get('yaw', 0):.0f})")
    for name in twin.camera_names():
        f = twin.get_camera_frame(name)
        if f is not None:
            cv2.imwrite(str(out / f"camera_{name}_inflight.jpg"), f)
    if rec:
        rec.save_still(out / "case00_link_smoke_flight.jpg")
    ok = twin.land() and ok
    print(f"land: {ok}  z={twin.truth.get('z', 0):.3f}  battery={twin.state.battery_v:.2f} V "
          f"({twin.state.battery_percentage}%)")
    if rec:
        time.sleep(1.0)
        rec.stop()
    if args.stop:
        twin._send("STOP")
    twin.disconnect()
    print("PASS" if ok and cal is not None else "FAIL")
    return 0 if ok and cal is not None else 1


if __name__ == "__main__":
    sys.exit(main())
