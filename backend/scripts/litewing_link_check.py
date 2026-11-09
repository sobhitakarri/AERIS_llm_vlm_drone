"""
Check the radio link to a LiteWing with no flight code: UDP reachability, the
cflib handshake, then a few seconds of the sensors. Props off, drone on the bench.

    py -3.13 backend/scripts/litewing_link_check.py [udp://192.168.43.42:2390]
"""
import socket
import sys
import time

URI = sys.argv[1] if len(sys.argv) > 1 else "udp://192.168.43.42:2390"
if URI.startswith("udp://") and not URI.rsplit(":", 1)[-1].isdigit():
    URI += ":2390"
host, port = URI[len("udp://"):].rsplit(":", 1)
port = int(port)

print(f"1) Target {host}:{port}")
# No raw probe: the firmware answers whoever sent the last valid packet, so a probe from a
# throw-away socket steals the reply address from the real connection.
probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
probe.connect((host, port))
mine = probe.getsockname()[0]
probe.close()
if mine == host:
    print(f"   {host} is THIS PC's address. The LiteWing is normally 192.168.43.42 (the PC gets .43).")
    sys.exit(1)
print(f"   this PC is {mine}")

print("2) cflib handshake (15 s limit)")
import cflib.crtp                                       # noqa: E402
from cflib.crazyflie import Crazyflie                   # noqa: E402
from cflib.crazyflie.log import LogConfig               # noqa: E402
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  # noqa: E402
import threading                                        # noqa: E402

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))
from backend.drone import litewing_udp                  # noqa: E402

cflib.crtp.init_drivers()
litewing_udp.register()      # the firmware needs a checksum byte that cflib 0.1.32 no longer sends
cf = Crazyflie(rw_cache="./cache")
scf = SyncCrazyflie(litewing_udp.to_driver_uri(URI), cf=cf)
done = threading.Event()
threading.Thread(target=lambda: (scf.open_link(), done.set()), daemon=True).start()
if not done.wait(15):
    print("   FAILED: no handshake. Is another app connected to the drone? Is the firewall blocking Python?")
    sys.exit(1)
print("   connected")
toc = cf.log.toc.toc
print("   log groups:", sorted(toc)[:12])
want = ["motion.deltaX", "motion.deltaY", "stateEstimate.z", "range.zrange", "pm.vbat",
        "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"]
missing = [v for v in want if v.split(".")[0] not in toc or v.split(".")[1] not in toc[v.split(".")[0]]]
print("   missing variables:", missing or "none")

print("3) sensors for 4 s (move the drone by hand)")
cfg = LogConfig("check", 100)
for v, t in [("motion.deltaX", "int16_t"), ("motion.deltaY", "int16_t"), ("stateEstimate.z", "float"),
             ("range.zrange", "uint16_t")]:
    if v not in missing:
        cfg.add_variable(v, t)
cf.log.add_config(cfg)
cfg.data_received_cb.add_callback(lambda ts, d, lg: print("  ", {k: round(float(x), 3) for k, x in d.items()}))
cfg.start()
time.sleep(4)
cfg.stop()
scf.close_link()
print("done")
