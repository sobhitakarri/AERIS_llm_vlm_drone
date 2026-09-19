"""
Live Webcam VLM Bounding Box Test
- Captures a frame from webcam
- Sends to Qwen2-VL-2B for bounding box detection
- Draws colored bboxes and shows live window
Press SPACE to capture + detect, Q to quit.
"""
import cv2, base64, json, re, urllib.request, sys, time

OLLAMA_URL = "http://localhost:11434/api/chat"
VLM_MODEL  = "hf.co/ggml-org/Qwen2-VL-2B-Instruct-GGUF:Q4_K_M"

# ── Targets to detect ─────────────────────────────────────────────────────────
TARGETS = [
    ("bottle",        (0,   200, 255)),  # cyan
    ("person",        (0,   255, 0  )),  # green
    ("hand",          (255, 150, 0  )),  # orange
    ("object on desk",(255, 0,   150)),  # pink
]

def ask_vlm(jpg_bytes: bytes, target: str) -> str:
    b64 = base64.b64encode(jpg_bytes).decode()
    prompt = (
        f"Where is the {target} in this image? "
        "Reply with ONLY a bounding box array on one line: "
        "[ymin, xmin, ymax, xmax] "
        "All integers 0-1000 (0=top/left, 1000=bottom/right). "
        "No text, no explanation. Just the array."
    )
    payload = {
        "model": VLM_MODEL,
        "messages": [{"role": "user", "content": prompt, "images": [b64]}],
        "stream": False,
        "options": {"temperature": 0.0, "num_predict": 80}
    }
    body = json.dumps(payload).encode()
    req = urllib.request.Request(OLLAMA_URL, data=body,
                                  headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return json.loads(resp.read().decode()).get("message", {}).get("content", "")

def parse_bbox(text: str):
    text = re.sub(r"```[a-z]*\n?", "", text).strip()
    try:
        j = json.loads(text)
        b = j.get("bbox") or j.get("bounding_box") or j.get("box")
        if b and len(b) == 4:
            return tuple(int(v) for v in b)
    except Exception:
        pass
    m = re.search(r"\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]", text)
    if m:
        return tuple(map(int, m.groups()))
    nums = re.findall(r"\d+", text)
    if len(nums) >= 4:
        vals = [int(n) for n in nums[:4]]
        if all(0 <= v <= 1000 for v in vals):
            return tuple(vals)
    return None

def draw_bbox(frame, bbox_norm, label, color, H, W):
    ymin, xmin, ymax, xmax = bbox_norm
    x1 = int(xmin * W / 1000); y1 = int(ymin * H / 1000)
    x2 = int(xmax * W / 1000); y2 = int(ymax * H / 1000)
    cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
    txt = label[:24]
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.rectangle(frame, (x1, y1-th-6), (x1+tw+4, y1), color, -1)
    fc = (0,0,0) if sum(color) > 400 else (255,255,255)
    cv2.putText(frame, txt, (x1+2, y1-4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, fc, 1)
    cx, cy = (x1+x2)//2, (y1+y2)//2
    cv2.drawMarker(frame, (cx,cy), color, cv2.MARKER_CROSS, 12, 2)
    print(f"  [{label}] norm({ymin},{xmin},{ymax},{xmax}) -> px({x1},{y1})-({x2},{y2}) center=({cx},{cy})")
    return frame

# ── Open webcam ───────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("ERROR: Could not open webcam (index 0). Try index 1.")
    sys.exit(1)

W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Webcam opened: {W}x{H}")
print("Press SPACE to capture + run VLM detection")
print("Press Q to quit")
print(f"Targets: {[t for t,c in TARGETS]}")

last_annotated = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed"); break

    display = frame.copy()
    if last_annotated is not None:
        display = last_annotated.copy()

    # Overlay instructions
    cv2.putText(display, "SPACE=detect  Q=quit", (10, H-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
    cv2.putText(display, f"Model: {VLM_MODEL[:40]}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100,255,100), 1)

    cv2.imshow("Webcam VLM Test - Qwen2-VL-2B", display)
    key = cv2.waitKey(30) & 0xFF

    if key == ord("q"):
        break
    elif key == ord(" "):
        print("\n--- Capturing frame and running VLM detection ---")
        ret2, snap = cap.read()
        if not ret2:
            continue
        _, jpg = cv2.imencode(".jpg", snap, [cv2.IMWRITE_JPEG_QUALITY, 85])
        jpg_bytes = jpg.tobytes()
        annotated = snap.copy()
        detected = 0
        for target, color in TARGETS:
            print(f"Querying: '{target}'...")
            t0 = time.time()
            try:
                raw = ask_vlm(jpg_bytes, target)
                elapsed = time.time() - t0
                print(f"  Raw ({elapsed:.1f}s): {raw[:150]}")
                bbox = parse_bbox(raw)
                if bbox:
                    annotated = draw_bbox(annotated, bbox, target, color, H, W)
                    detected += 1
                else:
                    print(f"  Could not parse bbox")
            except Exception as e:
                print(f"  ERROR: {e}")
        print(f"Detected {detected}/{len(TARGETS)} targets")
        cv2.imwrite("webcam_vlm_result.jpg", annotated)
        print("Saved: webcam_vlm_result.jpg")
        last_annotated = annotated

cap.release()
cv2.destroyAllWindows()
