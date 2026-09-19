import sys, json, base64, urllib.request, re, io
sys.path.insert(0, r"d:\UG\B.TECH\7th\Project_phase1")

IMAGE_PATH  = r"d:\UG\B.TECH\7th\Project_phase1\pysimverse_frame.jpg"
OUTPUT_PATH = r"d:\UG\B.TECH\7th\Project_phase1\vlm_test_result.jpg"
OLLAMA_URL  = "http://localhost:11434/api/chat"
VLM_MODEL   = "hf.co/ggml-org/Qwen2-VL-2B-Instruct-GGUF:Q4_K_M"

with open(IMAGE_PATH, "rb") as f:
    img_bytes = f.read()
b64 = base64.b64encode(img_bytes).decode()

try:
    from PIL import Image
    img_pil = Image.open(io.BytesIO(img_bytes))
    W, H = img_pil.size
    print(f"Image: {W}x{H} px  ({len(img_bytes)/1024:.1f} KB)")
except ImportError:
    W, H = 452, 340
    img_pil = None
    print(f"PIL not found, assuming {W}x{H}. pip install Pillow to draw boxes.")

print(f"Model : {VLM_MODEL}")
print("="*60)

TESTS = [
    ("cyan blue pillar",      (0, 200, 255)),
    ("black metal shelf",     (30, 30, 30)),
    ("white fluorescent light",(255, 220, 0)),
    ("floor ground surface",  (160, 100, 50)),
]

def ask_vlm(target):
    prompt = (
        f"Locate the {target} in this image.\n"
        "Output ONLY valid JSON:\n"
        "{\"bbox\": [ymin, xmin, ymax, xmax], \"label\": \"short description\"}\n"
        "All values are integers 0-1000 normalized to image dimensions."
    )
    payload = {
        "model": VLM_MODEL,
        "messages": [{"role": "user", "content": prompt, "images": [b64]}],
        "stream": False,
        "options": {"temperature": 0.0, "num_predict": 200}
    }
    body = json.dumps(payload).encode()
    req = urllib.request.Request(OLLAMA_URL, data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=90) as resp:
        data = json.loads(resp.read().decode())
    return data.get("message", {}).get("content", "")

def parse_bbox(text):
    try:
        j = json.loads(text.strip())
        b = j.get("bbox", [])
        if len(b) == 4:
            return tuple(int(v) for v in b), j.get("label", "?")
    except Exception:
        pass
    m = re.search(r"\[(\d+)[,\s]+(\d+)[,\s]+(\d+)[,\s]+(\d+)\]", text)
    if m:
        return tuple(map(int, m.groups())), "?"
    return None, None

results = []
for target, color in TESTS:
    print(f"\nTarget: \"{target}\"")
    try:
        raw = ask_vlm(target)
        print(f"  Raw    : {raw[:300]}")
        bbox, label = parse_bbox(raw)
        if bbox:
            ymin, xmin, ymax, xmax = bbox
            px = (int(xmin*W/1000), int(ymin*H/1000), int(xmax*W/1000), int(ymax*H/1000))
            cx, cy = (px[0]+px[2])//2, (px[1]+px[3])//2
            print(f"  NormBBox: ymin={ymin} xmin={xmin} ymax={ymax} xmax={xmax}")
            print(f"  Pixels  : ({px[0]},{px[1]}) to ({px[2]},{px[3]})  center=({cx},{cy})")
            print(f"  Size    : {px[2]-px[0]}w x {px[3]-px[1]}h px")
            print(f"  Label   : {label}")
            results.append((target, px, color, label))
        else:
            print("  FAIL: could not parse bbox")
    except Exception as e:
        print(f"  ERROR: {e}")

print("\n" + "="*60)
if img_pil and results:
    try:
        from PIL import Image, ImageDraw
        img_draw = img_pil.convert("RGB")
        draw = ImageDraw.Draw(img_draw)
        for target, (x1,y1,x2,y2), color, label in results:
            draw.rectangle([x1,y1,x2,y2], outline=color, width=3)
            txt = target[:22]
            tw = len(txt)*6 + 4
            draw.rectangle([x1, max(0,y1-16), x1+tw, y1], fill=color)
            fc = (255,255,255) if sum(color) < 400 else (0,0,0)
            draw.text((x1+2, max(0,y1-14)), txt, fill=fc)
        img_draw.save(OUTPUT_PATH, quality=95)
        print(f"Annotated image saved: {OUTPUT_PATH}")
    except Exception as e:
        print(f"Draw error: {e}")
else:
    if not img_pil:
        print("Install Pillow to draw bboxes: pip install Pillow")

print(f"\nResult: {len(results)}/{len(TESTS)} targets detected.")
