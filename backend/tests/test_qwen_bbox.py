"""VLM bbox parsing, coordinate-convention inference, and plausibility gate."""
from backend.models.gemini_provider import GeminiProvider
from backend.models.qwen_provider import QwenProvider
from backend.vision.bbox_convert import to_pixels

P = QwenProvider


def test_gemini_accepts_list_bbox_in_yxyx_order():
    """Gemini returns bbox as an array, not a dict of named edges. Calling .get()
    on it discarded otherwise-valid detections."""
    box = GeminiProvider._bbox_to_pixels([311, 510, 329, 542], 640, 480)
    assert box is not None
    u_min, v_min, u_max, v_max = box
    # ymin=311, xmin=510, ymax=329, xmax=542 normalised over 0-1000
    assert (u_min, u_max) == (326, 347)
    assert (v_min, v_max) == (149, 158)


def test_gemini_accepts_named_edge_dicts():
    assert GeminiProvider._bbox_to_pixels(
        {"u_min": 10, "v_min": 20, "u_max": 110, "v_max": 140}, 640, 480
    ) == (10, 20, 110, 140)
    # ymin/xmin keys are Gemini's own 0-1000 normalised form
    assert GeminiProvider._bbox_to_pixels(
        {"ymin": 0, "xmin": 0, "ymax": 500, "xmax": 500}, 640, 480
    ) == (0, 0, 320, 240)


def test_gemini_rejects_unusable_bbox_shapes():
    assert GeminiProvider._bbox_to_pixels("nonsense", 640, 480) is None
    assert GeminiProvider._bbox_to_pixels([1, 2, 3], 640, 480) is None
    assert GeminiProvider._bbox_to_pixels({"foo": 1}, 640, 480) is None


def test_offline_vlm_reports_not_found_rather_than_inventing_a_target():
    """The old offline path returned a box at frame centre, which grounds to the
    middle of the workspace and the drone flies there."""
    provider = GeminiProvider.__new__(GeminiProvider)
    provider.client = None
    provider.model_name = "stub"
    assert provider.resolve_target(b"", "red bottle", 640, 480) is None


def test_parses_bare_array():
    assert P._parse_bbox("[100, 50, 200, 150]") == (100.0, 50.0, 200.0, 150.0)


def test_parses_qwen2vl_grounding_tokens():
    text = "<|box_start|>(120,80),(260,300)<|box_end|>"
    assert P._parse_bbox(text) == (120.0, 80.0, 260.0, 300.0)


def test_parses_qwen25vl_bbox_2d_list():
    text = '[{"bbox_2d": [10, 20, 30, 40], "label": "bottle"}]'
    assert P._parse_bbox(text) == (10.0, 20.0, 30.0, 40.0)


def test_parses_fenced_json():
    assert P._parse_bbox('```json\n{"bbox": [1, 2, 3, 4]}\n```') == (1.0, 2.0, 3.0, 4.0)


def test_does_not_invent_bbox_from_prose():
    """The old parser took the first four integers anywhere, so a refusal became
    a bounding box."""
    assert P._parse_bbox("I can see 2 objects in this 640 by 480 image.") is None
    assert P._parse_bbox("There is no bottle here.") is None


def test_qwen_prefers_norm1000_when_both_conventions_fit():
    """Tall 800x1200 still: [336,101,661,941] fits as pixels (wrong half of a
    centred bottle) and as 0–1000 (centred). Qwen must pick 0–1000."""
    box = P._to_pixels((336, 101, 661, 941), 800, 1200)
    assert box is not None
    u_min, v_min, u_max, v_max = box
    assert u_min < 280 and u_max > 500
    assert v_min < 200 and v_max > 1000


def test_absolute_pixels_pass_through_via_infer():
    assert to_pixels((100, 50, 200, 150), 640, 480, assume="infer") == (100, 50, 200, 150)


def test_zero_to_one_normalised_is_scaled():
    assert P._to_pixels((0.25, 0.5, 0.75, 1.0), 640, 480) == (160, 240, 480, 480)


def test_zero_to_thousand_normalised_is_detected_by_overflow():
    """Values beyond the image bounds cannot be absolute pixels, so they must be
    the 0-1000 convention."""
    assert P._to_pixels((0, 0, 1000, 1000), 640, 480) == (0, 0, 640, 480)
    # 500 -> 500*640/1000 = 320 in x, 500*480/1000 = 240 in y
    assert P._to_pixels((500, 500, 750, 750), 640, 480) == (320, 240, 480, 360)


def test_inverted_corners_are_swapped():
    assert to_pixels((200, 150, 100, 50), 640, 480, assume="infer") == (100, 50, 200, 150)


def test_degenerate_and_out_of_range_rejected():
    assert P._to_pixels((100, 100, 100, 100), 640, 480) is None
    assert P._to_pixels((5000, 5000, 6000, 6000), 640, 480) is None


def test_full_frame_box_is_rejected_not_rewarded():
    """The old scoring gave a whole-image box ~0.95 confidence, so the classic
    grounding failure outranked every real detection."""
    assert P._plausibility(0, 0, 640, 480, 640, 480) == 0.0


def test_sliver_and_extreme_aspect_rejected():
    assert P._plausibility(10, 10, 12, 12, 640, 480) == 0.0
    assert P._plausibility(0, 100, 640, 105, 640, 480) == 0.0


def test_norm1000_is_asserted_not_guessed():
    """On a 640x480 frame a value like 500 is ambiguous between pixels and 0-1000
    normalised, so a known convention must override inference."""
    ambiguous = (100, 200, 300, 400)
    assert to_pixels(ambiguous, 640, 480, assume="infer") == (100, 200, 300, 400)
    assert to_pixels(ambiguous, 640, 480, assume="norm1000") == (64, 96, 192, 192)


def test_pixel_values_above_1000_still_recognised_as_pixels():
    """A 1920x1080 frame emits coordinates beyond 1000, which cannot be
    normalised values."""
    assert to_pixels((1200, 700, 1500, 900), 1920, 1080, assume="norm1000") == (
        1200, 700, 1500, 900
    )


def test_reasonable_box_scores_higher_than_huge_box():
    normal = P._plausibility(200, 150, 320, 290, 640, 480)
    huge = P._plausibility(20, 20, 600, 420, 640, 480)
    assert normal > huge
    assert 0.0 < normal <= 0.9
