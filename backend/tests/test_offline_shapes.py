"""Offline parser shape plans (used when Gemini is unavailable)."""
import unittest
from backend.models.gemini_provider import GeminiProvider, _extract_json, coerce_skills
from backend.schemas.capabilities import RobotCapabilities


class TestOfflineShapePlans(unittest.TestCase):
    def setUp(self):
        self.llm = GeminiProvider(api_key="")
        self.caps = RobotCapabilities()

    def test_rhombus_has_takeoff_and_closed_path(self):
        plan = self.llm.plan("take off to 1m and fly in a rhombus", self.caps)
        skills = [s.skill for s in plan.skills]
        self.assertEqual(skills[0], "TAKEOFF")
        self.assertGreaterEqual(skills.count("MOVE_TO"), 5)
        self.assertEqual(plan.skills[1].params["x"], 0.6)
        self.assertEqual(plan.skills[-1].params["x"], plan.skills[1].params["x"])
        self.assertEqual(plan.skills[-1].params["y"], plan.skills[1].params["y"])

    def test_rectangle_has_closed_path(self):
        plan = self.llm.plan("take off to 0.5m and fly in a rectangle", self.caps)
        moves = [s for s in plan.skills if s.skill == "MOVE_TO"]
        self.assertGreaterEqual(len(moves), 5)
        self.assertEqual(moves[0].params["z"], 0.5)
        self.assertNotEqual(abs(moves[0].params["x"]), abs(moves[0].params["y"]))

    def test_extract_json_strips_fences(self):
        data = _extract_json('```json\n{"reasoning": "ok", "skills": []}\n```')
        self.assertEqual(data["reasoning"], "ok")

    def test_coerce_flat_xy_into_params(self):
        skills = coerce_skills([
            {"skill": "TAKEOFF", "z": 1.0},
            {"skill": "MOVE_TO", "x": 0.7, "y": 0.35, "z": 1.0},
        ])
        self.assertEqual(skills[0].params["z"], 1.0)
        self.assertEqual(skills[1].params["x"], 0.7)

    def test_coerce_dedupes_identical_takeoff(self):
        skills = coerce_skills([
            {"skill": "TAKEOFF", "params": {}},
            {"skill": "TAKEOFF", "params": {}},
        ])
        self.assertEqual(len(skills), 1)

    def test_triangle_is_closed(self):
        plan = self.llm.plan("take off to 1m and fly in a triangle", self.caps)
        moves = [s for s in plan.skills if s.skill == "MOVE_TO"]
        self.assertEqual(len(moves), 4)
        self.assertEqual(moves[0].params["x"], moves[-1].params["x"])
        self.assertEqual(moves[0].params["y"], moves[-1].params["y"])

    def test_unknown_shape_still_flies_a_path(self):
        plan = self.llm.plan("take off to 1m and fly in a blob", self.caps)
        moves = [s for s in plan.skills if s.skill == "MOVE_TO"]
        self.assertGreaterEqual(len(moves), 4)


if __name__ == "__main__":
    unittest.main()
