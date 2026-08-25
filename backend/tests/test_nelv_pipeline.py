"""NeLV-style indoor pipeline: route, path densify, control sandwich."""
import unittest

from backend.models.gemini_provider import GeminiProvider
from backend.planning.control_platform import sandwich_takeoff_land
from backend.planning.path_planner import densify_move_tos
from backend.planning.route_planner import match_object
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.mission import SkillPrimitive


class TestNelvIndoorPipeline(unittest.TestCase):
    def setUp(self):
        self.llm = GeminiProvider(api_key="")
        self.caps = RobotCapabilities()
        self.ctx = {
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "objects": [{"label": "red_bottle", "x": 0.5, "y": 0.5, "z": 0.0}],
        }

    def test_find_uses_code_and_object_poi(self):
        intent_ok = self.llm.plan("find the red bottle", self.caps, self.ctx)
        self.assertEqual(intent_ok.source, "code")
        names = [s.skill for s in intent_ok.skills]
        self.assertIn("FIND", names)
        find = next(s for s in intent_ok.skills if s.skill == "FIND")
        self.assertIn("x", find.params)
        self.assertLess(find.params["x"], 0.5)

    def test_pipeline_stages_present(self):
        plan = self.llm.plan("take off", self.caps)
        self.assertEqual(plan.pipeline, ["parser", "route", "path", "control"])

    def test_densify_long_hop(self):
        skills = [
            SkillPrimitive(skill="TAKEOFF", params={"z": 0.5}),
            SkillPrimitive(skill="MOVE_TO", params={"x": 1.4, "y": 1.4, "z": 0.5}),
        ]
        out = densify_move_tos(skills, self.caps.workspace)
        moves = [s for s in out if s.skill == "MOVE_TO"]
        self.assertGreater(len(moves), 1)

    def test_sandwich_adds_takeoff(self):
        skills = [SkillPrimitive(skill="MOVE_TO", params={"x": 0.2, "y": 0.1, "z": 0.5})]
        out = sandwich_takeoff_land(skills, altitude=0.5, takeoff_first=True, land_at_end=True)
        self.assertEqual(out[0].skill, "TAKEOFF")
        self.assertEqual(out[-1].skill, "LAND")

    def test_match_object_prefers_label(self):
        obj = match_object("blue cube", [
            {"label": "red_bottle", "x": 0.5, "y": 0.5},
            {"label": "blue_cube", "x": -0.5, "y": 0.5},
        ])
        self.assertEqual(obj["label"], "blue_cube")


if __name__ == "__main__":
    unittest.main()
