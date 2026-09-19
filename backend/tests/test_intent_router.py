"""Code-first intent router, local planner, and plan cache."""
import os
import tempfile
import unittest

from backend.models.gemini_provider import GeminiProvider
from backend.planning import plan_cache
from backend.planning.intent_router import parse_intent, template_key
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.mission import SkillPrimitive


class TestIntentRouter(unittest.TestCase):
    def test_takeoff_uses_code(self):
        intent = parse_intent("take off")
        self.assertTrue(intent.use_code)
        self.assertEqual(intent.intent, "takeoff")

    def test_square_uses_code(self):
        intent = parse_intent("take off to 1m and fly in a square")
        self.assertTrue(intent.use_code)
        self.assertEqual(intent.intent, "shape")

    def test_goto_uses_code(self):
        intent = parse_intent("go to 0.4, -0.2")
        self.assertTrue(intent.use_code)
        self.assertEqual(intent.intent, "goto")
        self.assertEqual(intent.slots["x"], 0.4)
        self.assertEqual(intent.slots["y"], -0.2)

    def test_unknown_skips_code(self):
        intent = parse_intent("do a barrel roll then write a haiku")
        self.assertFalse(intent.use_code)

    def test_letter_s_then_land_is_not_land_only(self):
        intent = parse_intent("Trace the letter S in the air then come down")
        self.assertFalse(intent.use_code)
        self.assertNotEqual(intent.intent, "land")

    def test_plain_land_still_code(self):
        intent = parse_intent("come down")
        self.assertTrue(intent.use_code)
        self.assertEqual(intent.intent, "land")

    def test_plain_find_is_vision(self):
        intent = parse_intent("find the red bottle")
        self.assertTrue(intent.vision)
        self.assertEqual(intent.intent, "vision")

    def test_conditional_find_skips_code(self):
        intent = parse_intent(
            "Find the red bottle. If it is visible, circle it once and return home. Otherwise hover and wait"
        )
        self.assertFalse(intent.use_code)
        self.assertTrue(intent.vision)

    def test_fly_to_altitude_is_not_goto(self):
        intent = parse_intent("Fly to 5 meters altitude")
        self.assertFalse(intent.use_code)


class TestCodePlanner(unittest.TestCase):
    def setUp(self):
        self.llm = GeminiProvider(api_key="")
        self.caps = RobotCapabilities()

    def test_takeoff_is_code(self):
        plan = self.llm.plan("take off", self.caps)
        self.assertEqual(plan.source, "code")
        self.assertTrue(plan.reasoning.startswith("[CODE]"))
        self.assertEqual(plan.skills[0].skill, "TAKEOFF")

    def test_square_is_code(self):
        plan = self.llm.plan("take off to 1m and fly in a square", self.caps)
        self.assertEqual(plan.source, "code")
        moves = [s for s in plan.skills if s.skill == "MOVE_TO"]
        self.assertEqual(len(moves), 5)

    def test_goto_is_code(self):
        plan = self.llm.plan("go to 0.3, 0.2", self.caps)
        self.assertEqual(plan.source, "code")
        self.assertEqual(plan.skills[-1].params["x"], 0.3)


class TestPlanCache(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
        self._tmp.close()
        os.environ["MISSION_CACHE_PATH"] = self._tmp.name
        plan_cache.save_store({"exact": {}, "templates": {}})
        self.llm = GeminiProvider(api_key="")
        self.caps = RobotCapabilities()

    def tearDown(self):
        os.environ.pop("MISSION_CACHE_PATH", None)
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def test_template_fills_new_numbers(self):
        skills = [
            SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
            SkillPrimitive(skill="HOVER", params={"t": 2.0}),
        ]
        plan_cache.store_entry(
            "hover at 1.0 m for 2 s",
            {"prompt": "test"},
            {"skills": []},
            skills,
        )
        hit = plan_cache.lookup("hover at 0.8 m for 3 s")
        self.assertEqual(hit["hit"], "template")
        filled = hit["skills"]
        self.assertEqual(filled[0]["params"]["z"], 0.8)
        self.assertEqual(filled[1]["params"]["t"], 3.0)

    def test_template_key_masks_numbers(self):
        self.assertEqual(template_key("square 1m"), template_key("square 0.8m"))


if __name__ == "__main__":
    unittest.main()
