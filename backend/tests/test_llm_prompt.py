"""LLM planner prompt helpers."""
import unittest
from backend.planning.llm_prompt import is_takeoff_only_command, plan_is_too_thin, build_planner_prompt
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.mission import SkillPrimitive


class TestLlmPrompt(unittest.TestCase):
    def test_takeoff_only_detection(self):
        self.assertTrue(is_takeoff_only_command("take off to 1m"))
        self.assertFalse(is_takeoff_only_command("take off to 1m and fly in a rectangle"))
        self.assertFalse(is_takeoff_only_command("inspect the red bottle"))
        self.assertFalse(is_takeoff_only_command("hover 4 seconds then land"))

    def test_thin_plan(self):
        takeoff = [SkillPrimitive(skill="TAKEOFF", params={"z": 1.0})]
        self.assertTrue(plan_is_too_thin("inspect the bottle", takeoff))
        self.assertFalse(plan_is_too_thin("take off to 1m", takeoff))

    def test_prompt_is_general_not_shape_only(self):
        text = build_planner_prompt("go left 0.3m then land", RobotCapabilities())
        self.assertIn("ANYTHING", text)
        self.assertIn("go left 0.3m then land", text)
        self.assertIn("FIND", text)
        self.assertIn("LAND", text)


if __name__ == "__main__":
    unittest.main()
