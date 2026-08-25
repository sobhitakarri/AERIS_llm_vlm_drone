"""
Unit Tests for AI Pre-Execution Safety Validator.
"""
import unittest
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.planning.safety_validator import SafetyValidator


class TestSafetyValidator(unittest.TestCase):
    def test_valid_plan_acceptance(self):
        validator = SafetyValidator()
        plan = MissionPlan(
            raw_command="Fly in square",
            reasoning="Test",
            skills=[
                SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
                SkillPrimitive(skill="MOVE_TO", params={"x": 0.5, "y": 0.5, "z": 1.0}),
                SkillPrimitive(skill="LAND", params={})
            ]
        )
        res = validator.validate_plan(plan)
        self.assertTrue(res.is_valid)
        self.assertEqual(len(res.errors), 0)

    def test_geofence_out_of_bounds_rejection(self):
        validator = SafetyValidator()
        plan = MissionPlan(
            raw_command="Fly far away",
            reasoning="Test",
            skills=[
                SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
                SkillPrimitive(skill="MOVE_TO", params={"x": 10.0, "y": 10.0, "z": 5.0}),
                SkillPrimitive(skill="LAND", params={})
            ]
        )
        res = validator.validate_plan(plan)
        self.assertFalse(res.is_valid)
        self.assertGreaterThan(len(res.errors), 0) if hasattr(self, 'assertGreaterThan') else self.assertTrue(len(res.errors) > 0)


if __name__ == "__main__":
    unittest.main()
