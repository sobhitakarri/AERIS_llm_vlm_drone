"""
Unit Tests for Mission Planner Finite State Machine.
"""
import unittest
from backend.planning.planner import MissionPlanner, PlannerState
from backend.schemas.mission import MissionPlan, SkillPrimitive


class TestMissionPlanner(unittest.TestCase):
    def test_planner_fsm_lifecycle(self):
        planner = MissionPlanner()
        self.assertEqual(planner.state, PlannerState.IDLE)

        planner.start_mission("Test Takeoff")
        self.assertEqual(planner.state, PlannerState.PLANNING)

        plan = MissionPlan(
            raw_command="Test Takeoff",
            reasoning="Test",
            skills=[
                SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
                SkillPrimitive(skill="LAND", params={})
            ]
        )
        val = planner.process_generated_plan(plan)
        self.assertTrue(val.is_valid)
        self.assertEqual(planner.state, PlannerState.EXECUTING)

        planner.finish_mission()
        self.assertEqual(planner.state, PlannerState.DONE)


if __name__ == "__main__":
    unittest.main()
