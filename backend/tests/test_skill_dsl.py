"""
Unit Tests for Skill DSL & Primitive Schema Definitions.
"""
import unittest
from backend.planning.skill_dsl import SkillDSL


class TestSkillDSL(unittest.TestCase):
    def test_allowed_skills_creation(self):
        skill = SkillDSL.create_skill("TAKEOFF", z=1.2)
        self.assertEqual(skill.skill, "TAKEOFF")
        self.assertEqual(skill.params["z"], 1.2)

    def test_invalid_skill_rejection(self):
        with self.assertRaises(ValueError):
            SkillDSL.create_skill("INVALID_SKILL_NAME")

    def test_json_schema_generation(self):
        schema = SkillDSL.get_json_schema()
        self.assertIn("properties", schema)
        self.assertIn("skills", schema["properties"])


if __name__ == "__main__":
    unittest.main()
