"""
Unit Tests for Spatial Grounding (Homography Matrix Pixel->World Transformation).
"""
import unittest
from backend.vision.spatial_grounding import SpatialGrounding


class TestSpatialGrounding(unittest.TestCase):
    def test_pixel_to_world_conversion(self):
        grounding = SpatialGrounding()
        world_x, world_y, world_z = grounding.pixel_to_world(320, 240, z_altitude=1.0)
        self.assertLess(abs(world_x), 0.1)
        self.assertLess(abs(world_y), 0.1)
        self.assertEqual(world_z, 1.0)


if __name__ == "__main__":
    unittest.main()
