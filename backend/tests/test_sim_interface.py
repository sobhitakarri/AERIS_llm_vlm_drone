"""
Unit Tests for Desktop SimInterface.
"""
import unittest
from backend.drone.sim_interface import SimInterface
from backend.schemas.telemetry import FlightMode


class TestSimInterface(unittest.TestCase):
    def test_sim_takeoff_land_cycle(self):
        sim = SimInterface()
        sim.connect()
        self.assertTrue(sim.is_connected)

        sim.takeoff(altitude=1.0)
        state = sim.get_state()
        self.assertEqual(state.z, 1.0)
        self.assertTrue(state.is_armed)

        sim.move_to(0.5, 0.5, 1.0)
        state = sim.get_state()
        self.assertEqual(state.x, 0.5)
        self.assertEqual(state.y, 0.5)

        sim.land()
        state = sim.get_state()
        self.assertEqual(state.z, 0.0)
        self.assertFalse(state.is_armed)
        self.assertEqual(state.flight_mode, FlightMode.IDLE)


if __name__ == "__main__":
    unittest.main()
