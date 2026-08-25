"""
Unit Tests for MatlabInterface UDP Socket Bridge.
"""
import unittest
from backend.drone.matlab_interface import MatlabInterface
from backend.schemas.telemetry import FlightMode


class TestMatlabInterface(unittest.TestCase):
    def test_matlab_interface_lifecycle(self):
        matlab = MatlabInterface(send_port=5005, recv_port=5006)
        res = matlab.connect()
        self.assertTrue(res)
        self.assertTrue(matlab.is_connected)

        # Test takeoff setpoint dispatch
        matlab.takeoff(altitude=1.0)
        state = matlab.get_state()
        self.assertEqual(state.z, 1.0)
        self.assertTrue(state.is_armed)

        # Test move_to setpoint dispatch
        matlab.move_to(x=0.5, y=-0.5, z=1.0)
        state = matlab.get_state()
        self.assertEqual(state.x, 0.5)
        self.assertEqual(state.y, -0.5)

        # Test landing
        matlab.land()
        state = matlab.get_state()
        self.assertEqual(state.z, 0.0)
        self.assertFalse(state.is_armed)

        matlab.disconnect()
        self.assertFalse(matlab.is_connected)


if __name__ == "__main__":
    unittest.main()
