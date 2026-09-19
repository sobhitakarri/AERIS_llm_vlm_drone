"""LiteWing optical-flow odometry (Circuit-Digest dead-reckoning math)."""
from backend.drone.litewing_odometry import (
    FlowOdometry,
    clamp_hover_z,
    flow_to_velocity,
    hover_xy,
)


def test_flow_to_velocity_scale():
    # delta * 3.7 * 0.01
    assert abs(flow_to_velocity(10) - 0.37) < 1e-9


def test_integrate_moves_forward():
    odom = FlowOdometry()
    odom.enabled = True
    odom.ingest_flow(20, 0, 0.02)
    assert odom.x != 0.0


def test_disabled_does_not_integrate():
    odom = FlowOdometry()
    odom.enabled = False
    odom.ingest_flow(20, 0, 0.02)
    assert odom.x == 0.0


def test_hover_xy_axis_swap():
    # Positive X error should add to sent vy (official swap)
    vx, vy = hover_xy(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, trim_vx=0.0, trim_vy=0.0, kv=0.0)
    assert vy > 0
    assert abs(vx) < 1e-9


def test_clamp_hover_z():
    assert clamp_hover_z(1.5) == 0.8
    assert clamp_hover_z(0.05) == 0.2
    assert abs(clamp_hover_z(0.4) - 0.4) < 1e-9
