"""Run the real node against the simulated sonar. Needs a sourced ROS 2 workspace:

    colcon build --packages-up-to isss_ros2_driver && colcon test --packages-select isss_ros2_driver
"""

import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402

from isss.sim import SimulatedSonar  # noqa: E402
from isss_ros2_driver.node import IsssDriverNode  # noqa: E402
from isss_msgs.msg import ISSS, ISSSStatus  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_publishes_pings_and_status(ros):
    with SimulatedSonar(rate_hz=20) as sim:
        node = IsssDriverNode(parameter_overrides=[
            Parameter('ip', value='127.0.0.1'),
            Parameter('port', value=sim.port),
            Parameter('reconnect_interval_sec', value=0.2),
        ])
        listener = rclpy.create_node('isss_test_listener')
        pings, statuses = [], []
        listener.create_subscription(ISSS, 'isss/raw', pings.append, 10)
        listener.create_subscription(ISSSStatus, 'isss/status', statuses.append, 10)
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and (len(pings) < 3 or not statuses):
                rclpy.spin_once(node, timeout_sec=0.05)
                rclpy.spin_once(listener, timeout_sec=0.05)
        finally:
            node.destroy_node()
            listener.destroy_node()

    assert len(pings) >= 3
    assert len(pings[0].port_intensity) == 1000
    assert len(pings[0].starboard_depression) == 1000
    assert pings[0].header.frame_id == 'isss_link'
    assert statuses and statuses[-1].connected
