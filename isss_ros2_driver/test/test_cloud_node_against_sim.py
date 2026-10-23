"""Run the driver and processing nodes against the simulated sonar. Needs a sourced ROS 2 workspace:

    colcon build --packages-up-to isss_ros2_driver && colcon test --packages-select isss_ros2_driver
"""

import time

import pytest

rclpy = pytest.importorskip('rclpy')

import numpy as np  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402

from isss.sim import SimulatedSonar  # noqa: E402
from isss_proc.cloud import POINT_DTYPE, POINT_FIELDS, POINT_STEP  # noqa: E402
from isss_ros2_driver.cloud_node import IsssCloudNode  # noqa: E402
from isss_ros2_driver.node import IsssDriverNode  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_publishes_filtered_clouds(ros):
    with SimulatedSonar(rate_hz=20) as sim:
        driver = IsssDriverNode(parameter_overrides=[
            Parameter('ip', value='127.0.0.1'),
            Parameter('port', value=sim.port),
            Parameter('reconnect_interval_sec', value=0.2),
        ])
        processor = IsssCloudNode(parameter_overrides=[Parameter('block', value=2)])
        listener = rclpy.create_node('isss_cloud_test_listener')
        clouds = []
        listener.create_subscription(PointCloud2, 'isss/cloud', clouds.append, 10)
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and len(clouds) < 4:
                for node in (driver, processor, listener):
                    rclpy.spin_once(node, timeout_sec=0.02)
        finally:
            for node in (driver, processor, listener):
                node.destroy_node()

    assert len(clouds) >= 4  # one per ping
    cloud = clouds[-1]
    assert cloud.header.frame_id == 'isss_link'
    assert [(f.name, f.offset, f.datatype) for f in cloud.fields] == POINT_FIELDS
    assert cloud.point_step == POINT_STEP and cloud.height == 1 and cloud.width > 0
    points = np.frombuffer(bytes(cloud.data), dtype=POINT_DTYPE)
    assert points.size == cloud.width
    assert (points['x'] == 0).all() and (points['z'] < 0).all()
    assert set(np.unique(points['side'])) <= {0, 1}
