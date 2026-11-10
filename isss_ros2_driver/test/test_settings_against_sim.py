"""The latched settings topics of the driver and cloud nodes, against the simulated sonar. Needs ROS 2:

    colcon build --packages-up-to isss_ros2_driver && colcon test --packages-select isss_ros2_driver
"""

import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from isss.sim import SimulatedSonar  # noqa: E402
from isss_msgs.srv import ConfigureSonar  # noqa: E402
from isss_ros2_driver.cloud_node import IsssCloudNode  # noqa: E402
from isss_ros2_driver.messages import SETTINGS_QOS  # noqa: E402
from isss_ros2_driver.node import IsssDriverNode  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin_until(nodes, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.02)
    return predicate()


def test_driver_settings_are_latched_and_follow_changes(ros):
    with SimulatedSonar(rate_hz=50) as sim:
        driver = IsssDriverNode(parameter_overrides=[
            Parameter('ip', value='127.0.0.1'),
            Parameter('port', value=sim.port),
            Parameter('sound_velocity', value=1450.0),
            Parameter('boresight_depression_deg', value=40.0),
        ])
        listener = rclpy.create_node('settings_listener')
        try:
            # Wait for the connect: the register writes join the settings then
            assert _spin_until([driver], lambda: driver._driver.register_writes)
            _spin_until([driver], lambda: False, timeout=1.2)  # one status tick publishes the change
            got = []
            # Subscribed only now: transient-local delivers the last message anyway, as to ros2 bag record
            listener.create_subscription(String, 'isss/settings', lambda m: got.append(json.loads(m.data)), SETTINGS_QOS)
            assert _spin_until([driver, listener], lambda: got)
            s = got[-1]
            assert s['format'] == 'isss-ros-settings' and s['node'] == '/isss_ros2_driver_node'
            assert s['parameters']['sound_velocity'] == 1450.0 and s['calibration']['boresight_depression_deg'] == 40.0
            assert s['sonar'] == {'range_m': 20.0, 'num_samples': 1000, 'num_phases': 2, 'decimation': 111}
            assert {'register': 18, 'value': 2} in s['register_writes']
            assert s['time']['use_device_time'] is False and s['software']['isss_core']

            client = listener.create_client(ConfigureSonar, 'isss/configure_sonar')
            future = client.call_async(ConfigureSonar.Request(range=30.0, num_samples=1000, num_phases=2))
            assert _spin_until([driver, listener], lambda: future.done() and got[-1]['sonar']['range_m'] == 30.0)
            assert future.result().success
        finally:
            for node in (driver, listener):
                node.destroy_node()


def test_cloud_settings_name_the_filter_chain(ros):
    cloud = IsssCloudNode(parameter_overrides=[Parameter('min_excess', value=20.0)])
    listener = rclpy.create_node('cloud_settings_listener')
    got = []
    listener.create_subscription(String, 'isss/cloud_settings', lambda m: got.append(json.loads(m.data)), SETTINGS_QOS)
    try:
        assert _spin_until([cloud, listener], lambda: got)
    finally:
        for node in (cloud, listener):
            node.destroy_node()
    assert got[0]['parameters']['min_excess'] == 20.0 and got[0]['stages']
    assert got[0]['node'] == '/isss_cloud_node'
