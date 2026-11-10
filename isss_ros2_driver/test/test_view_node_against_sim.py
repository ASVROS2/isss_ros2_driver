"""The viewer's topic source against the driver node and the simulated sonar. Needs a sourced ROS 2 workspace:

    colcon build --packages-up-to isss_ros2_driver && colcon test --packages-select isss_ros2_driver
"""

import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402

from isss.sim import SimulatedSonar  # noqa: E402
from isss_ros2_driver.node import IsssDriverNode  # noqa: E402
from isss_ros2_driver.view_node import TopicSource, parse_args  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_topic_source_buffers_pings_and_takes_the_config_from_them(ros):
    with SimulatedSonar(rate_hz=50) as sim:
        driver = IsssDriverNode(parameter_overrides=[
            Parameter('ip', value='127.0.0.1'),
            Parameter('port', value=sim.port),
            Parameter('reconnect_interval_sec', value=0.2),
            Parameter('counts_per_unit_sine', value=14000.0),
        ])
        viewer = rclpy.create_node('isss_view_test')
        source = TopicSource(viewer, 'isss/raw', keep=5)
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and len(source) < 8:
                for node in (driver, viewer):
                    rclpy.spin_once(node, timeout_sec=0.02)
            source.start()  # a message is in, so it returns at once
            arrays = source.arrays()
        finally:
            for node in (driver, viewer):
                node.destroy_node()

    assert len(source) == 5  # keep
    assert arrays.n_pings == 5 and arrays.num_samples == source.config.num_samples
    assert source.config.counts_per_unit_sine == 14000.0  # from the message, not a default
    assert not source.finished
    rclpy.shutdown()
    assert source.finished  # the window closes when ROS shuts down


def test_filter_options_are_those_of_isss_proc_view():
    opts = parse_args(['--filters', '--min-excess', '20', '--mpaa', '--topic', '/isss_stick/isss/raw'])
    assert opts.filters and opts.min_excess == 20.0 and opts.mpaa and opts.topic == '/isss_stick/isss/raw'
    assert not parse_args([]).filters
