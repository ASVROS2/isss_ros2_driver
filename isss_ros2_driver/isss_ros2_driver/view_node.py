"""Live viewer of an ``isss/raw`` topic: ``isss-view``'s window, fed by ROS instead of a TCP connection.

The window, keys and filter overlay are ``isss.viz.run_live`` with
``isss.viz.PingViewer``, the same code as ``isss-view --ip`` and
``isss-proc-view --ip``. Only the source differs: this one subscribes to the
driver's topic, so it runs next to the driver and never talks to the sonar.
The phase model (K, boresight, phase signs) comes from the messages.

    ros2 run isss_ros2_driver isss_ros_view
    ros2 run isss_ros2_driver isss_ros_view --filters --min-excess 20
    ros2 run isss_ros2_driver isss_ros_view --ros-args -r __ns:=/isss_stick
"""

import argparse
import collections
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from isss_msgs.msg import ISSS

from isss_ros2_driver.messages import config_key, ping_arrays, sonar_config

NODE_NAME = 'isss_ros_view'


class TopicSource:
    """The newest ``keep`` messages of an ``isss/raw`` topic, as a source for ``isss.viz.run_live``.

    After a configuration change (range, samples, phase model) only messages
    of the newest one are kept; the window keeps the phase model it opened with.
    """

    def __init__(self, node: Node, topic: str, keep: int = 600) -> None:
        self.config = None
        self._node = node
        self._topic = node.resolve_topic_name(topic)
        self._msgs = collections.deque(maxlen=keep)
        self._lock = threading.Lock()
        self._warned_change = False
        node.create_subscription(ISSS, topic, self._on_msg, 50)

    def _on_msg(self, msg: ISSS) -> None:
        with self._lock:
            if self._msgs and config_key(msg) != config_key(self._msgs[-1]):
                self._msgs.clear()
                if not self._warned_change:
                    self._node.get_logger().warn('Sonar configuration changed; restart the viewer '
                                                 'if the phase model changed')
                    self._warned_change = True
            self._msgs.append(msg)

    def start(self) -> None:
        """Wait for the first message: it carries the configuration the window needs."""
        log = self._node.get_logger()
        log.info(f'Waiting for {self._topic} ...')
        last_note = time.monotonic()
        while not len(self):
            if self.finished:
                raise KeyboardInterrupt
            time.sleep(0.1)
            if time.monotonic() - last_note > 5.0:
                log.info(f'Still nothing on {self._topic}: is the driver running, with the same '
                         'RMW_IMPLEMENTATION and ROS_DOMAIN_ID?')
                last_note = time.monotonic()
        with self._lock:
            self.config = sonar_config(self._msgs[-1])
        log.info(f'Receiving {self._topic}')

    def stop(self) -> None:
        pass

    @property
    def finished(self) -> bool:
        """ROS shut down (Ctrl-C or SIGTERM: rclpy handles them), so the window should close."""
        return not rclpy.ok(context=self._node.context)

    def __len__(self) -> int:
        with self._lock:
            return len(self._msgs)

    def arrays(self):
        with self._lock:
            msgs = list(self._msgs)
        return ping_arrays(msgs)


def parse_args(argv):
    from isss_proc.cli import filter_args

    parser = argparse.ArgumentParser(
        prog=NODE_NAME,
        description='View a live isss/raw topic in the isss-view window. Click the waterfall or use the '
        'slider to pick a ping; arrow keys step, p toggles phase, f follows the newest ping, '
        '1/2 flip the port/starboard angle. ROS arguments go after --ros-args.')
    parser.add_argument('--topic', default='isss/raw',
                        help='ISSS topic; relative names follow the node namespace (default: isss/raw)')
    parser.add_argument('--keep', type=int, default=600, help='Pings to keep in the waterfall')
    parser.add_argument('--filters', action='store_true',
                        help='Overlay the processing filters, as isss-proc-view does (options below)')
    parser.add_argument('--trail', type=int, default=50, help='With --filters: pings shown in the profile')
    filter_args(parser)
    return parser.parse_args(argv)


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(args if args is not None else sys.argv)[1:]
    opts = parse_args(argv)

    import matplotlib.pyplot as plt  # noqa: F401  (fail before ROS starts if matplotlib is missing)

    from isss.viz import run_live

    viewer_kwargs = {}
    if opts.filters:
        from isss_proc.cli import pipeline_from
        from isss_proc.pipeline import viewer_keep
        viewer_kwargs = {'keep': viewer_keep(pipeline_from(opts), opts.context), 'trail': opts.trail}

    rclpy.init(args=args)
    node = Node(NODE_NAME)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    try:
        source = TopicSource(node, opts.topic, opts.keep)
        spinner.start()
        run_live(source, title=f'ROS {source._topic}', **viewer_kwargs)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
