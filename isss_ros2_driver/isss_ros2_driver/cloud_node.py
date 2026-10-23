"""ROS 2 processing node: filtered ISSS point clouds from ``isss/raw``.

The processing lives in ``isss_proc`` and the message conversion in
``isss.sources``. This node only buffers ``isss_msgs/ISSS`` messages, pushes
them to an ``isss_proc.Stream`` a block at a time, and publishes the kept
returns of each ping as ``sensor_msgs/PointCloud2`` (layout: ``isss_proc.cloud``).
"""

from array import array

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from isss.sources import config_from_message_fields, pings_from_message_fields
from isss_msgs.msg import ISSS
from isss_proc import Stream, make_pipeline
from isss_proc.cloud import POINT_FIELDS, POINT_STEP, pack_points

NODE_NAME = 'isss_cloud_node'

# Defaults and types match config/isss_cloud.yaml; keep them in step.
PARAMETERS = {
    'block': 5,              # pings pushed at a time: the chain takes ~50 ms per 200-ping window
    'context': 200,          # pings of history each block is processed with
    'min_range': 1.0,
    'max_range': 0.0,        # 0: no limit
    'min_excess': 30.0,      # 0: no intensity stage
    'bias_states': True,
    'first_surface': True,
    'mpaa': False,
    'mpaa_pings': 1,
    'mpaa_band': 3.0,
}


class IsssCloudNode(Node):

    def __init__(self, **kwargs):
        super().__init__(NODE_NAME, **kwargs)
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        _validate(p)

        self._block = p['block']
        self._context = p['context']
        self._pipeline = make_pipeline(
            min_range=p['min_range'],
            max_range=p['max_range'] or None,
            min_excess=p['min_excess'],
            bias_states=p['bias_states'],
            first_surface=p['first_surface'],
            mpaa=p['mpaa'],
            mpaa_pings=p['mpaa_pings'],
            mpaa_band=p['mpaa_band'],
        )
        self._stream = None
        self._pending = []
        self._fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                        for name, offset, datatype in POINT_FIELDS]

        self._cloud_pub = self.create_publisher(PointCloud2, 'isss/cloud', 10)
        self.create_subscription(ISSS, 'isss/raw', self._on_ping, 50)
        self.get_logger().info(f'Filtering isss/raw with {", ".join(self._pipeline.names)} '
                               f'in blocks of {self._block} pings')

    def _on_ping(self, msg: ISSS) -> None:
        # A block holds pings of one configuration; a change processes what came before it
        if self._pending and _key(msg) != _key(self._pending[0]):
            self._process()
        self._pending.append(msg)
        if len(self._pending) >= self._block:
            self._process()

    def _process(self) -> None:
        msgs, self._pending = self._pending, []
        first = msgs[0]
        config = config_from_message_fields(
            range=first.range,
            num_samples=len(first.port_intensity),
            sound_velocity=first.sound_velocity,
            counts_per_unit_sine=first.counts_per_unit_sine,
            boresight_depression=first.boresight_depression,
            port_phase_sign=first.port_phase_sign,
            starboard_phase_sign=first.starboard_phase_sign,
        )
        if self._stream is None or config != self._stream.config:
            self._stream = Stream(config, self._pipeline, context=self._context)
        try:
            arrays = pings_from_message_fields(
                [m.ping_time for m in msgs],
                [m.decimation for m in msgs],
                [m.port_intensity for m in msgs],
                [m.starboard_intensity for m in msgs],
                [m.port_phase_counts for m in msgs],
                [m.starboard_phase_counts for m in msgs],
            )
        except ValueError as exc:
            self.get_logger().warn(f'Dropping {len(msgs)} pings: {exc}', throttle_duration_sec=5.0)
            return
        points = self._stream.push(arrays).points()
        # One cloud per ping, stamped and framed as that ping, so TF places each where it was taken
        for i, msg in enumerate(msgs):
            mine = points['ping'] == i
            self._cloud_pub.publish(self._cloud(msg, {k: v[mine] for k, v in points.items()}))

    def _cloud(self, msg: ISSS, points: dict) -> PointCloud2:
        n = int(points['y'].size)
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = n
        cloud.fields = self._fields
        cloud.is_bigendian = False
        cloud.point_step = POINT_STEP
        cloud.row_step = POINT_STEP * n
        cloud.data = array('B', pack_points(points))
        cloud.is_dense = True
        return cloud


def _key(msg: ISSS) -> tuple:
    return (len(msg.port_intensity), msg.decimation, msg.range, msg.sound_velocity, msg.counts_per_unit_sine,
            msg.boresight_depression, msg.port_phase_sign, msg.starboard_phase_sign)


def _validate(p: dict) -> None:
    if p['block'] < 1:
        raise ValueError("Parameter 'block' must be >= 1")
    if p['context'] < 0:
        raise ValueError("Parameter 'context' must be >= 0")
    if p['max_range'] and p['max_range'] <= p['min_range']:
        raise ValueError("Parameter 'max_range' must be 0 (no limit) or > 'min_range'")
    if p['mpaa_pings'] < 1:
        raise ValueError("Parameter 'mpaa_pings' must be >= 1")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IsssCloudNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as exc:
        rclpy.logging.get_logger(NODE_NAME).fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
