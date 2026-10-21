"""ROS 2 wrapper around ``isss.SonarDriver``.

Everything about talking to the sonar lives in ``isss_core``. This node only maps
parameters to a ``SonarConfig``, pings to ``isss_msgs/ISSS``, driver status to
``ISSSStatus`` and ``/diagnostics``, and services to driver calls.
"""

import math
from array import array

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger

from isss import NotConnectedError, Ping, SonarConfig, SonarDriver
from isss.protocol import sample_interval
from isss_msgs.msg import ISSS, ISSSStatus
from isss_msgs.srv import ConfigureSonar

NODE_NAME = 'isss_ros2_driver_node'

# Defaults and types match config/isss_driver.yaml; a YAML value of a different
# type (e.g. 2 vs 2.0) is rejected by rclpy, so keep them in step.
PARAMETERS = {
    'ip': '192.168.2.62',
    'port': 57349,
    'range': 20.0,
    'num_samples': 1000,
    'num_phases': 2,
    'save_path': '',
    'export_netcdf': True,
    'frame_id': 'isss_link',
    'sound_velocity': 1440.9,
    'counts_per_unit_sine': 15218.0,
    'boresight_depression_deg': 44.5,
    'port_phase_sign': 1,
    'starboard_phase_sign': 1,
    'recv_timeout_sec': 2,
    'use_device_time': True,
    'device_time_offset_sec': 0.0,
    'reconnect_interval_sec': 5.0,
}


class IsssDriverNode(Node):

    def __init__(self, **kwargs):
        super().__init__(NODE_NAME, **kwargs)
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        _validate(p)

        self._frame_id = p['frame_id']
        self._use_device_time = p['use_device_time']
        self._device_time_offset_sec = p['device_time_offset_sec']
        self._save_path = p['save_path']
        self._config = SonarConfig(
            ip=p['ip'],
            port=p['port'],
            range_m=p['range'],
            num_samples=p['num_samples'],
            num_phases=p['num_phases'],
            sound_velocity=p['sound_velocity'],
            counts_per_unit_sine=p['counts_per_unit_sine'],
            boresight_depression_deg=p['boresight_depression_deg'],
            port_phase_sign=p['port_phase_sign'],
            starboard_phase_sign=p['starboard_phase_sign'],
            recv_timeout_sec=float(p['recv_timeout_sec']),
            reconnect_interval_sec=p['reconnect_interval_sec'],
        )

        self._isss_pub = self.create_publisher(ISSS, 'isss/raw', 10)
        self._status_pub = self.create_publisher(ISSSStatus, 'isss/status', 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)

        self.create_service(ConfigureSonar, '~/configure_sonar', self._handle_configure)
        self.create_service(Trigger, '~/start_logging', self._handle_start_logging)
        self.create_service(Trigger, '~/stop_logging', self._handle_stop_logging)
        self.create_timer(1.0, self._publish_status)

        self._driver = SonarDriver(
            self._config,
            self._publish_ping,
            log_dir=self._save_path or None,
            export_netcdf=p['export_netcdf'],
            logger=self.get_logger(),
        )
        self._driver.start()

    def destroy_node(self):
        self._driver.stop()
        return super().destroy_node()

    # Pings (called from the driver's read thread)

    def _publish_ping(self, ping: Ping) -> None:
        header, samples = ping.header, ping.samples
        msg = ISSS()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self._stamp(header.ping_time)
        msg.packet_type = header.packet_type
        msg.ping_time = header.ping_time
        msg.frequency_id = header.frequency_id
        msg.decimation = header.decimation
        msg.range = float(self._config.range_m)
        msg.sample_interval = sample_interval(header.decimation)
        msg.sound_velocity = float(self._config.sound_velocity)
        msg.counts_per_unit_sine = float(self._config.counts_per_unit_sine)
        msg.boresight_depression = math.radians(self._config.boresight_depression_deg)
        msg.port_phase_sign = self._config.port_phase_sign
        msg.starboard_phase_sign = self._config.starboard_phase_sign
        # array.array skips rclpy's per-element range checks on 1000-sample arrays
        msg.port_intensity = array('B', samples.port_rss)
        msg.starboard_intensity = array('B', samples.starboard_rss)
        msg.port_phase_counts = array('h', samples.port_phase_counts)
        msg.starboard_phase_counts = array('h', samples.starboard_phase_counts)
        msg.port_depression = array('f', samples.port_depression)
        msg.starboard_depression = array('f', samples.starboard_depression)
        self._isss_pub.publish(msg)

    def _stamp(self, ping_time_100us: int):
        if not self._use_device_time:
            return self.get_clock().now().to_msg()
        seconds = ping_time_100us * 1e-4 + self._device_time_offset_sec
        return Time(nanoseconds=int(seconds * 1e9)).to_msg()

    # Status

    def _publish_status(self) -> None:
        state = self._driver.status()
        status = ISSSStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self._frame_id
        status.connected = state.connected
        status.ping_rate_hz = float(state.ping_rate_hz)
        status.last_error = state.last_error
        status.range = float(state.range_m)
        status.num_samples = state.num_samples
        status.num_phases = state.num_phases
        status.decimation = state.decimation
        self._status_pub.publish(status)

        diag = DiagnosticStatus()
        diag.name = 'isss_ros2_driver'
        diag.hardware_id = self._config.ip
        diag.level = DiagnosticStatus.OK if state.connected else DiagnosticStatus.ERROR
        diag.message = 'Streaming' if state.connected else state.last_error
        diag.values = [
            KeyValue(key='ping_rate_hz', value=f'{state.ping_rate_hz:.2f}'),
            KeyValue(key='range_m', value=f'{state.range_m:.2f}'),
            KeyValue(key='num_samples', value=str(state.num_samples)),
            KeyValue(key='decimation', value=str(state.decimation)),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = status.header.stamp
        diagnostics.status = [diag]
        self._diagnostics_pub.publish(diagnostics)

    # Services

    def _handle_configure(self, request, response):
        try:
            self._driver.configure(
                range_m=request.range,
                num_samples=request.num_samples,
                num_phases=request.num_phases,
            )
        except ValueError:
            response.success, response.message = False, 'Invalid configuration values'
        except NotConnectedError:
            response.success, response.message = False, 'Sonar is not connected'
        except OSError:
            response.success = False
            response.message = 'Failed to write sonar configuration registers'
        else:
            response.success, response.message = True, 'Sonar configured successfully'
        return response

    def _handle_start_logging(self, _request, response):
        if not self._save_path:
            response.success = False
            response.message = 'save_path parameter is empty; set it to enable logging'
            return response
        try:
            path = self._driver.start_logging()
        except OSError as exc:
            response.success, response.message = False, f'Failed to open log file: {exc}'
        else:
            response.success, response.message = True, f'Logging started: {path}'
        return response

    def _handle_stop_logging(self, _request, response):
        self._driver.stop_logging()
        response.success, response.message = True, 'Logging stopped'
        return response


def _validate(p: dict) -> None:
    if p['range'] <= 0.0:
        raise ValueError("Parameter 'range' must be > 0")
    if p['num_samples'] <= 0:
        raise ValueError("Parameter 'num_samples' must be > 0")
    if p['num_phases'] < 0:
        raise ValueError("Parameter 'num_phases' must be >= 0")
    if not 0 < p['port'] <= 65535:
        raise ValueError("Parameter 'port' must be in [1, 65535]")
    for name in ('port_phase_sign', 'starboard_phase_sign'):
        if p[name] not in (1, -1):
            raise ValueError(f"Parameter '{name}' must be 1 or -1")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IsssDriverNode()
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
