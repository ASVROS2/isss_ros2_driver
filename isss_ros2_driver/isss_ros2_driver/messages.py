"""``isss_msgs/ISSS`` messages to ``isss_core`` types, and the settings topics, shared by the nodes."""

import json
from datetime import datetime, timezone

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from isss.config import SonarConfig
from isss.diagnostics import PingArrays
from isss.sources import config_from_message_fields, pings_from_message_fields
from isss_msgs.msg import ISSS


def config_key(msg: ISSS) -> tuple:
    """What a run of pings must share to be processed or shown together."""
    return (len(msg.port_intensity), msg.decimation, msg.range, msg.sound_velocity, msg.counts_per_unit_sine,
            msg.boresight_depression, msg.port_phase_sign, msg.starboard_phase_sign)


def sonar_config(msg: ISSS) -> SonarConfig:
    """The ``SonarConfig`` the driver published ``msg`` with."""
    return config_from_message_fields(
        range=msg.range,
        num_samples=len(msg.port_intensity),
        sound_velocity=msg.sound_velocity,
        counts_per_unit_sine=msg.counts_per_unit_sine,
        boresight_depression=msg.boresight_depression,
        port_phase_sign=msg.port_phase_sign,
        starboard_phase_sign=msg.starboard_phase_sign,
    )


SETTINGS_FORMAT = 'isss-ros-settings'
SETTINGS_VERSION = 1
# Latched: a subscriber (or ros2 bag record) that starts later still gets the last message
SETTINGS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)


class SettingsPublisher:
    """Publishes a node's settings as JSON on a latched ``std_msgs/String`` topic, once per change.

    Each message is ``{"format": "isss-ros-settings", "version": 1, "utc": ...,
    "node": ..., "software": {...}, ...}`` with the node's own fields after that,
    so a bag holds every setting a recording was made with.
    """

    def __init__(self, node, topic: str, software: dict) -> None:
        self._node = node
        self._software = software
        self._pub = node.create_publisher(String, topic, SETTINGS_QOS)
        self._last = None

    def publish(self, fields: dict) -> bool:
        """Publish ``fields`` if they differ from the last message; returns whether it did."""
        if fields == self._last:
            return False
        self._last = fields
        message = {
            'format': SETTINGS_FORMAT,
            'version': SETTINGS_VERSION,
            'utc': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
            'node': self._node.get_fully_qualified_name(),
            'software': self._software,
            **fields,
        }
        self._pub.publish(String(data=json.dumps(message)))
        return True


def versions(*packages: str) -> dict:
    """Installed versions of ``packages`` (``None`` where unknown)."""
    from importlib.metadata import PackageNotFoundError, version

    found = {}
    for name in packages:
        try:
            found[name] = version(name)
        except PackageNotFoundError:
            found[name] = None
    return found


def ping_arrays(msgs: list) -> PingArrays:
    """``PingArrays`` of messages that share a ``config_key``; raises ``ValueError`` if their sample counts differ."""
    return pings_from_message_fields(
        [m.ping_time for m in msgs],
        [m.decimation for m in msgs],
        [m.port_intensity for m in msgs],
        [m.starboard_intensity for m in msgs],
        [m.port_phase_counts for m in msgs],
        [m.starboard_phase_counts for m in msgs],
    )
