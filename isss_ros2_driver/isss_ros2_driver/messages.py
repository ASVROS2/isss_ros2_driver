"""``isss_msgs/ISSS`` messages to ``isss_core`` types, shared by the nodes that subscribe to ``isss/raw``."""

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
