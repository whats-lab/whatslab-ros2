from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
from builtin_interfaces.msg import Time

from airglove_driver.topics import SIDES, wrist_frame
from airglove_msgs.msg import Alarm, DeviceStatus, HandJoints, HapticsResult, WristOrientation

if TYPE_CHECKING:
    import airglove_client

FINGER_COUNT = 5
HAPTIC_MAX = 3
HAPTICS_OFF = (0,) * FINGER_COUNT
STALE_RESULT_S = 1.0
STALE_STATUS_S = 3.0


def side_index(side: str) -> int:
    return SIDES.index(side)


def hand_msg(frame: 'airglove_client.HandFrame', side: str, pair: int, stamp: Time) -> HandJoints:
    msg = HandJoints()
    msg.header.stamp = stamp
    msg.header.frame_id = wrist_frame(side)
    msg.side = side_index(side)
    msg.pair = int(pair)
    msg.seq = int(frame.seq) & 0xFFFFFFFF
    data = np.concatenate(
        [
            np.asarray(frame.orientations, np.float32),
            np.asarray(frame.positions, np.float32),
            np.asarray(frame.radii, np.float32).reshape(-1, 1),
        ],
        axis=1,
    )
    msg.joints = data.reshape(-1).tolist()
    msg.valid = [bool(v) for v in np.asarray(frame.valid, dtype=bool)]
    return msg


def wrist_msg(
    frame: 'airglove_client.WristFrame', side: str, pair: int, stamp: Time
) -> WristOrientation:
    msg = WristOrientation()
    msg.header.stamp = stamp
    msg.header.frame_id = wrist_frame(side)
    msg.side = side_index(side)
    msg.pair = int(pair)
    msg.seq = int(frame.seq) & 0xFFFFFFFF
    msg.raw = [float(v) for v in np.asarray(frame.orientation, np.float32)[:4]]
    return msg


def status_msg(
    status: 'airglove_client.DeviceStatus | None',
    protocol: 'tuple[int, float] | None',
    pair: int,
    stamp: Time,
) -> DeviceStatus:
    msg = DeviceStatus()
    msg.header.stamp = stamp
    msg.pair = int(pair)
    if status is not None and float(status.age_s) <= STALE_STATUS_S:
        msg.left_connected = bool(status.left_connected)
        msg.right_connected = bool(status.right_connected)
    if protocol is not None:
        fresh = float(protocol[1]) <= STALE_STATUS_S
        msg.spine_protocol = int(protocol[0]) if fresh else 0
        msg.spine_age = float(protocol[1])
    else:
        msg.spine_protocol = 0
        msg.spine_age = -1.0
    return msg


def alarm_msg(alarm: 'airglove_client.Alarm', pair: int, stamp: Time) -> Alarm:
    msg = Alarm()
    msg.header.stamp = stamp
    msg.pair = int(pair)
    msg.code = str(alarm.code)
    if alarm.side is None:
        msg.side = Alarm.SIDE_UNKNOWN
    else:
        msg.side = Alarm.SIDE_LEFT if int(alarm.side) == 0 else Alarm.SIDE_RIGHT
    return msg


def haptics_result_msg(success: bool, side: str, pair: int, stamp: Time) -> HapticsResult:
    msg = HapticsResult()
    msg.header.stamp = stamp
    msg.pair = int(pair)
    msg.side = side_index(side)
    msg.success = bool(success)
    return msg


def track_result(
    age_s: float, before_s: float, after_s: float, last_latest_s: float | None
) -> tuple[bool, float]:
    earliest, latest = before_s - age_s, after_s - age_s
    if last_latest_s is None:
        return age_s < STALE_RESULT_S, latest
    if earliest > last_latest_s:
        return True, latest
    return False, min(last_latest_s, latest)


def strengths(values: Sequence[int]) -> list[int]:
    return [min(int(v), HAPTIC_MAX) for v in values]
