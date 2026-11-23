"""airglove_client 값 → airglove_msgs. rclpy 노드 없이 테스트할 수 있게 분리했다."""

from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
from builtin_interfaces.msg import Time

from airglove_driver.topics import SIDES, wrist_frame
from airglove_msgs.msg import Alarm, DeviceStatus, HandJoints, HapticsResult, WristOrientation

if TYPE_CHECKING:  # 타입 힌트용 — 테스트는 airglove_client 없이 돈다
    import airglove_client

FINGER_COUNT = 5
HAPTIC_MAX = 3
HAPTICS_OFF = (0,) * FINGER_COUNT
STALE_RESULT_S = 1.0  # 시작할 때 이미 이만큼 지난 햅틱 결과는 이전 실행의 것이라 버린다
STALE_STATUS_S = 3.0  # 1 초 주기 상태·하트비트 응답이 이만큼 없으면 끊긴 것으로 본다


def side_index(side: str) -> int:
    """'left' | 'right' → airglove_client 손 번호(0 | 1)."""
    return SIDES.index(side)


def hand_msg(frame: 'airglove_client.HandFrame', side: str, pair: int, stamp: Time) -> HandJoints:
    """airglove_client.HandFrame → HandJoints (26 × [q, p, radius] 그대로)."""
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
    """airglove_client.WristFrame → WristOrientation (Spine 의 [y, x, z, -w] 그대로)."""
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
    """device_status() / spine_protocol() 결과 → DeviceStatus.

    airglove_client 는 마지막 값을 age 와 함께 계속 돌려주므로, STALE_STATUS_S 보다 오래된
    상태는 연결 False, 프로토콜 0 으로 낸다. 응답이 한 번도 없으면 spine_age 는 -1.
    """
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
    """airglove_client.Alarm → Alarm. 손을 지정하지 않은 알람은 SIDE_UNKNOWN."""
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
    """haptics_result() 의 성공 여부 → HapticsResult."""
    msg = HapticsResult()
    msg.header.stamp = stamp
    msg.pair = int(pair)
    msg.side = side_index(side)
    msg.success = bool(success)
    return msg


def track_result(
    age_s: float, before_s: float, after_s: float, last_latest_s: float | None
) -> tuple[bool, float]:
    """햅틱 결과가 새로 왔는지 → (새 결과인가, 다음 비교에 쓸 도착 시각 상한).

    도착 시각은 [before - age, after - age] 안에 있다(before/after = 조회 직전·직후 monotonic).
    age 감소만 보면 결과가 연달아 올 때 놓치고, 한 시각만 쓰면 조회 중 지연으로 중복이 난다.
    새 결과는 도착 하한이 직전 결과의 상한보다 뒤일 때만 인정한다.
    """
    earliest, latest = before_s - age_s, after_s - age_s
    if last_latest_s is None:
        return age_s < STALE_RESULT_S, latest
    if earliest > last_latest_s:
        return True, latest
    return False, min(last_latest_s, latest)


def strengths(values: Sequence[int]) -> list[int]:
    """Haptics.strength(uint8[5]) → 0..HAPTIC_MAX 로 자른 세기."""
    return [min(int(v), HAPTIC_MAX) for v in values]
