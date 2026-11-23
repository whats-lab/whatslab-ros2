#!/usr/bin/env python3

import time
import traceback
from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from airglove_driver import convert
from airglove_driver.topics import (
    ALARM_TOPIC,
    SIDES,
    STATUS_TOPIC,
    haptics_result_topic,
    haptics_topic,
    joints_topic,
    wrist_topic,
)
from airglove_msgs.msg import (
    Alarm,
    DeviceStatus,
    HandJoints,
    Haptics,
    HapticsResult,
    WristOrientation,
)

if TYPE_CHECKING:
    import airglove_client

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
)
EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
)
RESEND_S = 0.2
STOP_RETRY_WINDOW_S = 5.0
ORPHAN_TICKS = 2
_INSTALL_HINT = (
    "airglove-client 가 없습니다: pip install --break-system-packages 'airglove-client @ "
    "git+https://github.com/whats-lab/AirGlove-Client.git@v1.0.0#subdirectory=python'"
)


def make_client(listen_port: int, spine_port: int, pair: int) -> 'airglove_client.Client':
    try:
        import airglove_client
    except ImportError as e:
        raise ImportError(_INSTALL_HINT) from e
    return airglove_client.Client('127.0.0.1', listen_port, '127.0.0.1', spine_port, pair=pair)


class AirGloveDriver(Node):

    def __init__(
        self, client_factory: Callable[[int, int, int], 'airglove_client.Client'] = make_client
    ):
        super().__init__('airglove_driver')
        self.declare_parameter('listen_port', 4040)
        self.declare_parameter('spine_port', 4042)
        self.declare_parameter('pair', 1)
        self.declare_parameter('publish_hz', 120.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('haptics_timeout', 1.0, ParameterDescriptor(dynamic_typing=True))

        def _p(name):
            return self.get_parameter(name).value

        publish_hz = float(_p('publish_hz'))
        if publish_hz <= 0:
            raise ValueError(f'publish_hz 는 0 보다 커야 한다: {publish_hz}')
        self._haptics_timeout = float(_p('haptics_timeout'))
        if self._haptics_timeout < 0:
            raise ValueError(f'haptics_timeout 은 0 이상(0 = 끔): {self._haptics_timeout}')
        self._pair = int(_p('pair'))
        self._client = client_factory(int(_p('listen_port')), int(_p('spine_port')), self._pair)
        self._hand_seq = {s: None for s in SIDES}
        self._wrist_seq = {s: None for s in SIDES}
        self._result_latest = {s: None for s in SIDES}
        self._protocol = -1
        self._haptics = {s: convert.HAPTICS_OFF for s in SIDES}
        self._stop_deadline = {s: None for s in SIDES}
        self._sent_at = {s: 0.0 for s in SIDES}
        self._stop_sends = {s: 0 for s in SIDES}
        self._command_at = {s: 0.0 for s in SIDES}
        self._orphan_ticks = {s: 0 for s in SIDES}

        self._joints_pub = {
            s: self.create_publisher(HandJoints, joints_topic(s), SENSOR_QOS) for s in SIDES
        }
        self._wrist_pub = {
            s: self.create_publisher(WristOrientation, wrist_topic(s), SENSOR_QOS) for s in SIDES
        }
        self._result_pub = {
            s: self.create_publisher(HapticsResult, haptics_result_topic(s), EVENT_QOS)
            for s in SIDES
        }
        self._status_pub = self.create_publisher(DeviceStatus, STATUS_TOPIC, EVENT_QOS)
        self._alarm_pub = self.create_publisher(Alarm, ALARM_TOPIC, EVENT_QOS)
        for s in SIDES:
            self.create_subscription(
                Haptics, haptics_topic(s), lambda m, s=s: self._on_haptics(s, m), EVENT_QOS
            )

        self.create_timer(1.0 / publish_hz, self._poll)
        self.create_timer(1.0, self._on_status_timer)
        self.get_logger().info(
            f'airglove_driver ready | Spine 127.0.0.1:{_p("spine_port")} → :{_p("listen_port")} '
            f'(pair {self._pair}, airglove_client {self._client.version()})'
        )

    def _isolated(self, step) -> None:
        try:
            step()
        except Exception:
            self.get_logger().error(
                f'airglove_client 오류\n{traceback.format_exc()}', throttle_duration_sec=1.0
            )

    def _stamp(self, now, age_s: float):
        age = Duration(seconds=max(0.0, float(age_s)))
        if now.nanoseconds < age.nanoseconds:
            return now.to_msg()
        return (now - age).to_msg()

    def _poll(self) -> None:
        if not rclpy.ok():
            return
        now = self.get_clock().now()
        for i, side in enumerate(SIDES):
            self._isolated(lambda i=i, side=side: self._keep_haptics_safe(i, side))
            self._isolated(lambda i=i, side=side: self._poll_haptics(i, side, now))
            self._isolated(lambda i=i, side=side: self._poll_side(i, side, now))
        self._isolated(self._drain_alarms)

    def _poll_side(self, i: int, side: str, now) -> None:
        hand = self._client.hand(i)
        if hand is not None and hand.seq != self._hand_seq[side]:
            self._hand_seq[side] = hand.seq
            self._joints_pub[side].publish(
                convert.hand_msg(hand, side, self._pair, self._stamp(now, hand.age_s))
            )
        wrist = self._client.wrist(i)
        if wrist is not None and wrist.seq != self._wrist_seq[side]:
            self._wrist_seq[side] = wrist.seq
            self._wrist_pub[side].publish(
                convert.wrist_msg(wrist, side, self._pair, self._stamp(now, wrist.age_s))
            )

    def _keep_haptics_safe(self, i: int, side: str) -> None:
        mono = time.monotonic()
        self._stop_stale_haptics(side, mono)
        self._retry_stop(i, side, mono)

    def _poll_haptics(self, i: int, side: str, now) -> None:
        before = time.monotonic()
        result = self._client.haptics_result(i)
        after = time.monotonic()
        if result is not None:
            success, age = result[0], float(result[1])
            is_new, self._result_latest[side] = convert.track_result(
                age, before, after, self._result_latest[side]
            )
            if is_new:
                self._result_pub[side].publish(
                    convert.haptics_result_msg(success, side, self._pair, self._stamp(now, age))
                )
                self._on_result(side, success, before - age)

    def _on_result(self, side: str, success: bool, earliest_arrival: float) -> None:
        if success:
            confirmed = self._stop_sends[side] >= 2 and earliest_arrival > self._sent_at[side]
            if confirmed:
                self._stop_deadline[side] = None
        elif any(self._haptics[side]):
            self.get_logger().warn(
                f'[{side}] 햅틱 명령 거절됨(장갑이 바쁘거나 끊김)', throttle_duration_sec=1.0
            )

    def _retry_stop(self, i: int, side: str, mono: float) -> None:
        deadline = self._stop_deadline[side]
        if deadline is None or mono - self._sent_at[side] < RESEND_S:
            return
        if mono > deadline:
            self._stop_deadline[side] = None
            self.get_logger().warn(f'[{side}] 햅틱 끄기가 확인되지 않아 재전송을 멈춤 — 연결 확인')
            return
        self._sent_at[side] = mono
        self._stop_sends[side] += 1
        if self._stop_sends[side] > 2:
            self.get_logger().warn(f'[{side}] 끄기 미확인 — 재전송', throttle_duration_sec=1.0)
        self._client.set_haptics(i, convert.HAPTICS_OFF)

    def _drain_alarms(self) -> None:
        now = self.get_clock().now()
        for alarm in self._client.alarms():
            self._alarm_pub.publish(convert.alarm_msg(alarm, self._pair, now.to_msg()))
            where = 'unknown' if alarm.side is None else SIDES[int(alarm.side)]
            self.get_logger().warn(f'Spine 알람: {alarm.code} ({where})')

    def _on_status_timer(self) -> None:
        self._isolated(self._stop_orphaned_haptics)
        self._isolated(self._status)

    def _status(self) -> None:
        if not rclpy.ok():
            return
        msg = convert.status_msg(
            self._client.device_status(),
            self._client.spine_protocol(),
            self._pair,
            self.get_clock().now().to_msg(),
        )
        self._status_pub.publish(msg)
        version = msg.spine_protocol
        if version != self._protocol:
            self._protocol = version
            if version == 0:
                self.get_logger().warn(
                    'Spine 응답 없음 — Spine 3.0.0 이 같은 PC 에서 손 출력을 켠 채 도는지 확인'
                )
            else:
                self.get_logger().info(f'Spine 프로토콜 {version}')

    def _stop_stale_haptics(self, side: str, mono: float) -> None:
        if self._haptics_timeout <= 0 or not any(self._haptics[side]):
            return
        if mono - self._command_at[side] > self._haptics_timeout:
            self.get_logger().warn(f'[{side}] 햅틱 명령이 {self._haptics_timeout}s 없어 끈다')
            self._command(side, convert.HAPTICS_OFF)

    def _stop_orphaned_haptics(self) -> None:
        if not rclpy.ok():
            return
        for side in SIDES:
            orphaned = any(self._haptics[side]) and self.count_publishers(haptics_topic(side)) == 0
            self._orphan_ticks[side] = self._orphan_ticks[side] + 1 if orphaned else 0
            if self._orphan_ticks[side] >= ORPHAN_TICKS:
                self.get_logger().warn(f'[{side}] 햅틱 발행자가 사라져 진동을 끈다')
                self._command(side, convert.HAPTICS_OFF)

    def _on_haptics(self, side: str, msg: Haptics) -> None:
        mono = time.monotonic()
        self._command_at[side] = mono
        values = convert.strengths(msg.strength)
        same = tuple(values) == tuple(self._haptics[side])
        if same and (not any(values) or mono - self._sent_at[side] < RESEND_S):
            return
        self._command(side, values)

    def _command(self, side: str, values: Sequence[int]) -> None:
        mono = time.monotonic()
        self._haptics[side] = values
        self._sent_at[side] = mono
        self._stop_deadline[side] = None if any(values) else mono + STOP_RETRY_WINDOW_S
        self._stop_sends[side] = 0 if any(values) else 1
        try:
            self._client.set_haptics(convert.side_index(side), values)
        except Exception:
            self.get_logger().error(f'[{side}] 햅틱 전송 실패\n{traceback.format_exc()}')

    def destroy_node(self) -> None:
        for attempt in range(2):
            if attempt:
                time.sleep(RESEND_S)
            for i, side in enumerate(SIDES):
                try:
                    self._client.set_haptics(i, convert.HAPTICS_OFF)
                except Exception:
                    self.get_logger().warn(
                        f'[{side}] 종료 시 햅틱 끄기 실패\n{traceback.format_exc()}'
                    )
        try:
            self._client.close()
        finally:
            super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AirGloveDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
