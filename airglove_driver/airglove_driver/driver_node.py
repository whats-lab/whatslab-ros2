#!/usr/bin/env python3
"""airglove_driver — Spine 3.0.0 AirGlove 를 ROS 2 로 내보내는 드라이버 노드.

Spine 은 같은 PC 의 UDP 4040 클라이언트 하나에만 보낸다. 이 노드가 그 클라이언트이고
(AirGlove-Client `airglove_client`), 받은 값을 Spine 형식 그대로 airglove_msgs 로 발행한다.

  pub  /airglove/{left,right}/joints          airglove_msgs/HandJoints        새 손 프레임마다
  pub  /airglove/{left,right}/wrist           airglove_msgs/WristOrientation  새 손목 값마다
  pub  /airglove/status                       airglove_msgs/DeviceStatus      1 Hz
  pub  /airglove/alarm                        airglove_msgs/Alarm             알람마다
  pub  /airglove/{left,right}/haptics_result  airglove_msgs/HapticsResult     햅틱 명령 결과마다
  sub  /airglove/{left,right}/haptics         airglove_msgs/Haptics

폴링·발행은 executor 스레드의 타이머에서만 한다.
"""

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

if TYPE_CHECKING:  # airglove_client 는 make_client 안에서 늦게 import 한다
    import airglove_client

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
)
EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
)
RESEND_S = 0.2  # 같은 세기 재전송·확인 안 된 끄기 재전송의 최소 간격
STOP_RETRY_WINDOW_S = 5.0  # 끄기가 이만큼 확인 안 되면 장갑이 끊긴 것으로 보고 그만 보낸다
ORPHAN_TICKS = 2  # 발행자 0 이 이만큼(1 Hz) 이어져야 끈다 — 그래프 캐시 지연으로 잘못 끄지 않게
_INSTALL_HINT = (
    "airglove-client 가 없습니다: pip install --break-system-packages 'airglove-client @ "
    "git+https://github.com/whats-lab/AirGlove-Client.git@v0.4.0#subdirectory=python'"
)


def make_client(listen_port: int, spine_port: int, pair: int) -> 'airglove_client.Client':
    """같은 PC 의 Spine 과 통신하는 airglove_client.Client 를 연다."""
    try:
        import airglove_client
    except ImportError as e:
        raise ImportError(_INSTALL_HINT) from e
    return airglove_client.Client('127.0.0.1', listen_port, '127.0.0.1', spine_port, pair=pair)


class AirGloveDriver(Node):
    """airglove_client 를 폴링해 airglove_msgs 로 발행하고 햅틱 명령을 Spine 으로 보낸다."""

    def __init__(
        self, client_factory: Callable[[int, int, int], 'airglove_client.Client'] = make_client
    ):
        super().__init__('airglove_driver')
        self.declare_parameter('listen_port', 4040)
        self.declare_parameter('spine_port', 4042)
        self.declare_parameter('pair', 1)
        # 런치에서 publish_hz:=120 처럼 정수로 넘겨도 받도록 타입을 고정하지 않는다.
        self.declare_parameter('publish_hz', 120.0, ParameterDescriptor(dynamic_typing=True))
        # 진동 중 이 시간 [s] 동안 Haptics 가 안 오면 끈다(발행자 네트워크 끊김 대비). 0 = 끔.
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
        self._protocol = -1  # 아직 확인 전 — 첫 상태에서 응답이 없어도 경고하게 0 과 구분
        self._haptics = {s: convert.HAPTICS_OFF for s in SIDES}  # 마지막으로 보낸 세기
        self._stop_deadline = {s: None for s in SIDES}  # 미확인 끄기 재전송 마감(monotonic)
        self._sent_at = {s: 0.0 for s in SIDES}  # 마지막 set_haptics (monotonic)
        self._stop_sends = {s: 0 for s in SIDES}  # 이번 끄기를 보낸 횟수
        self._command_at = {s: 0.0 for s in SIDES}  # 마지막 Haptics 수신 (monotonic)
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
        except Exception:  # 타이머 경계 — 한 번의 클라이언트 오류로 모든 토픽이 멈추지 않게
            self.get_logger().error(
                f'airglove_client 오류\n{traceback.format_exc()}', throttle_duration_sec=1.0
            )

    def _stamp(self, now, age_s: float):
        age = Duration(seconds=max(0.0, float(age_s)))
        if now.nanoseconds < age.nanoseconds:  # sim time 에서 /clock 전(now=0) — 음수 시각 방지
            return now.to_msg()
        return (now - age).to_msg()

    def _poll(self) -> None:
        if not rclpy.ok():
            return
        now = self.get_clock().now()
        # 한쪽 손·햅틱·알람의 오류가 서로를 막지 않게 따로 격리한다.
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
        """결과는 명령 번호가 없어 어느 명령의 것인지 모른다 — 끄기는 안전 쪽으로만 확정한다.

        끄기는 두 번 이상 보낸 뒤, 마지막 전송 이후에 도착한 성공만 확정으로 본다. 첫 전송 직후의
        성공은 앞 명령(예: 3)의 늦은 응답일 수 있다.
        """
        if success:
            confirmed = self._stop_sends[side] >= 2 and earliest_arrival > self._sent_at[side]
            if confirmed:
                self._stop_deadline[side] = None
        elif any(self._haptics[side]):
            self.get_logger().warn(
                f'[{side}] 햅틱 명령 거절됨(장갑이 바쁘거나 끊김)', throttle_duration_sec=1.0
            )

    def _retry_stop(self, i: int, side: str, mono: float) -> None:
        """성공 결과가 안 온 끄기 명령을 RESEND_S 마다, 최대 STOP_RETRY_WINDOW_S 동안 다시 보낸다.

        드라이버 하트비트가 살아 있는 동안은 Spine 의 3 초 자동 끔이 없어, 거절(바쁨)이나 UDP
        유실 뒤 포기하면 계속 떤다. 장갑이 끊겨 계속 실패하면 창이 끝나 멈춘다.
        """
        deadline = self._stop_deadline[side]
        if deadline is None or mono - self._sent_at[side] < RESEND_S:
            return
        if mono > deadline:
            self._stop_deadline[side] = None
            self.get_logger().warn(f'[{side}] 햅틱 끄기가 확인되지 않아 재전송을 멈춤 — 연결 확인')
            return
        self._sent_at[side] = mono
        self._stop_sends[side] += 1
        if self._stop_sends[side] > 2:  # 두 번째는 늘 보내는 확인용 — 그 뒤부터가 진짜 재전송
            self.get_logger().warn(f'[{side}] 끄기 미확인 — 재전송', throttle_duration_sec=1.0)
        self._client.set_haptics(i, convert.HAPTICS_OFF)

    def _drain_alarms(self) -> None:
        now = self.get_clock().now()
        for alarm in self._client.alarms():
            self._alarm_pub.publish(convert.alarm_msg(alarm, self._pair, now.to_msg()))
            where = 'unknown' if alarm.side is None else SIDES[int(alarm.side)]
            self.get_logger().warn(f'Spine 알람: {alarm.code} ({where})')

    def _on_status_timer(self) -> None:
        self._isolated(self._stop_orphaned_haptics)  # 상태 조회 오류와 무관하게 돌게 먼저 따로
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
        version = msg.spine_protocol  # 0 = 응답 없음(또는 끊김)
        if version != self._protocol:
            self._protocol = version
            if version == 0:
                self.get_logger().warn(
                    'Spine 응답 없음 — Spine 3.0.0 이 같은 PC 에서 손 출력을 켠 채 도는지 확인'
                )
            else:
                self.get_logger().info(f'Spine 프로토콜 {version}')

    def _stop_stale_haptics(self, side: str, mono: float) -> None:
        """진동 중인데 haptics_timeout 동안 명령이 없으면 끈다.

        원격 발행자의 네트워크가 끊기면 DDS lease(수십 초)가 끝날 때까지 발행자 수로는 모른다.
        """
        if self._haptics_timeout <= 0 or not any(self._haptics[side]):
            return
        if mono - self._command_at[side] > self._haptics_timeout:
            self.get_logger().warn(f'[{side}] 햅틱 명령이 {self._haptics_timeout}s 없어 끈다')
            self._command(side, convert.HAPTICS_OFF)

    def _stop_orphaned_haptics(self) -> None:
        """진동 중인데 명령을 보내던 노드가 사라졌으면(종료 등) 끈다."""
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
        # 발행자는 계속 발행한다 — 같은 세기는 RESEND_S 마다만 보내 Spine 을 바쁘게 만들지 않는다
        # (거절된 같은 명령도 이 간격으로 다시 간다). 같은 끄기는 _retry_stop 이 맡는다.
        same = tuple(values) == tuple(self._haptics[side])
        if same and (not any(values) or mono - self._sent_at[side] < RESEND_S):
            return
        self._command(side, values)

    def _command(self, side: str, values: Sequence[int]) -> None:
        mono = time.monotonic()
        self._haptics[side] = values
        self._sent_at[side] = mono
        # 끄기는 성공 결과가 올 때까지 _retry_stop 이 다시 보낸다(거절·UDP 유실·전송 실패 모두).
        self._stop_deadline[side] = None if any(values) else mono + STOP_RETRY_WINDOW_S
        self._stop_sends[side] = 0 if any(values) else 1
        try:
            self._client.set_haptics(convert.side_index(side), values)
        except Exception:  # 콜백 경계 — 명령 하나의 전송 실패로 드라이버가 죽지 않게
            self.get_logger().error(f'[{side}] 햅틱 전송 실패\n{traceback.format_exc()}')

    def destroy_node(self) -> None:
        # 종료 시 진동을 끈다 — RESEND_S 간격으로 두 번(유실·바쁨 대비, Spine 도 3 초 뒤 끈다).
        for attempt in range(2):
            if attempt:
                time.sleep(RESEND_S)
            for i, side in enumerate(SIDES):
                try:
                    self._client.set_haptics(i, convert.HAPTICS_OFF)
                except Exception:  # 종료 경계 — 한쪽 실패로 다른 손 끄기·close 를 건너뛰지 않게
                    self.get_logger().warn(
                        f'[{side}] 종료 시 햅틱 끄기 실패\n{traceback.format_exc()}'
                    )
        try:
            self._client.close()
        finally:
            super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """airglove_driver 노드를 띄운다."""
    rclpy.init(args=args)
    node = None
    try:
        node = AirGloveDriver()  # 생성 실패(설정 오류 등)도 아래 finally 에서 shutdown 한다
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            raise
        # SIGINT 이 context 를 먼저 닫아 spin·발행이 무효 context 를 만난 경우 — 정상 종료다.
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
