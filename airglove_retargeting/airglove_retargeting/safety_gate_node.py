#!/usr/bin/env python3
"""
safety_gate — 텔레옵 명령 안전 게이트 ROS 노드 (whatslab.safety 얇은 래퍼).

  ~/command_raw (JointState) ─ whatslab.safety.SafetyFilter
      (위치 clamp · 속도 rate-limit · watchdog · 래칭 e-stop · deadman) ─► ~/command_safe

defense-in-depth 의 한 층 — **HW e-stop / 모터 한계가 최종 권위**, 이 게이트는 보조.
ros2_control 드라이버면 그 enforce_command_limits 가 위치/속도 강제를 맡고 이 게이트는
텔레옵 품질층(watchdog/deadman/estop)만; non-ros2_control(SO-101)이면 clamp 까지 담당.
**게이트가 드라이버 command 의 유일 publisher 여야 한다(우회 금지).**

파라미터:
  robot_description(str) : URDF (limit 소스, 필수)
  limits_override(str)   : yaml {joint:{lower,upper,velocity}} — 더 빡빡하게(선택)
  timeout(float)         : watchdog [s] (기본 0.2)
  output_hz(float)       : 출력/rate-limit 기준 (기본 100)
  enable_deadman(bool)   : True 면 ~/enable(Bool) 눌러야 동작 (기본 False)
"""

import rclpy
import yaml
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from whatslab.safety import SafetyFilter, load_limits_from_urdf, tighten


class SafetyGateNode(Node):
    """~/command_raw 를 whatslab.safety.SafetyFilter 로 걸러 ~/command_safe 로 낸다."""

    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('limits_override', '')
        # timeout:=1 처럼 정수로 넘겨도 받도록 타입을 고정하지 않는다.
        self.declare_parameter('timeout', 0.2, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('output_hz', 100.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('enable_deadman', False)

        urdf = self.get_parameter('robot_description').value
        if not urdf:
            raise RuntimeError('robot_description(URDF) 필요 — limit 없음 = 안전 없음 (fail-fast)')
        limits = load_limits_from_urdf(urdf)
        ov = self.get_parameter('limits_override').value
        if ov:
            limits = tighten(limits, yaml.safe_load(ov) or {})

        hz = float(self.get_parameter('output_hz').value)
        if hz <= 0:
            raise ValueError(f'output_hz 는 0 보다 커야 한다: {hz}')
        self._filter = SafetyFilter(limits, dt=1.0 / hz)
        self._timeout = float(self.get_parameter('timeout').value)
        if self._timeout <= 0:
            raise ValueError(f'timeout 은 0 보다 커야 한다: {self._timeout}')
        self._deadman = bool(self.get_parameter('enable_deadman').value)
        if self._deadman:
            self._filter.set_enabled(False)  # 눌러야 동작

        self._desired = None
        self._last_stamp = None
        self._seeded = False

        self._pub = self.create_publisher(JointState, '~/command_safe', 10)
        self.create_subscription(JointState, '~/command_raw', self._on_cmd, 10)
        self.create_service(Trigger, '~/estop', self._on_estop)
        self.create_service(Trigger, '~/reset', self._on_reset)
        if self._deadman:
            self.create_subscription(Bool, '~/enable', self._on_enable, 10)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'safety_gate ready | {len(limits)} joints | timeout {self._timeout}s '
            f'| deadman {self._deadman}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: JointState):
        d = dict(zip(msg.name, msg.position))
        if not self._seeded:
            self._filter.seed(d)  # first-command ramp 기준
            self._seeded = True
        self._desired = d
        self._last_stamp = self._now()

    def _tick(self):
        stale = self._last_stamp is None or (self._now() - self._last_stamp) > self._timeout
        safe = self._filter.step(None if stale else self._desired)
        if not safe:
            return
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(safe.keys())
        m.position = list(safe.values())
        self._pub.publish(m)

    def _on_estop(self, _req, resp):
        self._filter.trip()
        resp.success = True
        resp.message = 'E-STOP 래칭 — /reset 전까지 hold'
        return resp

    def _on_reset(self, _req, resp):
        self._filter.reset()
        resp.success = True
        resp.message = 'reset — 재개(rate-limit 로 부드럽게)'
        return resp

    def _on_enable(self, msg: Bool):
        self._filter.set_enabled(bool(msg.data))


def main(args: list[str] | None = None) -> None:
    """safety_gate 노드를 띄운다."""
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyGateNode()  # 생성 실패(설정 오류 등)도 아래 finally 에서 shutdown 한다
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
        if rclpy.ok():  # SIGINT 핸들러가 이미 내렸으면 이중 shutdown 방지
            rclpy.shutdown()


if __name__ == '__main__':
    main()
