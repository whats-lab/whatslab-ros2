#!/usr/bin/env python3

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

    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('limits_override', '')
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
            self._filter.set_enabled(False)

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
            self._filter.seed(d)
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
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyGateNode()
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
