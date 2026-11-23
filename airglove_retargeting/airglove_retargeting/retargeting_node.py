#!/usr/bin/env python3

import os
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from airglove_msgs.msg import HandJoints
from airglove_retargeting.assets import description_share
from whatslab.core.types import HandPose, InputSample
from whatslab.retarget.retargeter import DEX_PREFIX, HandRetargeter

DEFAULT_RETARGET = 'geort:orca_right'
DEFAULT_URDF = 'orca_hand/urdf/{side}.urdf'
SIDES = ('left', 'right')

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
)


def hand_sample(msg: HandJoints) -> InputSample:
    kp = np.asarray(msg.joints, dtype=np.float64).reshape(
        HandJoints.JOINT_COUNT, HandJoints.JOINT_FIELDS
    )
    valid = np.asarray(msg.valid, dtype=bool)
    tracked = bool(valid.any())
    hand = HandPose(xr_joints=kp, xr_valid=valid, tracked=tracked)
    return InputSample(hand=hand, tracked=tracked)


def urdf_nonfixed_joints(urdf_path: str) -> list:
    root = ET.parse(urdf_path).getroot()
    return [j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed']


def resolve_urdf(path: str) -> str:
    if not path:
        return ''
    if os.path.isabs(path):
        return path
    share = description_share()
    return os.path.join(share, path) if share else ''


class RetargetingNode(Node):

    def __init__(self):
        super().__init__('airglove_retargeting')
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('retarget', DEFAULT_RETARGET)
        self.declare_parameter('robot_urdf', DEFAULT_URDF)
        self.declare_parameter('input_namespace', '/airglove')

        def _p(name):
            return self.get_parameter(name).value

        hand_type = str(_p('hand_type')).lower()
        if hand_type not in ('left', 'right', 'both'):
            raise ValueError(f'hand_type 은 left|right|both: {hand_type!r}')
        self._sides = list(SIDES) if hand_type == 'both' else [hand_type]
        cfg = str(_p('retarget'))
        root = description_share() or None
        self._ctrl = {s: HandRetargeter(cfg, side=s, urdf_root=root) for s in self._sides}
        self._all_joints = {s: self._urdf_joints(str(_p('robot_urdf')), s) for s in self._sides}

        ns = str(_p('input_namespace')).rstrip('/')
        topic = (
            (lambda s: f'/{s}_hand/joint_states')
            if hand_type == 'both'
            else (lambda s: '/joint_states')
        )
        self._pub = {s: self.create_publisher(JointState, topic(s), 10) for s in self._sides}
        for s in self._sides:
            self.create_subscription(
                HandJoints, f'{ns}/{s}/joints', lambda m, s=s: self._on_hand(s, m), SENSOR_QOS
            )
        self.get_logger().info(
            f'airglove_retargeting ready | hands={self._sides} | retarget={cfg} '
            f'| {len(self._ctrl[self._sides[0]].joint_names)} joints'
        )

    def _urdf_joints(self, pattern: str, side: str) -> list:
        driven = self._ctrl[side].joint_names
        other = 'right' if side == 'left' else 'left'
        robot_sides = (side, other) if '{side}' in pattern else (side,)
        for robot_side in robot_sides:
            urdf = resolve_urdf(pattern.replace('{side}', robot_side))
            if not urdf or not os.path.exists(urdf):
                continue
            joints = urdf_nonfixed_joints(urdf)
            if all(n in joints for n in driven):
                if robot_side != side:
                    note = f'[{side}] 로봇 손은 {robot_side} — {urdf}'
                    if self._ctrl[side].engine_name == DEX_PREFIX:
                        self.get_logger().warn(f'{note} (dex 설정의 관절 이름 확인)')
                    else:
                        self.get_logger().info(note)
                return joints
        self.get_logger().warn(
            f'[{side}] {pattern} 에 리타게팅 관절 {driven[:3]}… 을 가진 URDF 가 없다 — '
            '리타게팅 관절만 발행 (robot_urdf 확인)'
        )
        return []

    def _reset(self, side: str) -> None:
        try:
            self._ctrl[side].reset()
        except Exception:
            self.get_logger().error(
                f'[{side}] 엔진 리셋 실패\n{traceback.format_exc()}', throttle_duration_sec=1.0
            )

    def _on_hand(self, side: str, msg: HandJoints) -> None:
        ctrl = self._ctrl[side]
        sample = hand_sample(msg)
        try:
            cmd = ctrl.compute(sample)
        except Exception:
            self.get_logger().error(
                f'[{side}] 리타게팅 실패\n{traceback.format_exc()}', throttle_duration_sec=1.0
            )
            self._reset(side)
            return
        if not sample.tracked:
            return
        if ctrl.last_error is not None:
            reason = getattr(ctrl.last_error, 'cause', ctrl.last_error)
            self.get_logger().warn(f'[{side}] {reason}', throttle_duration_sec=1.0)
            return
        driven = dict(zip(cmd.joint_names, np.asarray(cmd.joint_angles, dtype=float).tolist()))
        names = self._all_joints[side] or list(driven)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = [driven.get(n, 0.0) for n in names]
        self._pub[side].publish(js)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RetargetingNode()
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
