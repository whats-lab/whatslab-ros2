#!/usr/bin/env python3
"""
atlas_hand — whatslab 을 감싼 단일 ROS 2 래퍼 노드.

  글러브/Quest OSC 입력(whatslab receiver) ─►
  손 리타게팅(whatslab HandRetargeter) ─► /joint_states + 사람 손 TF

계산은 전부 whatslab. 이 노드는 "데이터 수신 → 리타게팅 → 발행" 한 과정을 ROS 로
감싼다. 발행은 executor 스레드의 타이머에서만 한다 — 리시버 스레드에서 rclpy 를
호출하면 eigenpy/nlopt compute 와 executor 가 경쟁해 크래시하므로 폴링으로 분리했다.
"""

import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Int32MultiArray

from whatslab.core.types import HUMAN_HAND
from whatslab.receiver.glove_human_hand import AGA_FINGER_COUNT, GloveHumanHandReceiver
from whatslab.receiver.quest_hand import QuestHandReceiver
from whatslab.teleop.hand import HandRetargeter
from whatslab.teleop.hand.hand_configs import CONFIG_REGISTRY

from atlas_hand.config import (
    TOPIC_LEFT_HAPTIC,
    TOPIC_LEFT_HAPTIC_OFF,
    TOPIC_RIGHT_HAPTIC,
    TOPIC_RIGHT_HAPTIC_OFF,
)

# 사람 손 골격 이름(정규 23-포인트) — whatslab 단일 출처. TF 프레임 라벨용.
_HUMAN_JOINT_NAMES = [j.name for j in HUMAN_HAND]


def _urdf_nonfixed_joints(urdf_path: str) -> list:
    """URDF 의 non-fixed 관절 이름 전체(순서 보존). rviz/robot_state_publisher 는
    모든 관절 값이 있어야 갱신하므로, 리타게팅 대상이 아닌 관절(예: orca 손목)도
    0 으로 채워 발행하기 위한 전체 목록."""
    root = ET.parse(urdf_path).getroot()
    return [j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed']


class AtlasHandNode(Node):
    """OSC 수신 → 리타게팅 → /joint_states + TF 를 담당하는 단일 래퍼 노드."""

    def __init__(self):
        super().__init__('atlas_hand')
        self.declare_parameter('input_source', 'atlas')      # 'atlas' | 'meta_quest'
        self.declare_parameter('hand_type',    'left')       # 'left' | 'right' | 'both'
        self.declare_parameter('robot_config', 'base_hand')
        self.declare_parameter('server_port',  4040)         # AirGlove OSC
        self.declare_parameter('osc_port',     9000)         # Quest OSC
        self.declare_parameter('publish_hz',   60.0)         # timer 폴링/발행 주기

        source_type = self.get_parameter('input_source').value
        hand_type   = self.get_parameter('hand_type').value.lower()
        config_name = self.get_parameter('robot_config').value

        self._sides = ['left', 'right'] if hand_type == 'both' else [hand_type]
        models_root = get_package_share_directory('dexhand_description')

        # 손별 리타게터 + TF 부모 프레임(손목 링크). URDF/메쉬는 이 패키지의 models
        # 서브모듈을 쓴다(whatslab 은 코드만 제공).
        self._retargeters = {}
        self._tf_parent = {}
        self._all_joints = {}                  # side → URDF 전체 non-fixed 관절 (0-fill 발행용)
        for side in self._sides:
            self._retargeters[side] = HandRetargeter(side, config_name, urdf_root=models_root)
            cfg = CONFIG_REGISTRY[config_name](urdf_root=models_root)
            self._tf_parent[side] = cfg.get_wrist_link_name(side)
            self._all_joints[side] = _urdf_nonfixed_joints(cfg._get_urdf_path(side))

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self._tf  = tf2_ros.TransformBroadcaster(self)

        # whatslab receiver — 새 프레임 도착 시 on_update(side) 콜백에서 처리(폴링 없음)
        self._rx = self._make_receiver(source_type)
        self._rx.start()
        # 발행은 executor 스레드의 timer 에서만 — 리시버 스레드에서 rclpy 를 만지지 않는다.
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value), self._tick)
        self.get_logger().info(
            f'atlas_hand ready | source={source_type} | hands={self._sides} '
            f'| config={config_name}')

        # 햅틱 구독 (AirGlove 만 지원 — Quest 는 send_haptic 없음 → 무시)
        haptic_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Int32MultiArray, TOPIC_LEFT_HAPTIC,
                                 lambda m: self._haptic('left', m),  haptic_qos)
        self.create_subscription(Int32MultiArray, TOPIC_RIGHT_HAPTIC,
                                 lambda m: self._haptic('right', m), haptic_qos)
        self.create_subscription(Empty, TOPIC_LEFT_HAPTIC_OFF,
                                 lambda _: self._haptic_off('left'),  haptic_qos)
        self.create_subscription(Empty, TOPIC_RIGHT_HAPTIC_OFF,
                                 lambda _: self._haptic_off('right'), haptic_qos)

    def _make_receiver(self, source_type: str):
        # on_update 푸시 콜백을 쓰지 않는다 — 그 콜백은 OSC 리시버 스레드에서 돌아
        # 거기서 rclpy publish/TF 를 호출하면 executor 와 경쟁해 segfault 가 난다.
        # 대신 리시버는 최신 프레임만 저장하고, executor 스레드의 timer 가 폴링한다.
        if source_type in ('atlas', 'airglove'):
            return GloveHumanHandReceiver(
                glove_port=int(self.get_parameter('server_port').value))
        if source_type in ('meta_quest', 'quest'):
            return QuestHandReceiver(
                quest_port=int(self.get_parameter('osc_port').value))
        raise ValueError(
            f"알 수 없는 input_source: {source_type!r} ('atlas' | 'meta_quest')")

    def _tick(self) -> None:
        """executor 스레드(timer)에서 최신 프레임 폴링 → 리타게팅 → 발행."""
        if not rclpy.ok():                     # 종료(teardown) 중이면 발행 안 함
            return
        for side in self._sides:
            rt = self._retargeters.get(side)
            if rt is None:
                continue
            sample = self._rx.get(side)
            if sample.hand is None or not sample.hand.tracked:
                continue
            try:
                qpos = rt.compute(sample.hand.to_sensor_array())
            except Exception as e:
                self.get_logger().error(f'[{side}] 리타게팅 오류: {e}',
                                        throttle_duration_sec=1.0)
                continue
            self._publish(side, rt, qpos)

    def _publish(self, side: str, rt: HandRetargeter, qpos: np.ndarray) -> None:
        now = self.get_clock().now().to_msg()

        # URDF 전체 non-fixed 관절을 발행 — 리타게팅 값이 있으면 그 값, 없으면 0.
        # orca 손목(TopTower→Carpals)처럼 리타게팅 대상이 아닌 관절이 빠지면
        # robot_state_publisher 가 갱신을 못 해 rviz 가 멈춘다 → 0 으로 채운다.
        driven = dict(zip(rt.joint_names, np.asarray(qpos, dtype=float).tolist()))
        names = self._all_joints[side]
        js = JointState()
        js.header.stamp = now
        js.name         = names
        js.position     = [driven.get(n, 0.0) for n in names]
        self._pub.publish(js)

        # 사람 손과 로봇 손은 링크 길이가 달라 중간 관절은 로봇 링크에 얹히지 않는다.
        # IK 가 실제로 위치를 맞추는 손가락 팁(+wrist)만 wrist 링크 기준으로 발행.
        transforms = []
        parent = self._tf_parent[side]
        for i in [0, *rt.tip_human_indices]:
            if i >= len(_HUMAN_JOINT_NAMES):
                continue
            p = rt.last_human_positions[i]
            t = TransformStamped()
            t.header.stamp            = now
            t.header.frame_id         = parent
            t.child_frame_id          = f'human_{side}_{_HUMAN_JOINT_NAMES[i]}'
            t.transform.translation.x = float(p[0])
            t.transform.translation.y = float(p[1])
            t.transform.translation.z = float(p[2])
            t.transform.rotation.w    = 1.0
            transforms.append(t)
        self._tf.sendTransform(transforms)

    def _haptic(self, side: str, msg: Int32MultiArray) -> None:
        if len(msg.data) < AGA_FINGER_COUNT:
            self.get_logger().warn(
                f'[haptic] {side} 데이터 부족 ({len(msg.data)}/{AGA_FINGER_COUNT})')
            return
        self._send_haptic(side, list(msg.data))

    def _haptic_off(self, side: str) -> None:
        self._send_haptic(side, [0] * AGA_FINGER_COUNT)

    def _send_haptic(self, side: str, values) -> None:
        fn = getattr(self._rx, 'send_haptic', None)
        if fn is not None:
            fn(side, values)

    def destroy_node(self) -> None:
        self._rx.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AtlasHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():                # SIGINT 핸들러가 이미 내렸으면 이중 shutdown 방지
            rclpy.shutdown()


if __name__ == '__main__':
    main()
