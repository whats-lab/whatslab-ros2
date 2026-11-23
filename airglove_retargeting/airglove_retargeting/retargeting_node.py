#!/usr/bin/env python3
"""사람 손(OpenXR 26관절) → 로봇 손 관절각 노드 (whatslab HandRetargetController).

  /airglove/{side}/joints (airglove_msgs/HandJoints)
    ─► HandRetargetController ─► sensor_msgs/JointState

엔진은 `retarget` 파라미터로 고른다:
  geort:<설정>  GeoRT 소프트 프롬프트 (numpy, 기본 geort:orca_right) — 모델은 SDK 에 동봉
  dex:<손>      dex-retargeting DexPilot (URDF 만 있으면 됨) — 'whatslab-sdk[hand]' 필요

입력은 airglove_driver 가 발행하지만, 같은 HandJoints 를 내는 어떤 소스(Quest·Vision Pro
핸드트래킹 브리지 등)도 쓸 수 있다. 글러브(UDP 4040)는 열지 않으므로 다른 PC 에서 돌려도 된다.

GeoRT 모델은 한쪽 손(현재 오른손)용이라 왼손 입력은 좌우 반전해서 같은 로봇 손 관절로 푼다.
양손(hand_type=both)이면 두 손이 같은 관절 이름을 내므로 `/{side}_hand/joint_states` 로 나눠
발행한다.
"""

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
from whatslab.solvers.hand.controller import DEX_PREFIX, HandRetargetController

DEFAULT_RETARGET = 'geort:orca_right'
DEFAULT_URDF = 'orca_hand/urdf/{side}.urdf'
SIDES = ('left', 'right')

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
)


def hand_sample(msg: HandJoints) -> InputSample:
    """HandJoints → whatslab InputSample(hand.xr_joints 26×8, xr_valid).

    유효 관절이 하나도 없으면 추적 끊김으로 보내 컨트롤러가 직전 명령을 유지하게 한다.
    """
    kp = np.asarray(msg.joints, dtype=np.float64).reshape(
        HandJoints.JOINT_COUNT, HandJoints.JOINT_FIELDS
    )
    valid = np.asarray(msg.valid, dtype=bool)
    tracked = bool(valid.any())
    hand = HandPose(xr_joints=kp, xr_valid=valid, tracked=tracked)
    return InputSample(hand=hand, tracked=tracked)


def urdf_nonfixed_joints(urdf_path: str) -> list:
    """URDF 의 non-fixed 관절 이름 전체(순서 보존).

    robot_state_publisher 는 모든 관절 값이 있어야 갱신하므로, 리타게팅 대상이 아닌 관절
    (예: orca 손목)도 0 으로 채워 발행한다.
    """
    root = ET.parse(urdf_path).getroot()
    return [j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed']


def resolve_urdf(path: str) -> str:
    """절대경로면 그대로, 아니면 dexhand_description 자산 루트 기준. 없으면 ''."""
    if not path:
        return ''
    if os.path.isabs(path):
        return path
    share = description_share()
    return os.path.join(share, path) if share else ''


class RetargetingNode(Node):
    """HandJoints 를 HandRetargetController 로 풀어 로봇 손 JointState 로 발행한다."""

    def __init__(self):
        super().__init__('airglove_retargeting')
        self.declare_parameter('hand_type', 'right')  # 'left' | 'right' | 'both'
        self.declare_parameter('retarget', DEFAULT_RETARGET)  # geort:<설정> | dex:<손>
        # 0-fill 발행용 URDF (dexhand_description 기준). `{side}` 는 left/right 로 바뀐다.
        self.declare_parameter('robot_urdf', DEFAULT_URDF)
        self.declare_parameter('input_namespace', '/airglove')  # {ns}/{side}/joints 를 구독

        def _p(name):
            return self.get_parameter(name).value

        hand_type = str(_p('hand_type')).lower()
        if hand_type not in ('left', 'right', 'both'):
            raise ValueError(f'hand_type 은 left|right|both: {hand_type!r}')
        self._sides = list(SIDES) if hand_type == 'both' else [hand_type]
        cfg = str(_p('retarget'))
        # dex 엔진도 노드와 같은 자산 루트(ament 우선)에서 URDF 를 찾게 한다.
        root = description_share() or None
        self._ctrl = {s: HandRetargetController(s, cfg, urdf_root=root) for s in self._sides}
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
        """0-fill 할 URDF 관절 목록.

        `{side}` 는 사람 손 쪽을 먼저, 리타게팅 관절이 없으면 반대쪽을 쓴다 — 한 손 모델
        (geort:orca_right)은 왼손 입력도 오른손 로봇 관절로 풀기 때문이다. 맞는 URDF 가 없으면
        [] — 리타게팅 관절만 발행한다.
        """
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
                    if self._ctrl[side].engine_name == DEX_PREFIX:  # dex 는 같은 쪽이어야 정상
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
        """엔진 상태를 비운다.

        SDK 는 ValueError 때만 스스로 리셋해, 다른 오류 뒤에는 warm start 가 오염된 채 남는다.
        """
        try:
            self._ctrl[side].reset()
        except Exception:  # 콜백 경계 — 리셋 실패는 기록만, 다음 오류 때 다시 시도된다
            self.get_logger().error(
                f'[{side}] 엔진 리셋 실패\n{traceback.format_exc()}', throttle_duration_sec=1.0
            )

    def _on_hand(self, side: str, msg: HandJoints) -> None:
        ctrl = self._ctrl[side]
        # dex 엔진(SDK)은 xr_valid 를 보지 않아, 원점으로 채운 무효 관절을 진짜 손끝으로 푼다.
        # 전부 무효(추적 끊김)는 compute 로 보내 엔진을 리셋하게 둔다.
        if ctrl.engine_name == DEX_PREFIX and any(msg.valid) and not all(msg.valid):
            self.get_logger().warn(
                f'[{side}] 무효 관절이 있어 건너뜀 — 로봇 손은 직전 값 유지 (dex)',
                throttle_duration_sec=1.0,
            )
            return
        sample = hand_sample(msg)
        try:
            cmd = ctrl.compute(sample)
        except Exception:  # 콜백 경계 — 한 프레임의 엔진 오류(nlopt 등)로 노드가 죽지 않게
            # rclpy 로거는 exc_info 를 받지 않는다 — 트레이스백을 메시지에 넣는다.
            self.get_logger().error(
                f'[{side}] 리타게팅 실패\n{traceback.format_exc()}', throttle_duration_sec=1.0
            )
            self._reset(side)
            return
        if not sample.tracked:  # 추적 끊김 — 엔진 상태만 비우고 발행하지 않는다(watchdog 몫)
            return
        if ctrl.last_error is not None:  # 이번 프레임을 못 풀어 직전 값 유지
            reason = getattr(ctrl.last_error, 'cause', ctrl.last_error)
            self.get_logger().warn(f'[{side}] {reason}', throttle_duration_sec=1.0)
            return
        driven = dict(zip(cmd.joint_names, np.asarray(cmd.joint_angles, dtype=float).tolist()))
        names = self._all_joints[side] or list(driven)
        js = JointState()
        # 드라이버가 다른 PC 일 수 있어 그쪽 시계 대신 이 노드 시계로 찍는다(TF 시각 어긋남 방지).
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = [driven.get(n, 0.0) for n in names]
        self._pub[side].publish(js)


def main(args: list[str] | None = None) -> None:
    """airglove_retargeting 노드를 띄운다."""
    rclpy.init(args=args)
    node = None
    try:
        node = RetargetingNode()  # 생성 실패(설정 오류 등)도 아래 finally 에서 shutdown 한다
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
