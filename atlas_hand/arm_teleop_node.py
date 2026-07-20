#!/usr/bin/env python3
"""팔+손 텔레옵 노드 — whatslab 프리미티브 직접 조립(GloveModel 프리셋 미사용).

whatslab 의 `GloveModel` 은 파이썬 SDK 사용자 편의를 위한 프리셋이다. ROS 노드는
더 명시적인 제어(컴포넌트별 추적 확인·부분 발행·에러 로깅)가 필요하므로, 프리셋
대신 공개 프리미티브를 직접 엮는다:

  · RobotModel(rig)               로봇/리그 기구학
  · RobotArmIK(robot)             정준 EE 목표 → 팔 관절각
  · ArmCalibration(reach, W)      머리-상대 손목 yaw 정렬 + reach 스케일
  · make_hand_controller(cfg,s)   손가락 → 로봇 손 관절각
  · Quest 컨트롤러 / AirGlove 리시버

배선(GloveModel 과 동일 규약): 출력 side = 글러브(손) side. 그 팔 목표 =
반대 손 컨트롤러 위치 + 글러브 손목 회전(머리-상대). IK/캘리브/리타게팅 수학은
프리미티브를 그대로 재사용해 whatslab 과의 divergence 를 막는다.
"""
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from whatslab.core.types import Pose
from whatslab.model import ArmCalibration, RobotArmIK
from whatslab.receiver.glove_human_hand import GloveHumanHandReceiver
from whatslab.receiver.quest_controller import QuestControllerReceiver
from whatslab.robot import RobotModel


class AtlasArmTeleopNode(Node):
    """반대손 컨트롤러(팔) + 글러브(손) → /joint_states + /arm/joint_trajectory."""

    def __init__(self):
        super().__init__('atlas_arm_teleop')
        self.declare_parameter('rig', 'rigs/nero_orca_right.yaml')
        self.declare_parameter('controller_side', 'left')   # 팔 위치를 주는 컨트롤러 손
        self.declare_parameter('hand_side', 'right')         # 글러브 낀 손 = 출력 로봇 side
        self.declare_parameter('quest_port', 9000)
        self.declare_parameter('glove_port', 4040)
        self.declare_parameter('publish_hz', 60.0)
        self.declare_parameter('arm_traj_time', 0.1)

        def _p(name):
            return self.get_parameter(name).value

        self._ctrl_side = str(_p('controller_side'))
        self._hand_side = str(_p('hand_side'))
        self._traj_time = float(_p('arm_traj_time'))

        # 프리미티브 조립 (hand config 는 rig 가 결정 — rig.hand.retarget)
        robot = RobotModel.from_yaml(_p('rig'))
        self._ik = RobotArmIK(robot) if robot.has_arm else None
        cfg = robot.rig.hand.retarget if robot.rig.hand is not None else None
        self._retarget = (robot.make_hand_controller(cfg, self._hand_side)
                          if robot.has_hand and cfg else None)
        self._calib = ArmCalibration(
            reach_max=robot.rig.solver.reach_max,
            input_reach=robot.rig.calibration.input_reach) if self._ik else None

        self._arm_names = list(self._ik.joint_names) if self._ik else []
        hand_names = list(self._retarget.joint_names) if self._retarget else []

        # 리시버: 팔 = Quest 컨트롤러, 손 = AirGlove
        self._arm_rx = QuestControllerReceiver(quest_port=int(_p('quest_port')))
        self._hand_rx = GloveHumanHandReceiver(glove_port=int(_p('glove_port')))
        self._arm_rx.start()
        self._hand_rx.start()

        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._arm_pub = self.create_publisher(JointTrajectory, '/arm/joint_trajectory', 10)
        self.create_service(Trigger, '~/calibrate', self._on_calibrate)
        self.create_timer(1.0 / float(_p('publish_hz')), self._publish)
        self.get_logger().info(
            f'atlas_arm_teleop ready | ctrl={self._ctrl_side} hand={self._hand_side} '
            f'| arm {len(self._arm_names)}j + hand {len(hand_names)}j')

    @staticmethod
    def _head_relative(quat, hmd_quat):
        """손목 quat 을 머리 yaw 기준 상대로(Rz(-h)·G). 몸/의자 회전엔 불변. whatslab
        GloveModel 과 동일 규약 — hmd 없으면 원본 그대로."""
        if hmd_quat is None:
            return quat
        G = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        H = Rotation.from_quat(np.asarray(hmd_quat, dtype=float)).as_matrix()
        h = float(np.arctan2(H[1, 0], H[0, 0]))
        c, s = np.cos(-h), np.sin(-h)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return Rotation.from_matrix(Rz @ G).as_quat()

    def _arm_pose(self):
        """현재 팔 목표 Pose(컨트롤러 위치 + 머리-상대 글러브 손목) 또는 미추적 시 None."""
        arm_s = self._arm_rx.get(self._ctrl_side)
        hand = self._hand_rx.get(self._hand_side).hand
        ctrl = arm_s.controller
        if ctrl is None or hand is None or hand.wrist is None or not hand.tracked:
            return None
        hmd_q = arm_s.hmd.quat if arm_s.hmd is not None else None
        return Pose(ctrl.pos, self._head_relative(hand.wrist.quat, hmd_q))

    def _publish(self):
        if not rclpy.ok():                     # 종료(teardown) 중이면 발행 안 함
            return
        q = {}
        pose = self._arm_pose()
        if pose is not None and self._ik is not None:
            data = self._calib.apply({'arm_pose': pose})   # yaw 정렬 + reach 스케일 → 4x4
            if data.get('arm_target') is not None:
                q_arm = self._ik.solve(data['arm_target'])
                q.update(zip(self._arm_names, (float(v) for v in q_arm)))

        hand_s = self._hand_rx.get(self._hand_side)
        if self._retarget is not None and hand_s.hand is not None and hand_s.hand.tracked:
            cmd = self._retarget.compute(hand_s)
            q.update(zip(cmd.joint_names, (float(v) for v in cmd.joint_angles)))

        if not q:                                          # 아무 것도 추적 안 됨 → 무발행
            return
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = list(q.keys())
        js.position = [q[n] for n in js.name]
        self._js_pub.publish(js)

        if any(n in q for n in self._arm_names):           # 팔 해가 있을 때만 궤적 발행
            jt = JointTrajectory()
            jt.header.stamp = now
            jt.joint_names = self._arm_names
            pt = JointTrajectoryPoint()
            pt.positions = [q.get(n, 0.0) for n in self._arm_names]
            pt.time_from_start = Duration(
                sec=int(self._traj_time),
                nanosec=int((self._traj_time % 1.0) * 1e9))
            jt.points = [pt]
            self._arm_pub.publish(jt)

    def _on_calibrate(self, _req, resp):
        pose = self._arm_pose()
        ok = bool(self._calib and pose is not None and self._calib.capture({'arm_pose': pose}))
        resp.success = ok
        resp.message = '손목 yaw 캘리브 완료' if ok else '캘리브 실패 (팔/글러브 신호 없음)'
        return resp

    def destroy_node(self):
        self._arm_rx.stop()
        self._hand_rx.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AtlasArmTeleopNode()
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
