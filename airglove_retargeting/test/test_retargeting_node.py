import numpy as np
import pytest
import rclpy

from airglove_msgs.msg import HandJoints
from airglove_retargeting.retargeting_node import RetargetingNode

CHAINS = (
    (-0.025, (0.0, 0.040, 0.035, 0.028)),
    (-0.020, (0.0, 0.070, 0.045, 0.025, 0.020)),
    (0.0, (0.0, 0.070, 0.050, 0.030, 0.022)),
    (0.018, (0.0, 0.065, 0.045, 0.028, 0.020)),
    (0.034, (0.0, 0.060, 0.035, 0.020, 0.018)),
)


def open_hand_joints() -> np.ndarray:
    xr = np.zeros((26, 8))
    xr[:, 3] = 1.0
    xr[:, 7] = 0.01
    i = 2
    for x, steps in CHAINS:
        z = 0.0
        for step in steps:
            z -= step
            xr[i, 4:7] = (x, 0.0, z)
            i += 1
    xr[0, 4:7] = (0.0, 0.0, -0.035)
    return xr


class _Recorder:

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _msg(joints: np.ndarray, valid: bool = True) -> HandJoints:
    msg = HandJoints()
    msg.joints = joints.astype(np.float32).reshape(-1).tolist()
    msg.valid = [valid] * 26
    return msg


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.parametrize('retarget', ['geort:orca_right', 'dex:dg5f_s'])
def test_real_retargeter_publishes_joint_state(ros, retarget):
    rclpy.shutdown()
    rclpy.init(args=['--ros-args', '-p', f'retarget:={retarget}',
                     '-p', 'robot_urdf:=tesollo_dg5f_s' if retarget.startswith('dex') else
                     'robot_urdf:=orca_hand_v2'])
    node = RetargetingNode()
    rec = _Recorder()
    node._pub['right'] = rec
    node._on_hand('right', _msg(open_hand_joints()))
    assert node._ctrl['right'].last_failure is None
    assert len(rec.msgs) == 1
    js = rec.msgs[0]
    assert set(node._ctrl['right'].joint_names) <= set(js.name)
    assert np.all(np.isfinite(js.position))
    node.destroy_node()


def test_unsolvable_input_logs_failure_and_does_not_publish(ros):
    node = RetargetingNode()
    rec = _Recorder()
    node._pub['right'] = rec
    node._on_hand('right', _msg(np.zeros((26, 8))))
    assert node._ctrl['right'].last_failure is not None
    assert rec.msgs == []
    node.destroy_node()


def test_untracked_input_does_not_publish(ros):
    node = RetargetingNode()
    rec = _Recorder()
    node._pub['right'] = rec
    node._on_hand('right', _msg(open_hand_joints(), valid=False))
    assert rec.msgs == []
    node.destroy_node()


def test_left_glove_on_right_model_uses_right_urdf():
    rclpy.init(args=['--ros-args', '-p', 'hand_type:=left'])
    n = RetargetingNode()
    joints = n._all_joints['left']
    assert joints and set(n._ctrl['left'].joint_names) <= set(joints)
    n.destroy_node()
    rclpy.shutdown()
