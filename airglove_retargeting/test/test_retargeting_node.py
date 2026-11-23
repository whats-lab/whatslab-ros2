import pytest
import rclpy

from airglove_msgs.msg import HandJoints
from airglove_retargeting.retargeting_node import RetargetingNode


class _FailingController:

    engine_name = 'geort:'
    joint_names = ['j0']
    last_error = None
    was_reset = False

    def compute(self, sample):
        raise RuntimeError('solver blew up')

    def reset(self):
        self.was_reset = True


@pytest.fixture
def node():
    rclpy.init()
    n = RetargetingNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_engine_error_is_logged_not_raised(node):
    ctrl = _FailingController()
    node._ctrl['right'] = ctrl
    msg = HandJoints()
    msg.valid = [True] * 26
    node._on_hand('right', msg)
    assert ctrl.was_reset


def test_left_glove_on_right_model_uses_right_urdf():
    rclpy.init(args=['--ros-args', '-p', 'hand_type:=left'])
    try:
        n = RetargetingNode()
        joints = n._all_joints['left']
        assert joints and set(n._ctrl['left'].joint_names) <= set(joints)
        n.destroy_node()
    finally:
        rclpy.shutdown()
