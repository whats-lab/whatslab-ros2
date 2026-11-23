import pytest
import rclpy

from airglove_msgs.msg import HandJoints
from airglove_retargeting.retargeting_node import RetargetingNode


class _FailingController:
    """엔진 오류(ValueError 가 아닌)를 내는 가짜 HandRetargetController."""

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
    assert ctrl.was_reset  # 다음 프레임은 깨끗한 엔진 상태에서 시작한다


def test_left_glove_on_right_model_uses_right_urdf():
    # geort:orca_right 는 왼손 입력도 오른손 로봇 관절을 내므로 {side} 는 right URDF 로 풀린다.
    rclpy.init(args=['--ros-args', '-p', 'hand_type:=left'])
    try:
        n = RetargetingNode()
        joints = n._all_joints['left']
        assert joints and set(n._ctrl['left'].joint_names) <= set(joints)
        n.destroy_node()
    finally:
        rclpy.shutdown()


class _DexController(_FailingController):
    engine_name = 'dex:'


def test_dex_skips_frames_with_invalid_joints(node):
    # dex 엔진은 xr_valid 를 보지 않으므로 원점으로 채운 관절을 풀지 않게 프레임을 건너뛴다.
    ctrl = _DexController()
    node._ctrl['right'] = ctrl
    msg = HandJoints()
    valid = [True] * 26
    valid[10] = False
    msg.valid = valid
    node._on_hand('right', msg)
    assert not ctrl.was_reset  # compute 를 부르지 않았다(불렀으면 실패 → reset)
