import numpy as np

from airglove_msgs.msg import HandJoints
from airglove_retargeting.retargeting_node import hand_sample


def test_hand_joints_to_keypoints():
    msg = HandJoints()
    data = np.arange(208, dtype=np.float32) / 100.0
    msg.joints = data.tolist()
    valid = [True] * 26
    valid[4] = False
    msg.valid = valid
    s = hand_sample(msg)
    assert s.tracked and s.hand.tracked
    assert s.hand.xr_joints.shape == (26, 8)
    assert np.allclose(s.hand.xr_joints.reshape(-1), data)
    assert not s.hand.xr_valid[4] and s.hand.xr_valid.sum() == 25


def test_all_invalid_joints_are_untracked():
    msg = HandJoints()
    msg.joints = [0.0] * 208
    msg.valid = [False] * 26
    s = hand_sample(msg)
    assert not s.tracked and not s.hand.tracked
