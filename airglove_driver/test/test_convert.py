from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time

from airglove_driver import convert
from airglove_msgs.msg import Alarm, HandJoints


def _frame(seq=7):
    rng = np.random.default_rng(1)
    valid = np.ones(26, bool)
    valid[2] = False
    return SimpleNamespace(
        orientations=rng.normal(size=(26, 4)).astype(np.float32),
        positions=rng.normal(size=(26, 3)).astype(np.float32),
        radii=np.full(26, 0.01, np.float32),
        valid=valid,
        seq=seq,
        age_s=0.0,
    )


def test_hand_msg_keeps_spine_layout():
    f = _frame()
    msg = convert.hand_msg(f, 'right', 1, Time(sec=3))
    assert msg.side == HandJoints.SIDE_RIGHT and msg.pair == 1 and msg.seq == 7
    assert msg.header.frame_id == 'airglove_right_wrist'
    j = np.asarray(msg.joints, np.float32).reshape(26, 8)
    assert np.allclose(j[:, 0:4], f.orientations)
    assert np.allclose(j[:, 4:7], f.positions)
    assert np.allclose(j[:, 7], 0.01)
    assert list(msg.valid) == list(f.valid)


def test_wrist_msg_is_raw():
    w = SimpleNamespace(orientation=np.array([0.1, 0.2, 0.3, -0.4], np.float32), seq=2, age_s=0.0)
    msg = convert.wrist_msg(w, 'left', 1, Time())
    assert np.allclose(msg.raw, [0.1, 0.2, 0.3, -0.4]) and msg.side == 0


def test_status_without_spine():
    msg = convert.status_msg(None, None, 1, Time())
    assert not msg.left_connected and msg.spine_protocol == 0 and msg.spine_age == -1.0
    st = SimpleNamespace(left_connected=True, right_connected=False, age_s=0.5)
    msg = convert.status_msg(st, (3, 0.5), 1, Time())
    assert msg.left_connected and not msg.right_connected and msg.spine_protocol == 3


def test_stale_status_reads_as_disconnected():
    st = SimpleNamespace(left_connected=True, right_connected=True, age_s=10.0)
    msg = convert.status_msg(st, (3, 10.0), 1, Time())
    assert not msg.left_connected and not msg.right_connected
    assert msg.spine_protocol == 0 and msg.spine_age == 10.0


def test_alarm_and_strengths():
    a = convert.alarm_msg(SimpleNamespace(code='RESTART', side=1), 1, Time())
    assert a.code == 'RESTART' and a.side == Alarm.SIDE_RIGHT
    assert convert.strengths([5, 0, 2, 1, 3]) == [3, 0, 2, 1, 3]


def test_new_haptics_result_by_arrival_window():
    assert not convert.track_result(5.0, 10.0, 10.0, None)[0]
    is_new, latest = convert.track_result(0.1, 10.0, 10.0, None)
    assert is_new and abs(latest - 9.9) < 1e-9
    assert not convert.track_result(0.103, 10.0, 10.003, 9.9)[0]
    assert convert.track_result(0.003, 10.0083, 10.0083, 9.9993)[0]
