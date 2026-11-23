import time

import pytest
import rclpy

from airglove_driver.driver_node import ORPHAN_TICKS, RESEND_S, AirGloveDriver
from airglove_msgs.msg import Haptics


class _FailingClient:
    """호출이 늘 실패하는 가짜 airglove_client.Client (AirGloveError 는 RuntimeError)."""

    def __init__(self):
        self.closed = False

    @staticmethod
    def version():
        return 'fake'

    def set_haptics(self, side, strengths):
        raise RuntimeError('glove busy')

    def hand(self, side):
        raise RuntimeError('native error')

    def close(self):
        self.closed = True


@pytest.fixture
def driver():
    rclpy.init(args=['--ros-args', '-p', 'publish_hz:=120'])  # 정수로 넘겨도 받는다
    client = _FailingClient()
    node = AirGloveDriver(client_factory=lambda *_: client)
    yield node, client
    rclpy.shutdown()


def test_haptics_failure_is_logged_not_raised(driver):
    node, _ = driver
    node._on_haptics('left', Haptics(strength=[1] * 5))


def test_destroy_closes_client_even_if_haptics_off_fails(driver):
    node, client = driver
    node.destroy_node()
    assert client.closed


def test_client_error_in_poll_is_logged_not_raised(driver):
    node, _ = driver
    node._poll()  # 왼손·오른손·알람이 각각 실패해도 예외가 새지 않는다


def test_haptics_keyboard_rejects_unknown_side():
    from airglove_driver.haptics_keyboard import HapticsKeyboard

    rclpy.init(args=['--ros-args', '-p', 'side:=l'])
    try:
        with pytest.raises(ValueError):
            HapticsKeyboard()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('param', ['publish_hz:=0', 'haptics_timeout:=-1'])
def test_bad_rates_are_rejected_before_opening_client(param):
    opened = []
    rclpy.init(args=['--ros-args', '-p', param])
    try:
        with pytest.raises(ValueError):
            AirGloveDriver(client_factory=lambda *_: opened.append(1))
    finally:
        rclpy.shutdown()
    assert not opened


class _RecordingClient:
    """보낸 햅틱을 기록하고, 정해 둔 햅틱 결과를 돌려주는 가짜 클라이언트."""

    def __init__(self):
        self.sent = []
        self.result = None  # (success, 도착 monotonic) — age 는 실제 클라이언트처럼 계속 는다

    @staticmethod
    def version():
        return 'fake'

    def set_haptics(self, side, strengths):
        self.sent.append((side, tuple(strengths)))

    def hand(self, side):
        return None

    def wrist(self, side):
        return None

    def haptics_result(self, side):
        if self.result is None or side != 0:
            return None
        success, arrival = self.result
        return success, time.monotonic() - arrival

    def alarms(self):
        return []

    def close(self):
        pass


@pytest.fixture
def recording():
    rclpy.init()
    client = _RecordingClient()
    node = AirGloveDriver(client_factory=lambda *_: client)
    yield node, client
    rclpy.shutdown()


def _vibrate_then_stop(node, client, side='left'):
    node._on_haptics(side, Haptics(strength=[3] * 5))
    node._on_haptics(side, Haptics(strength=[0] * 5))
    client.sent.clear()


def test_stop_is_confirmed_only_after_a_second_send(recording):
    node, client = recording
    _vibrate_then_stop(node, client)
    client.result = (True, time.monotonic())  # 앞 명령(3)의 늦은 성공일 수 있다
    node._poll()
    assert node._stop_deadline['left'] is not None
    node._sent_at['left'] -= RESEND_S
    node._poll()  # 확인용 두 번째 끄기
    assert client.sent == [(0, (0,) * 5)]
    client.result = (True, time.monotonic())  # 두 번째 전송 뒤 도착한 성공 — 확정
    node._poll()
    assert node._stop_deadline['left'] is None
    node._sent_at['left'] -= RESEND_S
    node._poll()
    assert len(client.sent) == 1


def test_unconfirmed_stop_is_resent(recording):
    # 끄기 패킷이 사라져 결과가 안 와도(UDP) 창 안에서는 계속 보낸다.
    node, client = recording
    _vibrate_then_stop(node, client)
    for _ in range(3):
        node._sent_at['left'] -= RESEND_S
        node._poll()
    assert client.sent == [(0, (0,) * 5)] * 3


def test_stop_retry_gives_up_after_window(recording):
    # 장갑이 끊겨 끄기가 계속 확인되지 않으면 창이 끝난 뒤 그만 보낸다.
    node, client = recording
    _vibrate_then_stop(node, client)
    node._stop_deadline['left'] = time.monotonic() - 1.0
    node._sent_at['left'] -= RESEND_S
    node._poll()
    assert client.sent == [] and node._stop_deadline['left'] is None


def test_repeated_zeros_do_not_restart_the_stop(recording):
    node, client = recording
    _vibrate_then_stop(node, client)
    node._sent_at['left'] -= RESEND_S
    node._on_haptics('left', Haptics(strength=[0] * 5))  # 발행자가 0 을 계속 보낸다
    assert client.sent == [] and node._stop_sends['left'] == 1


def test_identical_commands_are_rate_limited(recording):
    node, client = recording
    for _ in range(10):  # 발행자가 같은 세기를 빠르게 계속 보낸다
        node._on_haptics('right', Haptics(strength=[2] * 5))
    assert client.sent == [(1, (2,) * 5)]
    node._sent_at['right'] -= RESEND_S
    node._on_haptics('right', Haptics(strength=[2] * 5))
    node._on_haptics('right', Haptics(strength=[3] * 5))  # 바뀐 세기는 바로 보낸다
    assert client.sent[1:] == [(1, (2,) * 5), (1, (3,) * 5)]


def test_orphaned_haptics_are_stopped(recording):
    node, client = recording
    node._on_haptics('right', Haptics(strength=[3] * 5))  # 발행 노드 없이 들어온 명령
    for _ in range(ORPHAN_TICKS):
        node._stop_orphaned_haptics()
    assert client.sent[-1] == (1, (0,) * 5)


def test_stamp_before_sim_clock_is_not_negative(recording):
    from rclpy.time import Time

    node, _ = recording
    stamp = node._stamp(Time(nanoseconds=0), 0.1)  # use_sim_time, /clock 전
    assert stamp.sec == 0 and stamp.nanosec == 0


def test_haptics_stop_when_commands_stop_arriving(recording):
    # 원격 발행자의 네트워크가 끊기면 발행자 수는 lease 동안 그대로라 시간으로 끈다.
    node, client = recording
    node._on_haptics('left', Haptics(strength=[2] * 5))
    node._command_at['left'] -= 2.0  # haptics_timeout(1 s) 넘게 명령 없음
    node._poll()
    assert client.sent[-1] == (0, (0,) * 5)


def test_failed_stop_send_is_retried(recording):
    node, client = recording

    def fail_once(side, strengths):
        client.set_haptics = lambda s, v: client.sent.append((s, tuple(v)))
        raise RuntimeError('send failed')

    node._on_haptics('left', Haptics(strength=[3] * 5))
    client.set_haptics = fail_once
    node._on_haptics('left', Haptics(strength=[0] * 5))
    node._sent_at['left'] -= RESEND_S
    node._poll()  # 결과가 없어도 _retry_stop 이 다시 보낸다
    assert client.sent == [(0, (3,) * 5), (0, (0,) * 5)]
