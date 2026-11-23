import time

import pytest
import rclpy

from airglove_driver.driver_node import ORPHAN_TICKS, RESEND_S, AirGloveDriver
from airglove_msgs.msg import Haptics


class _FailingClient:

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
    rclpy.init(args=['--ros-args', '-p', 'publish_hz:=120'])
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
    node._poll()


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

    def __init__(self):
        self.sent = []
        self.result = None

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
    client.result = (True, time.monotonic())
    node._poll()
    assert node._stop_deadline['left'] is not None
    node._sent_at['left'] -= RESEND_S
    node._poll()
    assert client.sent == [(0, (0,) * 5)]
    client.result = (True, time.monotonic())
    node._poll()
    assert node._stop_deadline['left'] is None
    node._sent_at['left'] -= RESEND_S
    node._poll()
    assert len(client.sent) == 1


def test_unconfirmed_stop_is_resent(recording):
    node, client = recording
    _vibrate_then_stop(node, client)
    for _ in range(3):
        node._sent_at['left'] -= RESEND_S
        node._poll()
    assert client.sent == [(0, (0,) * 5)] * 3


def test_stop_retry_gives_up_after_window(recording):
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
    node._on_haptics('left', Haptics(strength=[0] * 5))
    assert client.sent == [] and node._stop_sends['left'] == 1


def test_identical_commands_are_rate_limited(recording):
    node, client = recording
    for _ in range(10):
        node._on_haptics('right', Haptics(strength=[2] * 5))
    assert client.sent == [(1, (2,) * 5)]
    node._sent_at['right'] -= RESEND_S
    node._on_haptics('right', Haptics(strength=[2] * 5))
    node._on_haptics('right', Haptics(strength=[3] * 5))
    assert client.sent[1:] == [(1, (2,) * 5), (1, (3,) * 5)]


def test_orphaned_haptics_are_stopped(recording):
    node, client = recording
    node._on_haptics('right', Haptics(strength=[3] * 5))
    for _ in range(ORPHAN_TICKS):
        node._stop_orphaned_haptics()
    assert client.sent[-1] == (1, (0,) * 5)


def test_stamp_before_sim_clock_is_not_negative(recording):
    from rclpy.time import Time

    node, _ = recording
    stamp = node._stamp(Time(nanoseconds=0), 0.1)
    assert stamp.sec == 0 and stamp.nanosec == 0


def test_haptics_stop_when_commands_stop_arriving(recording):
    node, client = recording
    node._on_haptics('left', Haptics(strength=[2] * 5))
    node._command_at['left'] -= 2.0
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
    node._poll()
    assert client.sent == [(0, (3,) * 5), (0, (0,) * 5)]
