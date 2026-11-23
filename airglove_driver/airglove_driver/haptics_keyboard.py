#!/usr/bin/env python3

import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from airglove_driver.convert import FINGER_COUNT
from airglove_driver.topics import SIDES, haptics_topic
from airglove_msgs.msg import Haptics

DRIVER_WAIT_S = 2.0
KEEPALIVE_S = 0.3
QUIT_KEYS = ('q', 'Q', '\x04')


class HapticsKeyboard(Node):

    def __init__(self):
        super().__init__('airglove_haptics_keyboard')
        self.declare_parameter('side', 'left')
        self._side = str(self.get_parameter('side').value).lower()
        if self._side not in SIDES:
            self.destroy_node()
            raise ValueError(f'side 는 left|right: {self._side!r}')
        self._pub = self.create_publisher(Haptics, haptics_topic(self._side), 10)

    def send(self, level: int, log: bool = True) -> None:
        self._pub.publish(Haptics(strength=[level] * FINGER_COUNT))
        if log:
            self.get_logger().info(f'[{self._side}] 햅틱 {level}')

    def wait_for_driver(self) -> None:
        deadline = time.monotonic() + DRIVER_WAIT_S
        while self._pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                self.get_logger().warn(
                    f'{self._pub.topic_name} 구독자 없음 — airglove_driver 가 도는지 확인'
                )
                return
            time.sleep(0.05)

    def stop_haptics(self) -> None:
        self.send(0)
        self._pub.wait_for_all_acked(Duration(seconds=1.0))


def _getch(fd: int, timeout_s: float) -> str | None:
    ready, _, _ = select.select([fd], [], [], timeout_s)
    return os.read(fd, 1).decode('latin-1') if ready else None


def _run(node: HapticsKeyboard, fd: int) -> None:
    level, sent_at = 0, time.monotonic()
    while True:
        wait = KEEPALIVE_S - (time.monotonic() - sent_at)
        if level and wait <= 0:
            node.send(level, log=False)
            sent_at, wait = time.monotonic(), KEEPALIVE_S
        ch = _getch(fd, wait if level else None)
        if ch is None:
            continue
        if not ch or ch in QUIT_KEYS:
            return
        if ch in '0123':
            level = int(ch)
            node.send(level)
            sent_at = time.monotonic()


def _raise_exit(signum, _frame):
    raise SystemExit(128 + signum)


def main(args: list[str] | None = None) -> int:
    if not sys.stdin.isatty():
        print(
            'haptics_keyboard 는 터미널에서 실행해야 합니다 (키 입력을 읽는다).', file=sys.stderr
        )
        return 1
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_exit)
    fd = sys.stdin.fileno()
    old_tty = termios.tcgetattr(fd)
    node = None
    try:
        tty.setcbreak(fd, termios.TCSANOW)
        node = HapticsKeyboard()
        node.wait_for_driver()
        print('0 끔 · 1 약 · 2 중 · 3 강 · q 종료')
        _run(node, fd)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_tty)
        if node is not None:
            node.stop_haptics()
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
