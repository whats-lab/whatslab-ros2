#!/usr/bin/env python3
"""햅틱 수동 테스트 — 키 0~3 으로 다섯 손가락 세기를 보낸다.

ros2 run airglove_driver haptics_keyboard --ros-args -p side:=left

0 끔 · 1 약 · 2 중 · 3 강 · q 종료(끄고 나감). 진동 중에는 같은 세기를 KEEPALIVE_S 마다 다시
보낸다 — 드라이버는 haptics_timeout 동안 명령이 없으면 끈다.
"""

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
KEEPALIVE_S = 0.3  # 드라이버 haptics_timeout(기본 1 s)보다 짧게
QUIT_KEYS = ('q', 'Q', '\x04')  # cbreak 라 Ctrl-D 는 EOF 가 아니라 문자로 온다


class HapticsKeyboard(Node):
    """한쪽 손 햅틱 토픽에 키보드로 고른 세기를 발행한다."""

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
        """드라이버 구독이 보일 때까지(최대 DRIVER_WAIT_S) 기다린다.

        DDS 발견 전에 보낸 첫 키는 사라진다.
        """
        deadline = time.monotonic() + DRIVER_WAIT_S
        while self._pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                self.get_logger().warn(
                    f'{self._pub.topic_name} 구독자 없음 — airglove_driver 가 도는지 확인'
                )
                return
            time.sleep(0.05)

    def stop_haptics(self) -> None:
        """끄는 명령을 보내고 구독자가 받을 때까지(최대 1초) 기다린다.

        노드를 바로 없애면 이 명령이 전달되기 전에 publisher 가 사라질 수 있다.
        """
        self.send(0)
        self._pub.wait_for_all_acked(Duration(seconds=1.0))


def _getch(fd: int, timeout_s: float) -> str | None:
    """키 하나. timeout_s 안에 없으면 None, stdin 이 닫혔으면 ''."""
    ready, _, _ = select.select([fd], [], [], timeout_s)
    # sys.stdin 버퍼를 거치면 select 가 버퍼에 남은 키를 못 본다 — fd 에서 바로 읽는다.
    return os.read(fd, 1).decode('latin-1') if ready else None


def _run(node: HapticsKeyboard, fd: int) -> None:
    """키를 읽어 세기를 보내고, 진동 중에는 마지막 전송 후 KEEPALIVE_S 마다 다시 보낸다."""
    level, sent_at = 0, time.monotonic()
    while True:
        wait = KEEPALIVE_S - (time.monotonic() - sent_at)
        if level and wait <= 0:  # 다른 키가 계속 와도 시간 기준으로 보낸다
            node.send(level, log=False)
            sent_at, wait = time.monotonic(), KEEPALIVE_S
        ch = _getch(fd, wait if level else None)
        if ch is None:
            continue
        if not ch or ch in QUIT_KEYS:  # '' = stdin 닫힘
            return
        if ch in '0123':
            level = int(ch)
            node.send(level)
            sent_at = time.monotonic()


def _raise_exit(signum, _frame):
    raise SystemExit(128 + signum)


def main(args: list[str] | None = None) -> int:
    """터미널에서 키를 읽어 햅틱을 보낸다. 끝날 때 0 을 보낸다."""
    if not sys.stdin.isatty():
        print(
            'haptics_keyboard 는 터미널에서 실행해야 합니다 (키 입력을 읽는다).', file=sys.stderr
        )
        return 1
    # rclpy 신호 처리기는 context 를 먼저 닫아 끄기 명령을 못 보내게 한다 — 신호는 예외로 받아
    # finally 에서 끈다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_exit)
    fd = sys.stdin.fileno()
    old_tty = termios.tcgetattr(fd)
    node = None
    try:
        # cbreak 를 내내 유지한다 — 키마다 켜고 끄면 그 사이 누른 키(0·q 포함)가 버려진다.
        tty.setcbreak(fd, termios.TCSANOW)
        node = HapticsKeyboard()  # 콜백이 없어 spin 하지 않는다 — 발행·ack 대기는 DDS 몫
        node.wait_for_driver()
        print('0 끔 · 1 약 · 2 중 · 3 강 · q 종료')
        _run(node, fd)
    except ValueError as e:  # 잘못된 side
        print(e, file=sys.stderr)
        return 1
    except KeyboardInterrupt:  # cbreak 라 Ctrl-C 는 SIGINT 로 온다 — 정상 종료
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
