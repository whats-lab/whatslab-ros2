<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%"></p>

<h1 align="center">airglove_ros2</h1>

<p align="center"><a href="README.md">English</a> | <b>한국어</b></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS_2-Jazzy-blue.svg" alt="ROS 2"/></a>
  <img src="https://img.shields.io/badge/Spine-3.0.0-green.svg" alt="Spine"/>
</p>

주식회사 왓츠랩(WHATs LAB Corp)의 데이터 글러브 **AirGlove** 용 ROS 2 드라이버와 손 리타게팅입니다.
드라이버는 글러브 미들웨어 Spine 이 보내는 값(손마다 OpenXR 26관절, 손목 IMU, 장치 상태, 알람)을
그대로 발행하고 손가락 햅틱을 받습니다. 리타게팅 패키지는 손 관절을 로봇 손 관절각으로 바꿉니다.

<p align="center"><img src="readme/real2sim_hand.jpg" alt="real-to-sim hand" width="80%"></p>

## 패키지

| 패키지 | 역할 | 의존 |
|---|---|---|
| `airglove_msgs` | Spine 메시지마다 하나: `HandJoints`, `WristOrientation`, `DeviceStatus`, `Alarm`, `Haptics`, `HapticsResult` | — |
| `airglove_driver` | Spine 3.0.0 → 토픽, 햅틱 → 글러브. 글러브를 여는 유일한 노드. | [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client) (pip) |
| `airglove_retargeting` | `HandJoints` → 로봇 손 `JointState`(GeoRT 소프트 프롬프트 신경망), 관절 명령 `safety_gate`, `hand_view` | [whatslab-sdk](https://github.com/whats-lab/whatslab-sdk) (pip) |

Spine 은 PC 당 클라이언트 하나(UDP 4040)에만 보내므로 `airglove_driver` 는 Spine PC 에서 돌립니다.
리타게팅 노드는 토픽만 구독하므로 ROS 네트워크의 어느 PC 에서든 돌릴 수 있고,
`airglove_msgs/HandJoints` 를 내는 다른 소스(헤드셋 핸드트래킹 브리지 등)도 입력으로 받습니다.

## 토픽

| 토픽 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/airglove/{left,right}/joints` | `airglove_msgs/HandJoints` | 드라이버 → | 새 손 프레임마다. 초당 `publish_hz`(120)번 확인합니다. 26 × `[qx qy qz qw px py pz radius]`, 손목 기준, m, OpenXR 관절 순서·축. `valid[26]` |
| `/airglove/{left,right}/wrist` | `airglove_msgs/WristOrientation` | 드라이버 → | Spine 이 보내는 손등 IMU 원시 quat(`[y, x, z, -w]`). 손목 자세가 아님 |
| `/airglove/status` | `airglove_msgs/DeviceStatus` | 드라이버 → | 1 Hz: 글러브 연결, Spine 프로토콜(3), heartbeat 경과 |
| `/airglove/alarm` | `airglove_msgs/Alarm` | 드라이버 → | `RESTART`, `ATLAS_DEVICE_ERROR` |
| `/airglove/{left,right}/haptics` | `airglove_msgs/Haptics` | → 드라이버 | 세기 5개(엄지..새끼), 0 끔 · 1 약 · 2 중 · 3 강 |
| `/airglove/{left,right}/haptics_result` | `airglove_msgs/HapticsResult` | 드라이버 → | 마지막 햅틱 명령 결과 |
| `/joint_states` | `sensor_msgs/JointState` | 리타게팅 → | 로봇 손 관절. `hand_type:=both` 면 `/{side}_hand/joint_states` |

진동시키는 동안에는 `Haptics` 를 계속 발행해 주세요. 드라이버는 `haptics_timeout`(1초) 동안 명령이
없으면 0 을 보냅니다(`0` 이면 0 을 보낼 때까지 마지막 명령을 유지합니다). 또 그 햅틱 토픽에 발행하는
노드가 없어지거나(보내던 노드가 죽은 경우 등. 1초마다 확인해 두 번 연속, 약 2초면 끕니다) 드라이버가 종료할 때 0 을 보냅니다. 끄기는
0.2초 간격으로 최소 두 번 보내고, Spine 이 확인(`success` true)할 때까지 최대 5초 동안 다시 보냅니다.
발행자가 0 을 반복해 보내도 다시 전달하지 않습니다. Spine 도 드라이버가 3초 동안 응답이 없으면 끕니다.

## 메시지

모든 메시지는 Spine pair(`pair`, `/pair/N`)를, 해당하면 `side`(`SIDE_LEFT=0`, `SIDE_RIGHT=1`)를 담습니다.
`header.stamp` 는 드라이버가 값을 받은 시각입니다(Spine 은 시각을 보내지 않음).

| 메시지 | Spine OSC 메시지 | 필드 |
|---|---|---|
| `HandJoints` | 1 / 2 | `joints[208]`: OpenXR 26관절(PALM, WRIST, THUMB_METACARPAL .. LITTLE_TIP) × `[qx qy qz qw px py pz radius]`, 미터, 손목 기준(WRIST = 단위). 양손 모두 OpenXR 손 관절 축(−Z 가 손끝 방향 뼈, +Y 가 손등). `valid[j]` 는 Spine 이 관절 `j` 에 유한하지 않은 값을 보냈을 때 거짓이고, 그 관절은 원점·단위 회전으로 채웁니다. `seq` 는 드라이버가 그 손에서 받은 프레임 수. `header.frame_id` = `airglove_<side>_wrist`. |
| `WristOrientation` | 18 / 19 | `raw[4]`: Spine 이 보낸 그대로의 손등 IMU 쿼터니언 — 센서의 `[w, x, y, z]` 를 `[y, x, z, −w]` 로 재배열한 값이고, 정규화하지 않았고 어떤 월드·ROS 좌표계도 아닙니다. 손목 자세가 아니므로 자체 센서-월드 정렬과 함께만 쓰세요. |
| `DeviceStatus` | 3 + 14 | `left_connected`, `right_connected`; `spine_protocol`(Spine 3.0.0 이면 3, 하트비트 응답 전이면 0); `spine_age`(마지막 하트비트 응답 후 초, 아직 없으면 −1). |
| `Alarm` | 13 | `code`(예: `RESTART`, `ATLAS_DEVICE_ERROR`); Spine 이 손을 지정하지 않으면 `side` 는 `SIDE_UNKNOWN=255`. |
| `Haptics` | 9 / 10 | `strength[5]` 엄지..소지: `OFF=0`, `WEAK=1`, `MEDIUM=2`, `STRONG=3`(0..3 으로 자름). |
| `HapticsResult` | 11 / 12 | 글러브가 연결돼 있지 않거나 이전 명령을 처리 중이면 `success` 는 거짓. |

## 설치

ROS 2 Jazzy, Ubuntu 24.04. colcon 과 rclpy 가 시스템 Python 에서 돌기 때문에 pip 패키지도 시스템 Python 에
설치합니다. Ubuntu 24.04 는 이 Python 을 외부 관리 환경(PEP 668)으로 막아 두므로 `--break-system-packages` 를 붙입니다.
numpy 는 `<2` 로 고정합니다(Jazzy 시스템 패키지가 numpy 1.x 기준으로 빌드됨).

```bash
cd ~/ros2_ws/src && git clone https://github.com/whats-lab/whatslab-ros2.git airglove_ros2
pip install --break-system-packages -r airglove_ros2/requirements.txt  # airglove-client, whatslab-sdk, dexhand-description
cd ~/ros2_ws && colcon build && source install/setup.bash
```

드라이버만 쓸 때(whatslab-sdk 불필요): `pip install --break-system-packages -r airglove_ros2/airglove_driver/requirements.txt`,
`colcon build --packages-up-to airglove_driver`.

### 리타게팅 엔진

`retarget` 로 엔진을 고릅니다 (기본 `geort:orca_right`).

| `retarget` | 엔진 | 필요한 것 | `robot_urdf` (dexhand-description) |
|---|---|---|---|
| `geort:orca_right` | GeoRT 소프트 프롬프트 | — (모델은 whatslab-sdk 에 동봉) | `orca_hand` (기본) |
| `geort:hx5_right` | GeoRT 소프트 프롬프트 | — (모델은 whatslab-sdk 에 동봉) | `robotis_hx5_d20` |
| `dex:orca`, `dex:allegro`, `dex:leap`, `dex:ability`, `dex:schunk`, `dex:dg5f`, `dex:robotis` | dex-retargeting DexPilot | `pip install --break-system-packages 'whatslab-sdk[hand] @ git+https://github.com/whats-lab/whatslab-sdk.git@v1.0.0' 'numpy<2.0'` (torch) | 해당 손 URDF |

GeoRT 는 numpy 만으로 돕니다. 학습된 모델은 whatslab-sdk 에 함께 들어 있고, 다른 모델을 쓰려면
`WHATSLAB_GEORT_MODEL` 로 지정하세요. 모델은 오른손용이고 왼손 글러브는 좌우 반전해서 씁니다 — 그래서
왼손 입력도 **오른손 로봇 관절**을 냅니다. 이때 노드가 오른손 URDF 를 알아서 고릅니다. `robot_urdf` 는
dexhand-description 의 손 폴더 이름(`hand_view` 의 `model` 과 같은 규칙)이나 URDF 경로(절대 경로 또는 그 패키지
기준, `{side}` = left/right)입니다. Allegro·DG-5F-S
프롬프트도 들어 있지만 관절 이름이 아직 dexhand-description 에 없는 URDF 기준입니다.
DexPilot 은 whatslab-sdk 의 `configs/hands/<손>_<side>.yaml` 이 있는 URDF 면 어떤 손이든 됩니다.

## 사용

```bash
ros2 launch airglove_retargeting retargeting.launch.py                 # 드라이버 + 리타게팅 (오른손)
ros2 launch airglove_retargeting retargeting.launch.py hand_type:=both
ros2 launch airglove_retargeting retargeting.launch.py driver:=false   # 리타게팅만
ros2 launch airglove_driver driver.launch.py                           # 드라이버만
ros2 run airglove_driver haptics_keyboard --ros-args -p side:=left     # 햅틱 테스트 (키 0-3)
ros2 launch airglove_retargeting hand_view.launch.py gui:=false        # rviz (기본 orca_hand 오른손)
ros2 launch airglove_retargeting hand_view.launch.py model:=robotis_hx5_d20   # geort:hx5_right·dex:robotis 를 rviz 로
ros2 launch airglove_retargeting hand_view.launch.py urdf:=/path/to/hand.urdf  # 임의의 URDF
ros2 run airglove_retargeting safety_gate --ros-args -p robot_description:="$(cat hand.urdf)"
```

`hand_type:=both` 는 `/{side}_hand/joint_states` 로 나눠 발행합니다. `hand_view` 는 `/joint_states` 로 한 손만
보여 주므로, 한쪽을 보려면 relay 해 주세요(패키지 `ros-jazzy-topic-tools`):
`ros2 run topic_tools relay /left_hand/joint_states /joint_states` 와 `hand_view.launch.py gui:=false`.
`hand_view` 의 `side` 는 로봇 손 쪽입니다. `geort:orca_right` 면 왼손 글러브여도 `right` 입니다.
`hand_view` 의 `model` 은 dexhand-description 의 손 폴더 이름입니다(`orca_hand`, `allegro_hand`, `tesollo_dg5f`, ...).
폴더 안에서 `<model>_<side>.urdf`, `<model>.urdf`, `urdf/<side>.urdf` 순으로 찾고, 없으면 유일한 `*<side>.urdf`
(`urdf/` 포함)를 씁니다. 한쪽에 변형이 여러 개인 폴더는 `urdf:=` 로 지정하세요(절대 경로 또는 dexhand-description
기준, `{side}` = left/right). 상대 메시 경로는 `file://` URI 로 바꿔 넘기고, rviz(`rviz:=false` 면 생략)는 URDF 루트
링크를 fixed frame 으로 하는 공통 설정 하나를 씁니다. `retargeting.launch.py` 는 `driver:=true` 일 때 아래 드라이버
파라미터(`pair`, `listen_port`, `spine_port`, `publish_hz`, `haptics_timeout`)도 받습니다.

| 노드 | 파라미터 |
|---|---|
| `airglove_driver` | `pair`(1), `listen_port`(4040), `spine_port`(4042), `publish_hz`(120, 새 프레임 확인 주기), `haptics_timeout`(1.0 s, 0 = 끔; haptics_keyboard 재전송 주기인 0.3 s 보다 길게) |
| `airglove_retargeting` | `hand_type`(`left`\|`right`\|`both`), `retarget` (`geort:orca_right`), `robot_urdf`(리타게팅 대상이 아닌 관절 0 채움), `input_namespace`(`/airglove`) |
| `safety_gate` | `robot_description`(필수), `limits_override`, `timeout`(0.2 s), `output_hz`(100), `enable_deadman`(false) — sub `~/command_raw` → pub `~/command_safe`; srv `~/estop`, `~/reset`; sub `~/enable` |

`safety_gate` 는 방어 층 중 하나일 뿐 마지막이 아닙니다 — 최종 권한은 하드웨어 비상정지와 모터 한계에 있습니다. 게이트가 드라이버 명령 토픽의 유일한 발행자여야 합니다(우회 금지). ros2_control 드라이버면 그 명령 한계가 위치·속도를 강제하고 게이트는 watchdog·deadman·비상정지를 더하며, 그 밖의 드라이버면 게이트가 clamp 까지 맡습니다.

## 호환성

Spine **3.0.0**(OSC 프로토콜 3), AirGlove-Client 0.4.0 을 씁니다. Spine 과 `airglove_driver` 는 같은 PC 에서
돌아야 합니다. Spine 2.x 는 지원하지 않습니다 — Spine 2.3.1 미만은 whatslab-ros2 0.1.0(`atlas_hand`)을
쓰시고, Spine 2.3.x 는 어느 릴리스에서도 지원하지 않습니다.

0.1.0(`atlas_hand`)에서 옮길 때:

| 0.1.0 | 지금 |
|---|---|
| `atlas_hand.launch.py` | `airglove_retargeting retargeting.launch.py` |
| 햅틱 `/{left,right}_hand/haptic` 의 `Int32MultiArray`, `…/haptic/off` 의 `Empty` | `/airglove/{left,right}/haptics` 의 `airglove_msgs/Haptics` (모두 0 = 끔) |
| `input_source:=meta_quest`(Quest 손 입력), 사람 손 TF | 없어졌습니다 — 직접 만든 브리지에서 `airglove_msgs/HandJoints` 를 발행하세요 |
| `atlas_arm_teleop` / `arm_teleop.launch.py` | 없어졌습니다 — 0.1.0 을 계속 쓰세요 |
| 파라미터 `robot_config`, `server_port`, `osc_port` | `retarget`, `listen_port`. `osc_port`(Quest)는 없어졌습니다 |

## 라이선스

**주식회사 왓츠랩(WHATs LAB Corp) 소스 코드 라이선스**(CC BY-NC-ND 4.0 기반)를 따릅니다
— source-available, 비영리, 2차 저작물 금지. 자세한 내용은 [LICENSE](LICENSE) 를 참고하세요.

Copyright © 주식회사 왓츠랩(WHATs LAB Corp). All rights reserved.
