<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%"></p>

<h1 align="center">airglove_ros2</h1>

<p align="center"><b>English</b> | <a href="README.ko.md">한국어</a></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS_2-Jazzy-blue.svg" alt="ROS 2"/></a>
  <img src="https://img.shields.io/badge/Spine-3.0.0-green.svg" alt="Spine"/>
</p>

ROS 2 driver and hand retargeting for the **AirGlove** data glove by WHATs LAB Corp.
The driver publishes what Spine (WHATs LAB's glove middleware) sends — 26 OpenXR hand joints
per hand, the wrist IMU, device status, alarms — and takes finger haptics. The retargeting
package turns the hand joints into robot-hand joint angles.

<p align="center"><img src="readme/real2sim_hand.jpg" alt="real-to-sim hand" width="80%"></p>

## Packages

| Package | What it does | Depends on |
|---|---|---|
| `airglove_msgs` | Messages, one per Spine message: `HandJoints`, `WristOrientation`, `DeviceStatus`, `Alarm`, `Haptics`, `HapticsResult` | — |
| `airglove_driver` | Spine 3.0.0 → topics, haptics → glove. The only node that opens the glove. | [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client) (pip) |
| `airglove_retargeting` | `HandJoints` → robot-hand `JointState` (GeoRT soft-prompt network), joint-command `safety_gate`, `hand_view` | [whatslab-sdk](https://github.com/whats-lab/whatslab-sdk) (pip) |

Spine talks to exactly one client per machine (UDP 4040), so run `airglove_driver` on the Spine
PC. The retargeting node only subscribes to topics and can run anywhere on the ROS network.
It accepts any source that publishes `airglove_msgs/HandJoints` (e.g. a headset hand-tracking bridge).

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/airglove/{left,right}/joints` | `airglove_msgs/HandJoints` | driver → | every new hand frame (~Spine rate). 26 × `[qx qy qz qw px py pz radius]`, wrist-relative, metres, OpenXR joint order and axes; `valid[26]` |
| `/airglove/{left,right}/wrist` | `airglove_msgs/WristOrientation` | driver → | raw dorsum IMU quaternion as Spine sends it (`[y, x, z, -w]`), not a wrist pose |
| `/airglove/status` | `airglove_msgs/DeviceStatus` | driver → | 1 Hz: gloves connected, Spine protocol (3), heartbeat age |
| `/airglove/alarm` | `airglove_msgs/Alarm` | driver → | `RESTART`, `ATLAS_DEVICE_ERROR` |
| `/airglove/{left,right}/haptics` | `airglove_msgs/Haptics` | → driver | 5 strengths (thumb..little), 0 off · 1 weak · 2 medium · 3 strong |
| `/airglove/{left,right}/haptics_result` | `airglove_msgs/HapticsResult` | driver → | result of the last haptics command |
| `/joint_states` | `sensor_msgs/JointState` | retargeting → | robot-hand joints; `/{side}_hand/joint_states` when `hand_type:=both` |

Publish `Haptics` continuously while the glove should vibrate: the driver sends zeros when no command
has arrived for `haptics_timeout` (1 s; `0` keeps the last command until zeros are sent). It also sends
zeros when no node publishes on that haptics topic any more (checked every second, e.g. the sender
crashed) and when it exits. A stop goes out at least twice, 0.2 s apart, and is resent until Spine
confirms it (`success` true), for up to 5 s; repeated zeros from a publisher are not forwarded again.
Spine itself turns haptics off when it has not heard the driver for 3 s.

## Installation

ROS 2 Jazzy, Ubuntu 24.04. The pip packages go into the system Python because colcon and rclpy run
on it; Ubuntu 24.04 marks that Python as externally managed (PEP 668), hence `--break-system-packages`.
numpy is pinned `<2` (Jazzy's system packages are built against numpy 1.x).

```bash
cd ~/ros2_ws/src && git clone https://github.com/whats-lab/whatslab-ros2.git airglove_ros2
pip install --break-system-packages -r airglove_ros2/requirements.txt  # airglove-client, whatslab-sdk, dexhand-description
cd ~/ros2_ws && colcon build && source install/setup.bash
```

Driver only (no whatslab-sdk): `pip install --break-system-packages -r airglove_ros2/airglove_driver/requirements.txt` and
`colcon build --packages-up-to airglove_driver`.

### Retargeting engines

`retarget` picks the engine (default `geort:orca_right`):

| `retarget` | Engine | Needs | `robot_urdf` (dexhand-description) |
|---|---|---|---|
| `geort:orca_right` | GeoRT soft prompt | — (model bundled with whatslab-sdk) | `orca_hand/urdf/{side}.urdf` (default, `{side}` = left/right) |
| `geort:hx5_right` | GeoRT soft prompt | — (model bundled with whatslab-sdk) | `robotis_hx5_d20/urdf/hx5_d20_right.urdf` |
| `dex:orca`, `dex:allegro`, `dex:leap`, `dex:ability`, `dex:schunk`, `dex:dg5f`, `dex:robotis` | dex-retargeting DexPilot | `pip install --break-system-packages 'whatslab-sdk[hand] @ git+https://github.com/whats-lab/whatslab-sdk.git@v0.5.0' 'numpy<2.0'` (torch) | the hand's URDF |

GeoRT runs on numpy only. The trained model ships with whatslab-sdk; set `WHATSLAB_GEORT_MODEL`
to use a different one. The model is right-handed; a left glove is mirrored onto it, so left input
still yields **right-hand robot joints**; with `{side}` in `robot_urdf` the node then picks the right-hand URDF itself. It also holds
Allegro and DG-5F-S prompts, but their joint names follow URDFs not yet in dexhand-description.
DexPilot works from any URDF with a `configs/hands/<hand>_<side>.yaml` in whatslab-sdk.

## Usage

```bash
ros2 launch airglove_retargeting retargeting.launch.py                 # driver + retargeting (right hand)
ros2 launch airglove_retargeting retargeting.launch.py hand_type:=both
ros2 launch airglove_retargeting retargeting.launch.py driver:=false   # retargeting only
ros2 launch airglove_driver driver.launch.py                           # driver only
ros2 run airglove_driver haptics_keyboard --ros-args -p side:=left     # haptics test (keys 0-3)
ros2 launch airglove_retargeting hand_view.launch.py gui:=false        # rviz (default: orca_hand, right)
```

`hand_type:=both` publishes per hand on `/{side}_hand/joint_states`. `hand_view` shows one hand from
`/joint_states`, so to watch one of them, relay it (package `ros-jazzy-topic-tools`):
`ros2 run topic_tools relay /left_hand/joint_states /joint_states` with `hand_view.launch.py gui:=false`.
`hand_view`'s `side` is the robot hand's side: `right` for `geort:orca_right`, even for a left glove.

| Node | Parameters |
|---|---|
| `airglove_driver` | `pair` (1), `listen_port` (4040), `spine_port` (4042), `publish_hz` (120), `haptics_timeout` (1.0 s, 0 = off; keep above 0.3 s, haptics_keyboard's resend period) |
| `airglove_retargeting` | `hand_type` (`left`\|`right`\|`both`), `retarget` (`geort:orca_right`), `robot_urdf` (zero-fills non-retargeted joints), `input_namespace` (`/airglove`) |
| `safety_gate` | `robot_description`, `limits_override`, `timeout`, `output_hz`, `enable_deadman` — sub `~/command_raw` → pub `~/command_safe`; srv `~/estop`, `~/reset`; sub `~/enable` |

## Compatibility

Spine **3.0.0** (OSC protocol 3) through AirGlove-Client 0.4.0. Spine and `airglove_driver` must run
on the same machine. Spine 2.x is not supported: for Spine below 2.3.1 use whatslab-ros2 0.1.0
(`atlas_hand`); Spine 2.3.x is not supported by any release.

Moving from 0.1.0 (`atlas_hand`):

| 0.1.0 | Now |
|---|---|
| `atlas_hand.launch.py` | `airglove_retargeting retargeting.launch.py` |
| haptics `Int32MultiArray` on `/{left,right}_hand/haptic`, `Empty` on `…/haptic/off` | `airglove_msgs/Haptics` on `/airglove/{left,right}/haptics` (all zeros = off) |
| `input_source:=meta_quest` (Quest hand input), human-hand TF | removed — publish `airglove_msgs/HandJoints` from your own bridge |
| `atlas_arm_teleop` / `arm_teleop.launch.py` | removed — stay on 0.1.0 |

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0) — source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
