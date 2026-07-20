<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%"></p>

<h1 align="center">whatslab-ros2</h1>

<p align="center"><a href="README.md">English</a> | <b>한국어</b></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS_2-Jazzy-blue.svg" alt="ROS 2"/></a>
  <img src="https://img.shields.io/badge/python-3.10%2B-blue.svg" alt="Python"/>
</p>

**whatslab-ros2** 는 [whatslab](https://github.com/whats-lab/whatslab-sdk) 텔레오퍼레이션
SDK 를 ROS 2 노드로 노출하는 얇은 래퍼입니다. 입력 수신·손/팔 리타게팅·IK·안전 등
모든 로직은 `whatslab` 에 있으며, 이 패키지는 그것을 ROS 2 토픽·서비스·런치에 연결만
합니다.

<p align="center"><img src="readme/real2sim_hand.jpg" alt="real-to-sim hand" width="80%"></p>

## 노드

ament 패키지 이름은 `atlas_hand` 이며, 3개의 실행 파일을 제공합니다:

| 실행 파일 | 역할 | 주요 토픽 / 서비스 |
|---|---|---|
| `atlas_hand` | 손 전용: whatslab 리시버 → 손 리타게팅 | pub `/joint_states`; sub 햅틱 피드백 |
| `atlas_arm_teleop` | 팔 + 손: whatslab `GloveModel` | pub `/joint_states`, `/arm/joint_trajectory`; srv `~/calibrate` |
| `safety_gate` | `whatslab.safety` clamp · rate-limit · watchdog · e-stop | sub `~/command_raw` → pub `~/command_safe`; srv `~/estop`, `~/reset`; sub `~/enable` |

### 주요 파라미터

- **`atlas_hand`** — `input_source` (`atlas`\|`meta_quest`), `hand_type` (`left`\|`right`\|`both`), `robot_config`, `server_port` (AirGlove OSC, 4040), `osc_port` (Quest OSC, 9000)
- **`atlas_arm_teleop`** — `rig`, `controller_side`, `hand_side`, `quest_port`, `glove_port`, `publish_hz`, `arm_traj_time` (손 config 는 rig 에서 유도)
- **`safety_gate`** — `robot_description`, `limits_override`, `timeout`, `output_hz`, `enable_deadman`

## 설치

ROS 2 Jazzy 와 `whatslab` SDK(pip 설치)가 필요합니다. numpy 는 `<2` 로 고정합니다.

```bash
# 1) whatslab SDK + 의존성 (ROS 2 파이썬 환경에서)
pip install -r requirements.txt      # whatslab-sdk[hand,receiver] @ git + numpy<2

# 2) colcon 워크스페이스에서 빌드
cd ~/ros2_ws && colcon build --packages-select atlas_hand && source install/setup.bash
```

URDF·메쉬는 [`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf)
패키지가 제공합니다(whatslab 의 `hand`/`arm` extra 가 가져옴).

## 사용

```bash
ros2 launch atlas_hand atlas_hand.launch.py      # 손 전용 텔레옵
ros2 launch atlas_hand arm_teleop.launch.py      # 팔 + 손 텔레옵
ros2 launch atlas_hand hand_view.launch.py       # rviz 뷰
```

**Meta Quest** 입력을 쓰려면 [whatslab-sdk](https://github.com/whats-lab/whatslab-sdk)
레포의 `assets/quest/` 에 있는 PoseDataTracker APK 를 헤드셋에 설치하고 같은 Wi-Fi 에
연결하세요. **AGA 글러브**는 [WHATs LAB](https://www.whatslab.co.kr/) 에서 구매할 수 있습니다.

## 호환성

데이터 글러브 텔레옵은 왓츠랩의 글러브 미들웨어 **Spine** 을 거칩니다. 지원하는 Spine
버전은 **2.3.1 미만**입니다(2.3.1 이상은 아직 지원하지 않습니다).

## 라이선스

**주식회사 왓츠랩(WHATs LAB Corp) 소스 코드 라이선스**(CC BY-NC-ND 4.0 기반)를 따릅니다
— source-available, 비영리, 2차 저작물 금지. 자세한 내용은 [LICENSE](LICENSE) 를 참고하세요.

Copyright © 주식회사 왓츠랩(WHATs LAB Corp). All rights reserved.
