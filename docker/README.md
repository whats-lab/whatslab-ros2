# airglove_ros2 Docker 환경

## 구조

```
docker/
├── Dockerfile      # ros:jazzy-ros-base 기반 빌드 이미지
├── docker.sh       # 빌드 / 셸 진입 스크립트
└── README.md
```

## 사전 요구사항

- Docker Engine 24.0+

---

## 빠른 시작

호스트 셸에서 아래 환경변수를 먼저 설정해야 합니다.

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

```bash
# 1. 이미지 빌드 (GitHub 에서 pip 의존성을 받으므로 네트워크 필요)
./docker/docker.sh build

# 2. 컨테이너 접속
./docker/docker.sh enter
```

`docker.sh enter`는 컨테이너 내부에도 `FASTDDS_BUILTIN_TRANSPORTS`를 전달하며,
호스트에서 미설정 시 기본값 `UDPv4`를 사용합니다.

---

## 명령어

| 명령어 | 설명 |
|--------|------|
| `./docker/docker.sh build` | 이미지 빌드 (airglove-client, whatslab-sdk, dexhand-description 포함. GeoRT 와 `dex:*` 엔진 모두 사용 가능) |
| `./docker/docker.sh enter` | 인터랙티브 셸 진입. 이미 떠 있으면 그 컨테이너로 들어갑니다 |

`enter` 는 `--privileged`, `--network host`, `--ipc=host` 로 띄우고 호스트 루트를 `/host` 에, 저장소를
`src/airglove_ros2` 에 마운트합니다. 컨테이너가 호스트 장치·파일 전체에 접근하므로 신뢰하는 PC 에서만 쓰세요.
rviz 는 `DISPLAY=:0` 과 X11 소켓을 쓰므로 호스트에서 `xhost +local:` 이 필요할 수 있습니다.

---

## Dockerfile 구성

```
ros:jazzy-ros-base
  └── rosdep install     # ROS 의존성 자동 설치
  └── pip install        # requirements.txt 전체 패키지 설치
  └── colcon build       # airglove_msgs · airglove_driver · airglove_retargeting 빌드
  └── .bashrc 설정       # source, alias cb
```
