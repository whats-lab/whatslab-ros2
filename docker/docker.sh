#!/bin/bash

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE="airglove_ros2:latest"
COLCON_WS="/root/ros2_ws"
FASTDDS_TRANSPORT="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

case "$1" in

  build)
    echo ">>> 이미지 빌드"
    docker build -f "${SCRIPT_DIR}/Dockerfile" \
      -t "$IMAGE" "$REPO_ROOT"
    ;;

  enter)
    CONTAINER="airglove_ros2_dev"
    
    if docker ps -q -f name="^${CONTAINER}$" | grep -q .; then
      docker exec -it \
        -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_TRANSPORT}" \
        "$CONTAINER" bash
    else
      docker run --rm -it --name "$CONTAINER" \
        --ipc=host \
        --network host \
        -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_TRANSPORT}" \
        -e DISPLAY=:0 \
        -v /:/host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$REPO_ROOT:$COLCON_WS/src/airglove_ros2" \
        --privileged \
        "$IMAGE" bash
    fi
    ;;

  *)
    echo "사용법: $0 {build | enter}"
    exit 1
    ;;

esac
