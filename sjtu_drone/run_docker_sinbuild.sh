#!/usr/bin/env bash
set -euo pipefail

usage(){
  echo "Usage: $0 [-r <humble|iron|rolling>]"
  echo
  echo "Environment overrides:"
  echo "  HOST_WS=/path/on/host              (default: /home/franfuentes/ros2_sjtu_ws)"
  echo "  CONTAINER_WS=/path/in/container    (default: /ros2_ws)"
  echo "  FORCE_PULL=1                       (force docker pull even if image exists locally)"
  exit 1
}

ROS_DISTRO="${ROS_DISTRO:-humble}"  # [humble, iron, rolling]
while getopts "r:h" opt; do
  case "$opt" in
    r)
      if [[ "$OPTARG" != "humble" && "$OPTARG" != "iron" && "$OPTARG" != "rolling" ]]; then
        echo "Invalid ROS distro: $OPTARG" >&2
        usage
      fi
      ROS_DISTRO="$OPTARG"
      ;;
    h|\?)
      usage
      ;;
  esac
done

IMAGE="georgno/sjtu_drone:ros2-${ROS_DISTRO}"
FORCE_PULL="${FORCE_PULL:-0}"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

HOST_WS="${HOST_WS:-/home/franfuentes/ros2_sjtu_ws}"
CONTAINER_WS="${CONTAINER_WS:-/ros2_ws}"

HOST_SRC="${HOST_WS}/src"
CONTAINER_SRC="${CONTAINER_WS}/src"

if [[ ! -d "${HOST_SRC}" ]]; then
  echo "Host src/ folder not found: ${HOST_SRC}" >&2
  exit 1
fi

# Ensure persistent dirs exist (in case you rely on host install/build/log)
mkdir -p "${HOST_WS}/build" "${HOST_WS}/install" "${HOST_WS}/log"

if [[ "${FORCE_PULL}" == "1" ]]; then
  docker pull "${IMAGE}"
else
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    docker pull "${IMAGE}"
  fi
fi

xhost +local:docker >/dev/null

docker run \
  -it --rm \
  -v ${HOST_SRC}:${CONTAINER_SRC}:rw \
  -v ${HOST_WS}/build:${CONTAINER_WS}/build:rw \
  -v ${HOST_WS}/install:${CONTAINER_WS}/install:rw \
  -v ${HOST_WS}/log:${CONTAINER_WS}/log:rw \
  -v ${XSOCK}:${XSOCK} \
  -v ${XAUTH}:${XAUTH} \
  -e DISPLAY=${DISPLAY} \
  -e XAUTHORITY=${XAUTH} \
  --env=QT_X11_NO_MITSHM=1 \
  --privileged \
  --net=host \
  --name="sjtu_drone" \
  -w "${CONTAINER_WS}" \
  "${IMAGE}" \
  bash -lc "
    set -euo pipefail
    cd ${CONTAINER_WS}

    echo '[run_docker] Sourcing ROS + workspace overlay (no build)...'
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source install/setup.bash
    set -u

    echo '[run_docker] Launching sjtu_drone...'
    ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
  "

xhost -local:docker >/dev/null

