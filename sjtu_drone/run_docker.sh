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

ROS_DISTRO="${ROS_DISTRO:-iron}"  # [humble, iron, rolling]
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

# ----------------------------
# Workspace mount (host -> container): mount ONLY src/
# ----------------------------
HOST_WS="${HOST_WS:-/home/franfuentes/ros2_sjtu_ws}"
CONTAINER_WS="${CONTAINER_WS:-/ros2_ws}"

HOST_SRC="${HOST_WS}/src"
CONTAINER_SRC="${CONTAINER_WS}/src"

if [[ ! -d "${HOST_SRC}" ]]; then
  echo "Host src/ folder not found: ${HOST_SRC}" >&2
  echo "Set HOST_WS to your workspace path (it must contain src/)." >&2
  exit 1
fi

# Only pull if the image is missing locally, unless FORCE_PULL=1
if [[ "${FORCE_PULL}" == "1" ]]; then
  docker pull "${IMAGE}"
else
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    docker pull "${IMAGE}"
  fi
fi

# X11 access for GUI tools (Gazebo/RViz)
xhost +local:docker >/dev/null

# Bind-mount ONLY src/ so you keep the image's prebuilt install/ (avoids ABI issues with Gazebo plugins)
VOLUMES="-v ${HOST_SRC}:${CONTAINER_SRC}:rw"

docker run \
  -it --rm \
  ${VOLUMES} \
  -v ${XSOCK}:${XSOCK} \
  -v ${XAUTH}:${XAUTH} \
  -e DISPLAY=${DISPLAY} \
  -e XAUTHORITY=${XAUTH} \
  --env=QT_X11_NO_MITSHM=1 \
  --privileged \
  --net=host \
  --name="sjtu_drone" \
  -w "${CONTAINER_WS}" \
  "${IMAGE}"

xhost -local:docker >/dev/null
