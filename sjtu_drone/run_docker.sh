#!/usr/bin/env bash
set -euo pipefail

usage(){
  cat <<'EOF'
Usage: ./run_docker_build.sh [-r <humble|iron|rolling>]

Environment overrides:
  HOST_WS=/path/on/host           (if unset, autodetect by searching for src/)
  CONTAINER_WS=/path/in/container (default: /ros2_ws)
  FORCE_PULL=1                    (force docker pull even if image exists locally)
EOF
  exit 1
}

find_ws_root() {
  local d
  d="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while true; do
    if [[ -d "${d}/src" ]]; then
      echo "${d}"
      return 0
    fi
    [[ "${d}" == "/" ]] && return 1
    d="$(dirname "${d}")"
  done
}

ROS_DISTRO="${ROS_DISTRO:-humble}"
while getopts "r:h" opt; do
  case "$opt" in
    r)
      case "$OPTARG" in
        humble|iron|rolling) ROS_DISTRO="$OPTARG" ;;
        *) echo "Invalid ROS distro: $OPTARG" >&2; usage ;;
      esac
      ;;
    h|\?) usage ;;
  esac
done

IMAGE="georgno/sjtu_drone:ros2-${ROS_DISTRO}"
FORCE_PULL="${FORCE_PULL:-0}"

XSOCK=/tmp/.X11-unix
XAUTH=${XAUTHORITY:-$HOME/.Xauthority}

CONTAINER_WS="${CONTAINER_WS:-/ros2_ws}"

if [[ -z "${HOST_WS:-}" ]]; then
  if ! HOST_WS="$(find_ws_root)"; then
    echo "ERROR: Could not autodetect HOST_WS. Set HOST_WS=/path/to/colcon_ws (must contain src/)." >&2
    exit 1
  fi
fi

HOST_SRC="${HOST_WS}/src"
[[ -d "${HOST_SRC}" ]] || { echo "Host src/ folder not found: ${HOST_SRC}" >&2; exit 1; }

mkdir -p "${HOST_WS}/build" "${HOST_WS}/install" "${HOST_WS}/log"

if [[ "${FORCE_PULL}" == "1" ]]; then
  docker pull "${IMAGE}"
else
  docker image inspect "${IMAGE}" >/dev/null 2>&1 || docker pull "${IMAGE}"
fi

xhost +local:docker >/dev/null || true

docker run \
  -it --rm \
  -v "${HOST_SRC}:${CONTAINER_WS}/src:rw" \
  -v "${HOST_WS}/build:${CONTAINER_WS}/build:rw" \
  -v "${HOST_WS}/install:${CONTAINER_WS}/install:rw" \
  -v "${HOST_WS}/log:${CONTAINER_WS}/log:rw" \
  -v "${XSOCK}:${XSOCK}" \
  -v "${XAUTH}:${XAUTH}" \
  -e DISPLAY="${DISPLAY}" \
  -e XAUTHORITY="${XAUTH}" \
  --env=QT_X11_NO_MITSHM=1 \
  --privileged \
  --net=host \
  --name="sjtu_drone" \
  -w "${CONTAINER_WS}" \
  "${IMAGE}" \
  bash -lc "
    set -euo pipefail
    cd ${CONTAINER_WS}

    echo '[run_docker_build] Sourcing ROS...'
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u

    # Limpieza segura: solo build cache (mantiene install/log persistentes)
    echo '[run_docker_build] Cleaning build cache...'
    rm -rf build/*

    echo '[run_docker_build] Building (symlink-install)...'
    colcon build --symlink-install

    echo '[run_docker_build] Sourcing overlay...'
    set +u
    source install/setup.bash
    set -u

    echo '[run_docker_build] Launching sjtu_drone...'
    ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
  "

xhost -local:docker >/dev/null || true

