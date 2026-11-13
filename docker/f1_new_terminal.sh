#!/usr/bin/env bash
# Open a shell in the "f1tenth" ROS 2 container, creating/starting it first
# if needed. Used by the f1_new_terminal alias.
#
# The container gets:
#   - host networking   -> foxglove_bridge on ws://localhost:8765 works from the host
#   - all /dev + a cgroup rule for USB serial (ttyACM/ttyUSB) -> Arduino can be
#     unplugged / replugged without recreating the container
#   - X11 + NVIDIA GPU  -> same as the old rocker start_f1 command (f1tenth sim)
#   - /home/vamsi/sim_ws/src -> /sim_ws/src (as before)
#   - this repo -> /tsp05
#
# Usage: f1_new_terminal [command...]   (default: bash)

set -euo pipefail

NAME=f1tenth
IMAGE=f1tenth_tsp05:latest
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Build the image if it does not exist yet
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker build -t "$IMAGE" "$REPO_DIR/docker"
fi

if ! docker container inspect "$NAME" >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true

    gpu_args=()
    if docker info 2>/dev/null | grep -q 'nvidia.com/gpu=all'; then
        gpu_args=(--device nvidia.com/gpu=all)
    fi

    docker run -d --name "$NAME" \
        --init \
        --network host \
        -v /dev:/dev \
        --device-cgroup-rule='c 166:* rmw' \
        --device-cgroup-rule='c 188:* rmw' \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        "${gpu_args[@]}" \
        -v /home/vamsi/sim_ws/src:/sim_ws/src \
        -v "$REPO_DIR":/tsp05 \
        --entrypoint bash \
        "$IMAGE" -c 'sleep infinity' >/dev/null
elif [ "$(docker container inspect -f '{{.State.Running}}' "$NAME")" != "true" ]; then
    docker start "$NAME" >/dev/null
fi

if [ $# -eq 0 ]; then
    set -- bash
fi

if [ -t 0 ]; then
    exec docker exec -it "$NAME" "$@"
else
    exec docker exec -i "$NAME" "$@"
fi
