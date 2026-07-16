#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOCAL_PREFIX="${HOME}/.local"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi

if [[ -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/devel/setup.bash"
fi

if [[ -d "${LOCAL_PREFIX}/lib" ]]; then
  export LD_LIBRARY_PATH="${LOCAL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

usage() {
  cat <<'EOF'
Usage:
  ./process.sh 1 INPUT_BAG PREPROCESSED IMAGE_TOPIC POINTS_TOPIC CAMERA_MODEL INTRINSIC DISTORTION
  ./process.sh 1 MAP_PLY IMAGE PREPROCESSED CAMERA_MODEL INTRINSIC DISTORTION
  ./process.sh 2 PREPROCESSED
  ./process.sh 3 PREPROCESSED
  ./process.sh 4 PREPROCESSED

Example:
  ./process.sh 1 data.bag preprocessed /camera/image /lidar/points \
    plumb_bob 810.38,810.28,822.84,622.47 \
    -0.04,0.08,0.001,-0.002,-0.04
  ./process.sh 1 map.ply image.jpg map_preprocessed \
    plumb_bob 2242.45,2242.69,1579.98,1172.42 \
    -0.0272569,0.00207913,-0.00411347,-0.00208982,-0.00291759
  ./process.sh 2 preprocessed
  ./process.sh 3 preprocessed
  ./process.sh 4 preprocessed
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

case "$1" in
  1)
    if [[ $# -eq 7 && "$2" == *.ply ]]; then
      rosrun direct_visual_lidar_calibration preprocess_map \
        --map_path "$2" \
        --image_path "$3" \
        --dst_path "$4" \
        --camera_model "$5" \
        --camera_intrinsics "$6" \
        --camera_distortion_coeffs "$7"
      exit 0
    fi

    if [[ $# -ne 8 ]]; then
      usage
      exit 1
    fi

    rosrun direct_visual_lidar_calibration preprocess \
      "$2" \
      "$3" \
      --image_topic "$4" \
      --points_topic "$5" \
      --camera_model "$6" \
      --camera_intrinsic "$7" \
      --camera_distortion_coeffs "$8"
    ;;
  2)
    if [[ $# -ne 2 ]]; then
      usage
      exit 1
    fi

    rosrun direct_visual_lidar_calibration initial_guess_manual "$2"
    ;;
  3)
    if [[ $# -ne 2 ]]; then
      usage
      exit 1
    fi

    rosrun direct_visual_lidar_calibration viewer "$2"
    ;;
  4)
    if [[ $# -ne 2 ]]; then
      usage
      exit 1
    fi

    rosrun direct_visual_lidar_calibration calibrate "$2"
    ;;
  *)
    usage
    exit 1
    ;;
esac
