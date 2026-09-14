#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
REPO_ROOT=$(cd "$PROJECT_DIR/.." && pwd)
TRON2_MUJOCO_SIM_BIN="$PROJECT_DIR/tron2_mujoco_sim"
TRON2_MUJOCO_DEFAULT_MODEL="${TRON2_MUJOCO_DEFAULT_MODEL:-$REPO_ROOT/robot_description/tron2a/DACH_TRON2A/xml/robot_grasper.xml}"

export PROJECT_DIR
export REPO_ROOT
export TRON2_MUJOCO_SIM_BIN
export TRON2_MUJOCO_DEFAULT_MODEL
export LD_LIBRARY_PATH="$PROJECT_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MROS_LOCALHOST_ONLY="${MROS_LOCALHOST_ONLY:-1}"
export MROS_SIM_TIME="${MROS_SIM_TIME:-0}"
export IS_SIM="${IS_SIM:-1}"

has_model_source() {
  local arg
  for arg in "$@"; do
    case "$arg" in
      --model|--model=*|--config|--config=*)
        return 0
        ;;
    esac
  done
  return 1
}

has_check_model() {
  local arg
  for arg in "$@"; do
    if [[ "$arg" == "--check-model" ]]; then
      return 0
    fi
  done
  return 1
}

if [[ ! -x "$TRON2_MUJOCO_SIM_BIN" ]]; then
  echo "ERROR: C++ simulator not found: $TRON2_MUJOCO_SIM_BIN" >&2
  exit 1
fi

args=("$@")
if ! has_model_source "${args[@]}"; then
  args=(--model "$TRON2_MUJOCO_DEFAULT_MODEL" "${args[@]}")
fi

if has_check_model "${args[@]}"; then
  exec "$TRON2_MUJOCO_SIM_BIN" "${args[@]}"
fi

"$TRON2_MUJOCO_SIM_BIN" --check-model "${args[@]}"

exec "$TRON2_MUJOCO_SIM_BIN" "${args[@]}"
