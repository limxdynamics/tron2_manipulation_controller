#!/usr/bin/env bash

# Minimal setup for tron2a_manipulation_controllers_sim.launch.
# Source this file before running mroslaunch from install.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "ERROR: source this script instead of executing it:" >&2
  echo "  source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

INSTALL_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export MROS_PKG_PATH="$INSTALL_DIR"
export MROS_BIN_PATH="$INSTALL_DIR/bin"
export MROS_LIB_PATH="$INSTALL_DIR/lib"
export MROS_ETC_PATH="$INSTALL_DIR/etc"
export PATH="$INSTALL_DIR/bin:${PATH:-}"
if [[ "${TRON2_KEEP_EXTERNAL_LD_LIBRARY_PATH:-0}" == "1" ]]; then
  export LD_LIBRARY_PATH="$INSTALL_DIR/lib:${LD_LIBRARY_PATH:-}"
else
  export LD_LIBRARY_PATH="$INSTALL_DIR/lib"
fi
export IS_SIM="${IS_SIM:-1}"
export MROS_LOCALHOST_ONLY="${MROS_LOCALHOST_ONLY:-1}"
export MROS_SIM_TIME="${MROS_SIM_TIME:-0}"

echo "TRON2 install loaded: $INSTALL_DIR"
