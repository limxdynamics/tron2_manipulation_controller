#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -eq 0 ]]; then
  echo "usage: $0 idle|develop|switch [joystick_sim args...]" >&2
  echo "  idle     publish L1+X" >&2
  echo "  develop  publish R1+Right" >&2
  echo "  switch   idle, wait, then develop" >&2
  exit 2
fi

find_bundle() {
  if [[ -x "${script_dir}/joystick_sim" ]]; then
    printf '%s\n' "${script_dir}"
    return 0
  fi
  if [[ -x "${script_dir}/output/joystick_sim" ]]; then
    printf '%s\n' "${script_dir}/output"
    return 0
  fi
  return 1
}

if bundle_dir="$(find_bundle)"; then
  export LD_LIBRARY_PATH="${bundle_dir}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export MROS_LOCALHOST_ONLY="${MROS_LOCALHOST_ONLY:-1}"
  export MROS_SIM_TIME="${MROS_SIM_TIME:-0}"
  export IS_SIM="${IS_SIM:-1}"
  exec "${bundle_dir}/joystick_sim" "$@"
fi

echo "joystick_sim bundle not found." >&2
echo "build first: colcon --log-base build/log build --merge-install \\" >&2
echo "  --build-base build/build --install-base build/install \\" >&2
echo "  --packages-select joystick_sim" >&2
exit 1
