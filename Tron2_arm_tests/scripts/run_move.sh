#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)

cd "$PROJECT_DIR"
exec python3 -m dual_arm_protocol_tests \
  --cases "$PROJECT_DIR/config/move_cases.yaml" \
  --output-dir "$PROJECT_DIR/results/move" \
  "$@"
