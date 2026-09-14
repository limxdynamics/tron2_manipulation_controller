#!/usr/bin/env bash

set -euo pipefail

INSTALL_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$INSTALL_DIR/local_setup.bash"

exec mroslaunch "tron2_controllers/tron2a_manipulation_controllers_sim.launch"
