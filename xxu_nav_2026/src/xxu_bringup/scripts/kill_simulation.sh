#!/bin/bash
# Stop only simulation/mapping launch trees belonging to this workspace.
set -euo pipefail
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
# A source checkout works without sourcing the installation first.
if [ -d "$script_dir/../xxu_bringup" ]; then
  export PYTHONPATH="$script_dir/..${PYTHONPATH:+:$PYTHONPATH}"
fi
exec python3 -m xxu_bringup.simulation_processes "$@"
