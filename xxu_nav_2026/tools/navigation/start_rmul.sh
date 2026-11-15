#!/usr/bin/env bash
# Reproducible RMUL simulation; no packages or binaries are loaded from log/.
set -euo pipefail
WS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
profile=""; gui=true; spin=false; dynamic_vehicle=false; idle_scan=false; extra=()
while (($#)); do
  case "$1" in
    --profile) profile=${2:?--profile needs baseline or tested_sim}; shift 2 ;;
    --headless) gui=false; shift ;;
    --spin) spin=true; shift ;;
    --dynamic-vehicle) dynamic_vehicle=true; shift ;;
    --idle-scan) idle_scan=true; shift ;;
    --) shift; extra=("$@"); break ;;
    -h|--help) echo "Usage: $0 --profile baseline|tested_sim [--headless] [--spin] [--idle-scan] [--dynamic-vehicle] [-- launch_arg:=value ...]"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
case "$profile" in
  baseline) sampling=direct; backend=backend.yaml ;;
  tested_sim) sampling=interval_mean; backend=backend_experimental.yaml ;;
  *) echo 'Explicitly choose --profile baseline or --profile tested_sim (experimental backend).' >&2; exit 2 ;;
esac
if [[ "$spin" == true && "$profile" != tested_sim ]]; then
  echo '--spin is an explicit tested_sim trial only.' >&2; exit 2
fi
# Reset package/library lookup inside this child shell, preserving the caller's environment.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u
if [[ -d "$WS_DIR/../z-localization/deps/lib" ]]; then
  export LD_LIBRARY_PATH="$WS_DIR/../z-localization/deps/lib:${LD_LIBRARY_PATH:-}"
fi
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-90}
export GZ_PARTITION=${GZ_PARTITION:-xxu_vehicle_check_rmul_$(date +%s)}
map="$WS_DIR/maps/rmul_rotating_20261112"
geometry="$WS_DIR/src/xxu_sim_bringup/models/ega_folded/geometry.json"
nav_params="$map/navigation.yaml"
if [[ "$profile" == tested_sim || "$idle_scan" == true ]]; then
  geometry_dir=$(mktemp -d /tmp/xxu_rmul_profile.XXXXXX)
  trap 'rm -rf -- "$geometry_dir"' EXIT
  if [[ "$profile" == tested_sim ]]; then
    cp "$geometry" "$geometry_dir/geometry.json"
    cp "$WS_DIR/tools/navigation/yaw_stability_profile.yaml" "$geometry_dir/navigation_profile.yaml"
    geometry="$geometry_dir/geometry.json"
  fi
  if [[ "$spin" == true || "$idle_scan" == true ]]; then
    nav_params="$geometry_dir/navigation_spin.yaml"
    python3 - "$map/navigation.yaml" "$nav_params" "$spin" "$idle_scan" <<'PY'
import sys, yaml
from xxu_bringup.navigation_parameters import continuous_spin_parameters
with open(sys.argv[1]) as handle:
    params = yaml.safe_load(handle)
if sys.argv[3] == "true":
    params = continuous_spin_parameters(params)
if sys.argv[4] == "true":
    params["behavior_server"]["ros__parameters"]["max_rotational_vel"] = 0.5
    smoother = params["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"][2] = 0.5
    smoother["min_velocity"][2] = -0.5
with open(sys.argv[2], 'w') as handle:
    yaml.safe_dump(params, handle, sort_keys=False)
PY
  fi
fi
printf 'RMUL profile=%s, IMU=%s, backend=%s, spin=%s, ROS_DOMAIN_ID=%s\n' "$profile" "$sampling" "$backend" "$spin" "$ROS_DOMAIN_ID"
cd "$WS_DIR"
ros2 launch xxu_sim_bringup rmul_navigation.launch.py \
  gazebo_gui:="$gui" rviz:="$gui" imu_sampling:="$sampling" dynamic_test_vehicle:="$dynamic_vehicle" \
  idle_scan:="$idle_scan" \
  frontend_config:="$map/lio.yaml" backend_config:="$map/$backend" \
  prior_map:="$map/prior" external_map:="$map/nav2.yaml" \
  external_nav2_params:="$nav_params" robot_geometry_file:="$geometry" \
  field_model:="$WS_DIR/src/xxu_sim_bringup/models/rmul_2027/model.sdf" \
  rgl_plugin_dir:="$WS_DIR/build/rgl_timed_runtime/install/RGLServerPlugin" "${extra[@]}"
