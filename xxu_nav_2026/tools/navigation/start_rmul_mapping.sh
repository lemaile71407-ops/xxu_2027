#!/usr/bin/env bash
# Start RMUL simulation with Z-Localization mapping and mapless Nav2.
set -euo pipefail
WS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)

gui=true
auto_waypoints=true
auto_save_map=true
save_directory="$WS_DIR/maps/rmul_mapping_$(date +%Y%m%d_%H%M%S)"
overrides=()
extra=()
while (($#)); do
  case "$1" in
    --headless) gui=false; shift ;;
    --manual) auto_waypoints=false; shift ;;
    --no-save) auto_save_map=false; shift ;;
    --save-dir) save_directory=${2:?--save-dir needs a directory}; shift 2 ;;
    --waypoints) overrides+=("waypoints_file:=${2:?--waypoints needs a CSV file}"); shift 2 ;;
    --scan-max-height) overrides+=("scan_max_height:=${2:?--scan-max-height needs metres}"); shift 2 ;;
    --) shift; extra=("$@"); break ;;
    -h|--help)
      echo "Usage: $0 [--headless] [--manual] [--no-save] [--save-dir dir] [--waypoints csv] [--scan-max-height m] [-- launch_arg:=value ...]"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

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
export GZ_PARTITION=${GZ_PARTITION:-xxu_rmul_mapping_$(date +%s)}

config_dir="$WS_DIR/maps/rmul_rotating_20261112"
cd "$WS_DIR"
exec ros2 launch xxu_sim_bringup rmul_mapping.launch.py \
  gazebo_gui:="$gui" rviz:="$gui" \
  auto_waypoints:="$auto_waypoints" auto_save_map:="$auto_save_map" \
  save_directory:="$save_directory" \
  frontend_config:="$config_dir/lio.yaml" backend_config:="$config_dir/backend.yaml" \
  field_model:="$WS_DIR/src/xxu_sim_bringup/models/rmul_2027/model.sdf" \
  rgl_plugin_dir:="$WS_DIR/build/rgl_timed_runtime/install/RGLServerPlugin" \
  "${overrides[@]}" "${extra[@]}"
