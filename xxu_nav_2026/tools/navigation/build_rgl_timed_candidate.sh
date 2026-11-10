#!/usr/bin/env bash
# Build the opt-in timing candidate from the pinned upstream commit and project patch.
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
plugin="$workspace/src/rm_sim_26/external/RGLGazeboPlugin"
patch_file="$workspace/tools/navigation/rgl_timed_scan.patch"
expected_revision=d4bf3cf36fe4a363a56df1bec2ce3809720db563
base_library="$workspace/install/rgl_plugin/RGLServerPlugin/libRobotecGPULidar.so"
base_headers="$workspace/build/rgl_plugin/external/RobotecGPULidar/include"

if [[ ! -f "$base_library" || ! -f "$base_headers/rgl/api/core.h" ]]; then
  echo 'Build the pinned RGL plugin first with tools/navigation/build_rgl.sh.' >&2
  exit 1
fi
if [[ "$(git -C "$plugin" rev-parse HEAD)" != "$expected_revision" ]]; then
  echo 'The RGL timing patch requires the pinned RGLGazeboPlugin revision.' >&2
  exit 1
fi
if [[ $# -eq 0 ]]; then
  candidate_dir="$(mktemp -d "${TMPDIR:-/tmp}/xxu-rgl-timed-XXXXXXXX")"
elif [[ $# -eq 2 && "$1" == '--output-dir' ]]; then
  # Refuse to overwrite an existing build or a symlink supplied by the caller.
  if [[ -e "$2" || -L "$2" ]]; then
    echo 'The output directory must not already exist.' >&2
    exit 1
  fi
  mkdir -p -- "$2"
  candidate_dir="$(cd -- "$2" && pwd)"
else
  echo 'Usage: build_rgl_timed_candidate.sh [--output-dir NEW_DIRECTORY]' >&2
  exit 1
fi
mkdir -p "$candidate_dir/source"
git -C "$plugin" archive "$expected_revision" | tar -x -C "$candidate_dir/source"
# A workspace-local output must not discover the enclosing project repository.
GIT_CEILING_DIRECTORIES="$candidate_dir" git -C "$candidate_dir/source" apply --check "$patch_file"
GIT_CEILING_DIRECTORIES="$candidate_dir" git -C "$candidate_dir/source" apply "$patch_file"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
cmake -S "$candidate_dir/source" -B "$candidate_dir/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$candidate_dir/install" \
  -DRGL_CUSTOM_LIBRARY_PATH="$base_library" -DRGL_CUSTOM_API_HEADER_PATH="$base_headers"
cmake --build "$candidate_dir/build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "$candidate_dir/build"
echo "RGL timing candidate: $candidate_dir/install/RGLServerPlugin"
