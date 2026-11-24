#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo 'Source your ROS 2 installation before running this script.' >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$(dirname -- "$(dirname -- "$script_dir")")"
if [[ "$(basename -- "$source_dir")" != src ]]; then
  echo 'Place ros2_crsf_receiver under <workspace>/src, or run colcon build manually.' >&2
  exit 2
fi

cd -- "$(dirname -- "$source_dir")"
exec colcon build --symlink-install --packages-up-to crsf_receiver "$@"
