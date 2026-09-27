#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$ROOT/src"

clone_at() {
  local url="$1" branch="$2" commit="$3" dst="$4"
  if [[ ! -d "$dst/.git" ]]; then
    git clone --depth 1 --branch "$branch" --recursive "$url" "$dst"
  fi
  if [[ "$(git -C "$dst" rev-parse HEAD)" != "$commit" ]]; then
    git -C "$dst" fetch --depth 1 origin "$commit"
    git -C "$dst" checkout --detach "$commit"
    git -C "$dst" submodule update --init --recursive
  fi
}

clone_at https://github.com/Livox-SDK/livox_ros_driver2.git master \
  21445540f0d100dc86a7e6df312dd70bbdb4afdf "$ROOT/src/livox_ros_driver2"
clone_at https://github.com/Ericsii/FAST_LIO_ROS2.git ros2 \
  2fffc570a25d0df172720bac034fbdb6a13d2162 "$ROOT/src/FAST_LIO_ROS2"

fast_lio="$ROOT/src/FAST_LIO_ROS2"
patch="$ROOT/patches/fast_lio_highrate.patch"
applied_marker="$fast_lio/.mid360s_applied.patch"
if git -C "$fast_lio" apply --unidiff-zero --reverse --check "$patch" 2>/dev/null; then
  echo "FAST-LIO high-rate patch already applied"
  cp "$patch" "$applied_marker"
else
  previous_patch=""
  if [[ -f "$applied_marker" ]] &&
     git -C "$fast_lio" apply --unidiff-zero --reverse --check "$applied_marker" 2>/dev/null; then
    previous_patch="$applied_marker"
  elif git -C "$fast_lio" apply --unidiff-zero --reverse --check \
      "$ROOT/patches/fast_lio_legacy.patch" 2>/dev/null; then
    previous_patch="$ROOT/patches/fast_lio_legacy.patch"
  fi
  if [[ -n "$previous_patch" ]]; then
    git -C "$fast_lio" apply --unidiff-zero --reverse "$previous_patch"
  fi
  if ! git -C "$fast_lio" apply --unidiff-zero --check "$patch"; then
    if [[ -n "$previous_patch" ]]; then
      git -C "$fast_lio" apply --unidiff-zero "$previous_patch"
    fi
    echo "FAST-LIO source has local changes that prevent the project patch" >&2
    exit 1
  fi
  git -C "$fast_lio" apply --unidiff-zero "$patch"
  cp "$patch" "$applied_marker"
fi

cp "$ROOT/config/MID360s_config.json" "$ROOT/src/livox_ros_driver2/config/MID360s_config.json"
cp "$ROOT/src/livox_ros_driver2/package_ROS2.xml" \
  "$ROOT/src/livox_ros_driver2/package.xml"
mkdir -p "$ROOT/src/livox_ros_driver2/launch"
cp "$ROOT/src/livox_ros_driver2/launch_ROS2/"* \
  "$ROOT/src/livox_ros_driver2/launch/"
cp "$ROOT/config/mid360s_bench_extrinsic_estimation.yaml" \
  "$fast_lio/config/mid360s_bench_extrinsic_estimation.yaml"
if [[ -f "$ROOT/config/mid360s_drone.yaml" ]]; then
  cp "$ROOT/config/mid360s_drone.yaml" "$fast_lio/config/mid360s_drone.yaml"
else
  cp "$ROOT/config/mid360s_drone.yaml.template" "$fast_lio/config/mid360s_drone.yaml"
fi

echo "Pinned MID360S ROS2 sources and project overlays installed."
