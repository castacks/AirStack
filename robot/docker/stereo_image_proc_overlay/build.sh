#!/usr/bin/env bash
# Build the pinned external ROS dependency into a new, isolated install prefix.
set -eo pipefail
[[ $# -eq 1 && "$1" = /* && "$1" != / ]] || {
  echo 'Usage: bash build.sh /absolute/new/install-prefix' >&2
  exit 2
}
rrm_stereo_prefix="$1"
[[ ! -e "$rrm_stereo_prefix" ]] || {
  echo "Refusing to overwrite existing prefix: $rrm_stereo_prefix" >&2
  exit 2
}
rrm_stereo_scripts=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
readonly rrm_stereo_revision=6c3df3099bc7b1ec92215719215c8eefd0d3aa69
readonly rrm_stereo_archive_sha256=313d097f361918c0e0c7a547c95b83b8a8deba8cb1a3b4f5909ed55a3a9d2582
rrm_stereo_build=$(mktemp -d /tmp/airstack-stereo-build.XXXXXX)
curl --fail --location --silent --show-error --retry 2 --max-time 120 \
  "https://codeload.github.com/ros-perception/image_pipeline/tar.gz/$rrm_stereo_revision" \
  --output "$rrm_stereo_build/source.tar.gz"
echo "$rrm_stereo_archive_sha256  $rrm_stereo_build/source.tar.gz" | sha256sum --check
tar -xzf "$rrm_stereo_build/source.tar.gz" -C "$rrm_stereo_build"
rrm_stereo_source="$rrm_stereo_build/image_pipeline-$rrm_stereo_revision"
git -C "$rrm_stereo_source" apply --check "$rrm_stereo_scripts/valid_window.patch"
git -C "$rrm_stereo_source" apply "$rrm_stereo_scripts/valid_window.patch"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd "$rrm_stereo_build"
colcon build --base-paths "$rrm_stereo_source/stereo_image_proc" \
  --merge-install --install-base "$rrm_stereo_prefix" --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --parallel-workers 1
python3 "$rrm_stereo_scripts/provenance.py" "$rrm_stereo_prefix" \
  "$rrm_stereo_revision" "$rrm_stereo_archive_sha256" \
  "$rrm_stereo_scripts/valid_window.patch" \
  "$rrm_stereo_source/stereo_image_proc/src/stereo_image_proc/disparity_node.cpp"
echo "Built stereo overlay at $rrm_stereo_prefix (build/source retained at $rrm_stereo_build)"
