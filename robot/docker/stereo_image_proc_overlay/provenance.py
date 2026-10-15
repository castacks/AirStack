"""Record the compiled dependency's immutable source and binary identities."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


prefix = Path(sys.argv[1])
ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
dependencies = [f'ros-{ros_distro}-{name}' for name in
                ['image-geometry', 'image-proc', 'image-transport', 'cv-bridge', 'rclcpp']]
dependencies.extend(['libopencv-dev', 'libstdc++6'])
record = {
    'schema_version': 'airstack-stereo-overlay/v1',
    'upstream_repository': 'https://github.com/ros-perception/image_pipeline',
    'upstream_version': '5.0.13',
    'upstream_commit': sys.argv[2],
    'upstream_archive_sha256': sys.argv[3],
    'patch_sha256': sha256(sys.argv[4]),
    'patched_disparity_source_sha256': sha256(sys.argv[5]),
    'library_sha256': sha256(prefix / 'lib/libstereo_image_proc.so'),
    'executable_sha256': sha256(prefix / 'lib/stereo_image_proc/disparity_node'),
    'architecture': platform.machine(),
    'ros_distro': ros_distro,
    'compiler': subprocess.check_output(['c++', '--version'], text=True).splitlines()[0],
    'dependency_versions': subprocess.check_output(
        ['dpkg-query', '-W', '-f=${Package}=${Version}\n', *dependencies], text=True).splitlines(),
}
(prefix / 'airstack-stereo-provenance.json').write_text(
    json.dumps(record, indent=2, sort_keys=True) + '\n')
