#!/bin/bash

set -euo pipefail

module_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_lib_dir="${module_root}/lib"
sdk_include_dir="${module_root}/include/natnet"

sdk_archive_name=""
sdk_download_url=""
temp_dir=""

info() {
    printf '[INFO] %s\n' "$1"
}

warn() {
    printf '[WARN] %s\n' "$1" >&2
}

fail() {
    printf '[ERROR] %s\n' "$1" >&2
    exit 1
}

have() {
    command -v "$1" >/dev/null 2>&1
}

cleanup() {
    if [ -n "$temp_dir" ] && [ -d "$temp_dir" ]; then
        rm -rf "$temp_dir"
    fi
}

trap cleanup EXIT

sdk_is_installed() {
    [ -f "${sdk_lib_dir}/libNatNet.so" ] || return 1
    [ -n "$(find "${sdk_include_dir}" -maxdepth 1 -type f -name 'NatNet*' -print -quit 2>/dev/null)" ]
}

choose_archive() {
    case "$(uname -m)" in
        x86_64)
            sdk_archive_name="NatNet_SDK_4.4_ubuntu.tar"
            sdk_download_url="https://d2mzlempwep3hb.cloudfront.net/NatNetSDKLinux/ubuntu/${sdk_archive_name}"
            info "Using the x86_64 NatNet SDK package"
            ;;
        aarch64|arm*)
            sdk_archive_name="NatNet_SDK_4.4_ubuntu_ARM.tar"
            sdk_download_url="https://d2mzlempwep3hb.cloudfront.net/NatNetSDKLinux/ubuntu_arm/${sdk_archive_name}"
            info "Using the ARM NatNet SDK package"
            ;;
        *)
            fail "Unsupported architecture: $(uname -m)"
            ;;
    esac
}

show_license_notice() {
    cat <<'EOF'
===============================================================================
OptiTrack NatNet SDK License
===============================================================================

The NatNet SDK is used for OptiTrack Motive motion capture integration.
It is only required if you intend to use OptiTrack with AirStack.

The SDK is proprietary and AirStack does not redistribute it.
Please review the OptiTrack terms before continuing:
https://optitrack.com/legal/software-license-agreement

This installer will download the SDK into the natnet_ros2 module tree:
  - lib/libNatNet.so
  - include/natnet/NatNet*

If you do not plan to use OptiTrack, you can skip this step by pressing 'N' or by running:
  airstack setup --no-natnet 

===============================================================================
EOF
}

download_archive() {
    info "Downloading ${sdk_archive_name}"

    if have wget; then
        wget -O "${temp_dir}/${sdk_archive_name}" "${sdk_download_url}"
        return 0
    fi

    if have curl; then
        curl -fsSL -o "${temp_dir}/${sdk_archive_name}" "${sdk_download_url}"
        return 0
    fi

    fail "Neither wget nor curl is available. Install one of them and try again."
}

extract_archive() {
    info "Extracting archive"
    mkdir -p "${temp_dir}/extract"
    tar -xf "${temp_dir}/${sdk_archive_name}" -C "${temp_dir}/extract"
}

install_files() {
    local source_lib
    local source_include

    source_lib="$(find "${temp_dir}/extract" -type f -name 'libNatNet.so' -print -quit)"
    source_include="$(find "${temp_dir}/extract" -type d -name include -print -quit)"

    [ -n "${source_lib}" ] || fail "libNatNet.so was not found inside the SDK archive"
    [ -n "${source_include}" ] || fail "include/ was not found inside the SDK archive"

    mkdir -p "${sdk_lib_dir}" "${sdk_include_dir}"

    info "Installing library and headers into the module tree"
    cp "${source_lib}" "${sdk_lib_dir}/"
    find "${source_include}" -maxdepth 1 -type f -name 'NatNet*' -exec cp -f {} "${sdk_include_dir}/" \;

    [ -f "${sdk_lib_dir}/libNatNet.so" ] || fail "NatNet library copy did not complete"
    [ -n "$(find "${sdk_include_dir}" -maxdepth 1 -type f -name 'NatNet*' -print -quit)" ] || fail "NatNet headers copy did not complete"
}

main() {
    if sdk_is_installed; then
        info "NatNet SDK already installed"
        info "Library: ${sdk_lib_dir}/libNatNet.so"
        info "Headers: ${sdk_include_dir}"
        exit 0
    fi

    choose_archive
    show_license_notice

    read -r -p "Accept the OptiTrack NatNet SDK license and download the SDK now? [Y/n] " reply
    reply="${reply:-y}"

    if ! [[ "${reply}" =~ ^[Yy]$ ]]; then
        warn "NatNet SDK installation skipped"
        exit 1
    fi

    temp_dir="$(mktemp -d "/tmp/natnet-sdk.XXXXXX")"
    download_archive
    extract_archive
    install_files

    info "NatNet SDK installation completed"
    info "Rebuild with: colcon build --packages-select natnet_ros2"
}

main "$@"
