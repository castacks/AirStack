#!/usr/bin/env bash
# Fetch and extract a Microsoft AirSim pre-built UE4 scene into
# simulation/ms-airsim/assets/scenes/<Name>/. Idempotent — skips if
# the scene has already been extracted.
#
# Usage: fetch_scene.sh [scene]
#        fetch_scene.sh --name <scene>   # print the scene's directory name
#                                        # (e.g. blocks → Blocks) and exit
#   scene (default: blocks) — one of:
#     blocks, airsimnh, abandonedpark, landscapemountains,
#     zhangjiajie, africasavannah, msbuild2018
#   (only assets that actually exist in the v1.8.1 release; Building_99.zip
#   is published but 0 bytes, and Forest/SoccerField were never released)

set -euo pipefail

NAME_ONLY=""
if [ "${1:-}" = "--name" ]; then
    NAME_ONLY=1
    shift
fi
SCENE="${1:-blocks}"
SCENES_DIR="${SCENES_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
RELEASE_URL="https://github.com/microsoft/AirSim/releases/download/v1.8.1"

case "$SCENE" in
    blocks)             NAME=Blocks;             ZIP=Blocks.zip ;;
    airsimnh)           NAME=AirSimNH;           ZIP=AirSimNH.zip ;;
    abandonedpark)      NAME=AbandonedPark;      ZIP=AbandonedPark.zip ;;
    landscapemountains) NAME=LandscapeMountains; ZIP=LandscapeMountains.zip ;;
    zhangjiajie)        NAME=ZhangJiajie;        ZIP=ZhangJiajie.zip ;;
    africasavannah)     NAME=Africa_Savannah;    ZIP=Africa_Savannah.zip ;;
    msbuild2018)        NAME=MSBuild2018;        ZIP=MSBuild2018.zip ;;
    *) echo "unknown scene: $SCENE" >&2; exit 2 ;;
esac

if [ -n "$NAME_ONLY" ]; then
    echo "$NAME"
    exit 0
fi

DEST="$SCENES_DIR/$NAME"
if compgen -G "$DEST/LinuxNoEditor/*.sh" > /dev/null; then
    echo "$NAME already present in $DEST"
    exit 0
fi

mkdir -p "$DEST"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Downloading $ZIP..."
curl -fL "$RELEASE_URL/$ZIP" -o "$TMP/$ZIP"
echo "Extracting to $DEST..."
# Zips wrap everything in a single top-level dir (e.g. LinuxBlocks1.8.1/).
# Extract to a tmp location, then lift the wrapper's contents into $DEST.
unzip -q "$TMP/$ZIP" -d "$TMP/extract"
shopt -s dotglob
mv "$TMP/extract"/*/* "$DEST/"
find "$DEST/LinuxNoEditor" -maxdepth 1 -name "*.sh" -exec chmod +x {} \;
echo "Ready: $DEST"
