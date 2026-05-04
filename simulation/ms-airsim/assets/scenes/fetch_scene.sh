#!/usr/bin/env bash
# Fetch and extract a Microsoft AirSim pre-built UE4 scene into
# simulation/ms-airsim/assets/scenes/<Name>/. Idempotent — skips if
# the scene has already been extracted.
#
# Usage: fetch_scene.sh [scene]
#   scene (default: blocks) — one of:
#     blocks, airsimnh, abandonedpark, forest,
#     landscapemountains, soccerfield, building99, zhangjiajie

set -euo pipefail

SCENE="${1:-blocks}"
SCENES_DIR="${SCENES_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
RELEASE_URL="https://github.com/microsoft/AirSim/releases/download/v1.8.1"

case "$SCENE" in
    blocks)             NAME=Blocks;             ZIP=Blocks.zip ;;
    airsimnh)           NAME=AirSimNH;           ZIP=Neighborhood.zip ;;
    abandonedpark)      NAME=AbandonedPark;      ZIP=AbandonedPark.zip ;;
    forest)             NAME=Forest;             ZIP=Forest.zip ;;
    landscapemountains) NAME=LandscapeMountains; ZIP=LandscapeMountains.zip ;;
    soccerfield)        NAME=SoccerField;        ZIP=SoccerField.zip ;;
    building99)         NAME=Building99;         ZIP=Building99.zip ;;
    zhangjiajie)        NAME=ZhangJiajie;        ZIP=ZhangJiajie.zip ;;
    *) echo "unknown scene: $SCENE" >&2; exit 2 ;;
esac

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
