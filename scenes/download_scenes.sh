#!/bin/bash
# Downloads scene folders from Google Drive into scenes/

FILE_ID="1dGlbfeRRWkyVTZVlJdZMl_hcMKjzxEoZ"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZIP_PATH="$SCRIPT_DIR/scenes.zip"

echo "Downloading scenes.zip from Google Drive..."
curl -L "https://drive.usercontent.google.com/download?id=${FILE_ID}&export=download&confirm=t" -o "$ZIP_PATH"

echo "Extracting..."
unzip -o "$ZIP_PATH" -d "$SCRIPT_DIR"

echo "Cleaning up..."
rm "$ZIP_PATH"

echo "Done. Scenes extracted to $SCRIPT_DIR"
