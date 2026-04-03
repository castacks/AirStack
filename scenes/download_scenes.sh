#!/bin/bash
# Downloads scene folders from Google Drive into scenes/

FOLDER_ID="1Sap3R2yZUPvcMn88CQ27pmp0TkxJNUi1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check for gdown
if ! command -v gdown &> /dev/null; then
    echo "Installing gdown..."
    if command -v pipx &> /dev/null; then
        pipx install gdown
    else
        pip install -q gdown --break-system-packages
    fi
fi

echo "Downloading scenes from Google Drive..."
gdown --folder "https://drive.google.com/drive/folders/${FOLDER_ID}" -O "$SCRIPT_DIR" --remaining-ok

echo "Done. Scenes downloaded to $SCRIPT_DIR"
