#!/bin/bash

# Exit on error
set -e

# Display banner
echo "========================================================="
echo "        WinTAK VirtualBox Setup Script"
echo "========================================================="

# Create log directory
LOG_DIR="$HOME/wintak_setup_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/setup_$(date +%Y%m%d_%H%M%S).log"

# Log function
log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

log "Starting WinTAK VirtualBox setup"

# Prompt for ANDREWID
read -p "Please enter your Andrew ID: " ANDREWID

# Check if ANDREWID is provided
if [ -z "$ANDREWID" ]; then
    log "Error: Andrew ID cannot be empty"
    exit 1
fi

# Set ANDREWID as environment variable
export ANDREWID
log "Setting up Ground Control Station for Andrew ID: $ANDREWID"

# Create vmware directory
log "Creating vmware directory..."
sudo mkdir -p "$HOME/vmware"
sudo chown -R $USER:$USER "$HOME/vmware"
sudo chmod -R 755 "$HOME/vmware"

# Create ROS config directories if they don't exist
mkdir -p ../ros_ws/src/ros2tak_tools/config
mkdir -p ../ros_ws/src/ros2tak_tools/creds

# Single rsync command to get all necessary files from airlab-storage
log "Copying all required files from airlab-storage to $HOME/vmware/..."
log "This may take some time depending on your network connection..."
sudo rsync --progress -avz ${ANDREWID}@airlab-storage.andrew.cmu.edu:/volume4/dsta/atak/setup/ "$HOME/vmware/" 2>&1 | tee -a "$LOG_FILE"

# Copy config and creds to ROS workspace
log "Copying config and creds to ROS workspace..."
sudo cp -R "$HOME/vmware/config/"* ../ros_ws/src/ros2tak_tools/config/
sudo cp -R "$HOME/vmware/creds/"* ../ros_ws/src/ros2tak_tools/creds/

# Set secure permissions on creds directory
log "Setting secure permissions on credentials directory..."
# Go to the creds directory
pushd ../ros_ws/src/ros2tak_tools/creds/ > /dev/null
# Set restrictive permissions on directories (700: rwx------)
sudo find . -type d -exec chmod 700 {} \; 2>&1 | tee -a "$LOG_FILE"
# Set restrictive permissions on files (600: rw-------)
sudo find . -type f -exec chmod 600 {} \; 2>&1 | tee -a "$LOG_FILE"
# Ensure proper ownership
sudo chown -R $USER:$USER . 2>&1 | tee -a "$LOG_FILE"
log "Credentials directory secured with restricted permissions"
# Return to original directory
popd > /dev/null

# Install VirtualBox with apt logs
log "Installing VirtualBox..."
(sudo dpkg -i "$HOME/vmware/virtualbox-7.1_7.1.6-167084~Ubuntu~jammy_amd64.deb" 2>&1 | tee -a "$LOG_FILE")
(sudo apt-get install -f -y 2>&1 | tee -a "$LOG_FILE")

# Configure VirtualBox
log "Configuring VirtualBox kernel modules..."
(sudo /sbin/vboxconfig 2>&1 | tee -a "$LOG_FILE")

# Import WinTAK VM
log "Importing WinTAK virtual machine..."
(VBoxManage import "$HOME/vmware/Windows11.ova" --vsys 0 --vmname "WinTAK" 2>&1 | tee -a "$LOG_FILE")

# Start WinTAK VM
log "Setup complete! Starting WinTAK..."
(VBoxManage startvm "WinTAK" 2>&1 | tee -a "$LOG_FILE")

log "WinTAK setup completed successfully"

echo "========================================================="
echo "  WinTAK is now running in VirtualBox"
echo "  To start WinTAK in the future, simply run:"
echo "  VBoxManage startvm \"WinTAK\""
echo "  Setup logs are available at: $LOG_FILE"
echo "========================================================="