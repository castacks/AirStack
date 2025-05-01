#!/usr/bin/env bash

# wintak.sh - WINTAK-related commands for AirStack
# This module provides commands for installing and managing WINTAK

# Function to install WINTAK
function cmd_wintak_install {
    log_info "Installing WINTAK..."
    
    # Create log directory
    local LOG_DIR="$HOME/wintak_setup_logs"
    mkdir -p "$LOG_DIR"
    local LOG_FILE="$LOG_DIR/setup_$(date +%Y%m%d_%H%M%S).log"
    
    # Log function
    function wintak_log {
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
    }
    
    wintak_log "Starting WinTAK VirtualBox setup"
    wintak_log "Setting up Ground Control Station"
    
    # Create vmware directory
    wintak_log "Creating vmware directory..."
    sudo mkdir -p "$HOME/vmware"
    sudo chown -R $USER:$USER "$HOME/vmware"
    sudo chmod -R 755 "$HOME/vmware"
    
    # Create ROS config directories if they don't exist
    local GCS_DIR="$PROJECT_ROOT/ground_control_station"
    mkdir -p "$GCS_DIR/ros_ws/src/ros2tak_tools/config"
    mkdir -p "$GCS_DIR/ros_ws/src/ros2tak_tools/creds"
    
    # Single rsync command to get all necessary files from airlab-storage
    wintak_log "Copying all required files from airlab-storage to $HOME/vmware/..."
    wintak_log "This may take some time depending on your network connection..."
    
    # Prompt for ANDREWID
    read -p "Please enter your Andrew ID: " ANDREWID
    
    # Check if ANDREWID is provided
    if [ -z "$ANDREWID" ]; then
        log_error "Error: Andrew ID cannot be empty"
        return 1
    fi
    
    # Set ANDREWID as environment variable
    export ANDREWID
    sudo rsync --progress -avz ${ANDREWID}@airlab-storage.andrew.cmu.edu:/volume4/dsta/engineering/atak/setup/ "$HOME/vmware/" 2>&1 | tee -a "$LOG_FILE"
    
    # Copy config and creds to ROS workspace
    wintak_log "Copying config and creds to ROS workspace..."
    sudo cp -R "$HOME/vmware/config/"* "$GCS_DIR/ros_ws/src/ros2tak_tools/config/"
    sudo cp -R "$HOME/vmware/creds/"* "$GCS_DIR/ros_ws/src/ros2tak_tools/creds/"
    
    # Set secure permissions on creds directory
    wintak_log "Setting secure permissions on credentials directory..."
    # Go to the creds directory
    pushd "$GCS_DIR/ros_ws/src/ros2tak_tools/creds/" > /dev/null
    # Set restrictive permissions on directories (700: rwx------)
    sudo find . -type d -exec chmod 700 {} \; 2>&1 | tee -a "$LOG_FILE"
    # Set restrictive permissions on files (600: rw-------)
    sudo find . -type f -exec chmod 600 {} \; 2>&1 | tee -a "$LOG_FILE"
    # Ensure proper ownership
    sudo chown -R $USER:$USER . 2>&1 | tee -a "$LOG_FILE"
    wintak_log "Credentials directory secured with restricted permissions"
    # Return to original directory
    popd > /dev/null
    
    # Install VirtualBox with apt logs
    wintak_log "Installing VirtualBox..."
    (sudo dpkg -i "$HOME/vmware/virtualbox-7.1_7.1.6-167084~Ubuntu~jammy_amd64.deb" 2>&1 | tee -a "$LOG_FILE")
    (sudo apt-get install -f -y 2>&1 | tee -a "$LOG_FILE")
    
    # Configure VirtualBox
    wintak_log "Configuring VirtualBox kernel modules..."
    (sudo /sbin/vboxconfig 2>&1 | tee -a "$LOG_FILE")
    
    # Import WinTAK VM
    wintak_log "Importing WinTAK virtual machine..."
    (VBoxManage import "$HOME/vmware/Windows11.ova" --vsys 0 --vmname "WinTAK" 2>&1 | tee -a "$LOG_FILE")
    
    # Start WinTAK VM
    wintak_log "Setup complete! Starting WinTAK..."
    (VBoxManage startvm "WinTAK" 2>&1 | tee -a "$LOG_FILE")
    
    wintak_log "WinTAK setup completed successfully"
    
    log_info "========================================================="
    log_info "  WinTAK is now running in VirtualBox"
    log_info "  To start WinTAK in the future, simply run:"
    log_info "  airstack wintak:start"
    log_info "  Setup logs are available at: $LOG_FILE"
    log_warn "  NOTE: WinTAK will ask you to reset the VM password on first boot. Just choose your own memorable password."
    log_info "========================================================="
    
    return 0
}

# Function to start WINTAK
function cmd_wintak_start {
    log_info "Starting WinTAK..."
    
    # Check if VirtualBox is installed
    if ! command -v VBoxManage &> /dev/null; then
        log_error "VirtualBox is not installed. Run 'airstack install --with-wintak' first."
        return 1
    fi
    
    # Check if WinTAK VM exists
    if ! VBoxManage list vms | grep -q "\"WinTAK\""; then
        log_error "WinTAK VM not found. Run 'airstack install --with-wintak' first."
        return 1
    fi
    
    # Start WinTAK VM
    VBoxManage startvm "WinTAK"
    
    log_info "WinTAK started successfully"
    return 0
}

# Function to stop WINTAK
function cmd_wintak_stop {
    log_info "Stopping WinTAK..."
    
    # Check if VirtualBox is installed
    if ! command -v VBoxManage &> /dev/null; then
        log_error "VirtualBox is not installed."
        return 1
    fi
    
    # Check if WinTAK VM exists
    if ! VBoxManage list vms | grep -q "\"WinTAK\""; then
        log_error "WinTAK VM not found."
        return 1
    fi
    
    # Check if WinTAK VM is running
    if ! VBoxManage list runningvms | grep -q "\"WinTAK\""; then
        log_warn "WinTAK VM is not running."
        return 0
    fi
    
    # Stop WinTAK VM
    VBoxManage controlvm "WinTAK" acpipowerbutton
    
    log_info "WinTAK shutdown initiated"
    return 0
}

# Register commands from this module
function register_wintak_commands {
    COMMANDS["wintak:install"]="cmd_wintak_install"
    COMMANDS["wintak:start"]="cmd_wintak_start"
    COMMANDS["wintak:stop"]="cmd_wintak_stop"
    
    # Add command help
    COMMAND_HELP["wintak:install"]="Install WinTAK VirtualBox environment"
    COMMAND_HELP["wintak:start"]="Start WinTAK virtual machine"
    COMMAND_HELP["wintak:stop"]="Stop WinTAK virtual machine"
}