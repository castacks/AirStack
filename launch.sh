#!/bin/bash

# Allow docker access to X-Server
echo "Allowing Docker access to X-Server..."
xhost +

# Check if we're in the AirStack directory
if [ ! -f "docker-compose.yaml" ]; then
    echo "Error: docker-compose.yml not found."
    echo "Please make sure you are in the AirStack directory."
    exit 1
fi

# Start docker compose services
echo "Starting Docker Compose services (Isaac Sim and robots)..."
docker compose up -d

echo ""
echo "Docker services are now running."
echo ""

# Display information about WinTAK
echo "To learn more about WinTAK, visit: https://www.youtube.com/watch?v=fiBt0wEiKh8"
echo "AirStack uses WinTAK for human-robot team awareness and collaboration in mission environments."

# Ask about WinTAK
read -p "Would you like to run WinTAK? (y/n): " run_wintak

if [[ $run_wintak =~ ^[Yy]$ ]]; then
    echo "Attempting to start WinTAK virtual machine..."

    # Check if WinTAK is already running
    vm_info=$(VBoxManage showvminfo "WinTAK" 2>/dev/null | grep "State:" || echo "")

    if [[ $vm_info == *"running"* ]]; then
        echo "WinTAK is already running. Please check your windows."
    # If not running, try to start the VM
    elif ! VBoxManage startvm "WinTAK" 2>/dev/null; then
        echo "Failed to start WinTAK. It may not be set up properly."
        read -p "Would you like to set up WinTAK now? (y/n): " setup_wintak

        if [[ $setup_wintak =~ ^[Yy]$ ]]; then
            # Check if setup script exists
            if [ -f "ground_control_station/installation/setup_ground_control_station.sh" ]; then
                echo "Running WinTAK setup script..."
                bash ground_control_station/installation/setup_ground_control_station.sh
            else
                echo "Error: Setup script not found at ground_control_station/installation/setup_ground_control_station.sh"
                exit 1
            fi
        else
            echo "WinTAK setup skipped."
        fi
    else
        echo "WinTAK virtual machine started successfully."
    fi
else
    echo "WinTAK startup skipped."
fi

echo "-------------------------------------------------------------"
echo "  █████  ██ ██████  ███████ ████████  █████   ██████ ██   ██ "
echo " ██   ██ ██ ██   ██ ██         ██    ██   ██ ██      ██  ██  "
echo " ███████ ██ ██████  ███████    ██    ███████ ██      █████   "
echo " ██   ██ ██ ██   ██      ██    ██    ██   ██ ██      ██  ██  "
echo " ██   ██ ██ ██   ██ ███████    ██    ██   ██  ██████ ██   ██ "
echo " ------------- Launched AirStack Successfully ---------------"