#!/bin/bash

echo "-- Starting AirStack shutdown procedure --"
echo ""

# Ask about WinTAK shutdown
read -p "Would you like to shut down WinTAK? (y/n): " shutdown_wintak

if [[ $shutdown_wintak =~ ^[Yy]$ ]]; then
    echo "Attempting to stop WinTAK virtual machine..."

    # Check if WinTAK is running
    vm_info=$(VBoxManage showvminfo "WinTAK" 2>/dev/null | grep "State:" || echo "")

    if [[ $vm_info == *"running"* ]]; then
        echo "Shutting down WinTAK..."
        # Attempt graceful shutdown first
        if ! VBoxManage controlvm "WinTAK" acpipowerbutton 2>/dev/null; then
            echo "Graceful shutdown failed. Forcing power off..."
            VBoxManage controlvm "WinTAK" poweroff 2>/dev/null
        else
            echo "WinTAK is shutting down gracefully."
            # Wait a bit for the VM to shut down
            echo "Waiting for WinTAK to finish shutting down..."
            sleep 5
        fi
    else
        echo "WinTAK is not currently running."
    fi
else
    echo "WinTAK shutdown skipped."
fi

echo ""

# Check if we're in the AirStack directory
if [ ! -f "docker-compose.yaml" ]; then
    echo "Error: docker-compose.yml not found."
    echo "Please make sure you are in the AirStack directory."
    exit 1
fi

# Stop docker compose services
echo "Stopping Docker Compose services (Isaac Sim and robots)..."
docker compose down

# Revoke docker access to X-Server
echo "Revoking Docker access to X-Server..."
xhost -

echo ""
echo "Docker services have been stopped."
echo ""
echo "-- AirStack shutdown completed successfully --"