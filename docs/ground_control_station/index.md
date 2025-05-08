# Ground Control Station

The Ground Control Station (GCS) is for operators to monitor and control the robots.

Requirements: 
- 60GB Hard Disk Space
- min 8GB RAM
- min 4 CPU Cores
- Ubuntu 22.04

## Setup 
WinTAK is setup as auto start on boot and connects to Airlabs TAK-Server. It runs on a Windows 11 VirtualBox Virtual Machine.

![Setup](asset/setup.png)

### Installation using AirStack CLI

The recommended way to install WinTAK is using the AirStack CLI tool:

```bash
# From the AirStack root directory
./airstack.sh install --with-wintak
```

This will:

1. Download the necessary files from airlab-storage
2. Install VirtualBox
3. Import the WinTAK virtual machine
4. Configure the necessary credentials and settings

### Alternative Manual Installation

Alternatively, you can run the setup script directly:

```bash
# Move to the directory:
cd ground_control_station/installation
# Execute the script
./setup_ground_control_station.sh
```

### Starting and Stopping WinTAK

Once installed, you can start and stop WinTAK using the AirStack CLI:

```bash
# Start WinTAK
./airstack.sh wintak:start

# Stop WinTAK
./airstack.sh wintak:stop
```

**NOTE:** If it asks to reset the password on first boot, please choose your own memorable password.

![WinTAK](asset/WinTAK_on_windows_virtualbox_vm.png)

## Know more about TAK using the youtube link below:
[![Video Title](https://img.youtube.com/vi/fiBt0wEiKh8/0.jpg)](https://www.youtube.com/watch?v=fiBt0wEiKh8&t=1s)