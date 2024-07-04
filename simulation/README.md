Currently this is the install. We will dockerize this later.

# Setup
Install ROS2-humble.

Install Isaac-Sim 4.0.0 via Omniverse Launcher. To install the Omniverse launcher download from this link:
``` 
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
```


Follow these instructions to setup Nucleus : https://airlab.slite.com/app/docs/X8dZ8w5S3GP9tw



If you are using the Ascent Spirit drone download the SITL software packages from this link:
https://drive.google.com/file/d/1UxgezaTrHe4WJ28zsVeRhv1VYfOU5VK8/view?usp=drive_link

Then unzip the file  AscentAeroSystemsSITLPackage.zip  in this folder:
```
cd AirLab-Autonomy-Stack/simulation/AscentAeroSystems
unzip AscentAeroSystemsSITLPackage.zip 
```

Setup the SITL with a comm link in "Application Settings" with the following settings:
- TCP
- Server: localhost:5760



# Install

Append to your `.bashrc` or `.zshrc`:
```bash
# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-4.0.0"
# Isaac Sim python executable
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
# Isaac Sim app
alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

```

Install python dependencies for mavlink:
```bash
# in this simulation folder
ISAACSIM_PYTHON -m pip install scipy
ISAACSIM_PYTHON -m pip install --editable ./dronekit-python
sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user

```


# Run
```bash
# terminal 1
cd Airlab-Autonomy-Stack/simulation
./AscentAeroSystems/launch_ascent_sitl.bash
```
Make sure the SITL simulation is running successfully before trying to start IssacSim. Otherwise IssacSim will hang and crash with an error similar to this:
```
024-07-04 20:11:43 [43,593ms] [Warning] [dronekit] Link timeout, no heartbeat in last 5 seconds
2024-07-04 20:12:08 [68,589ms] [Error] [dronekit.mavlink] Exception in MAVLink input loop
```


```bash
# terminal 2
cd Airlab-Autonomy-Stack/simulation
ISAACSIM_PYTHON launch_sim.py
```

```bash
# terminal 3
cd Airlab-Autonomy-Stack/simulation
rviz2 -d simulation.rviz
```




