Currently this is the install. We will dockerize this later.

# Setup
Install Isaac-Sim 4.0.0 via Omniverse Launcher.

Follow these instructions to setup Nucleus: https://airlab.slite.com/app/docs/X8dZ8w5S3GP9tw

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

Install python dependencies:
```bash
# in this simulation folder
ISAACSIM_PYTHON -m pip install scipy
ISAACSIM_PYTHON -m pip install --editable ./dronekit-python
```


# Run
```bash
# terminal 1
cd Airlab-Autonomy-Stack/simulation
./AscentAeroSystems/launch_ascent_sitl.bash
```
```bash
# terminal 2
cd Airlab-Autonomy-Stack/simulation
ISAACSIM_PYTHON launch_sim.py
```
(`ISAACSIM_PYTHON=~/.local/share/ov/pkg/isaac-sim-4.0.0/python.sh`)

```bash
# terminal 3
cd Airlab-Autonomy-Stack/simulation
rviz2 -d simulation.rviz
```




