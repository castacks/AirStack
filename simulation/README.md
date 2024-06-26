currently:

```
# terminal 1
cd Airlab-Autonomy-Stack/simulation
./AscentAeroSystems/launch_ascent_sitl.bash
```
```
# terminal 2
cd Airlab-Autonomy-Stack/simulation
ISAACSIM_PYTHON launch_sim.py
```
(`ISAACSIM_PYTHON=~/.local/share/ov/pkg/isaac-sim-4.0.0/python.sh`)

```
# terminal 3
cd Airlab-Autonomy-Stack/simulation
rviz2 -d simulation.rviz
```




