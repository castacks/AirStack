# Hardware-In-The-Loop Simulation
We configure a multi-machine HITL simulation, where a powerful desktop computer runs Isaac Simulator and rendering, and one/multiple jetson compute boards run robot-specific programs (planning, mapping, etc.).
## Requirement
A desktop computer configured according to [here](/docs/getting_started). One/multiple ORIN AGX/NX configured according to [here](/docs/real_world/installation/).

## Communication
All machines should connect to the same network. In our test, all machines are connected to the same router with ethernet cables. Ensure that all machines are able to `ping` others' IP addresses.

### Run
On the desktop computer, under your Airstack folder, run
```
docker compose up isaac-sim-hitl
```
You should see the isaac simulator being launched.
On the Jetson computer, run
```
docker compose up robot_l4t
```
Once the scene is played in the Isaac simulator, the rviz GUI on the Jetson should start displaying sensor data, which means the connection is successful. 

Screen record of desktop computer:
<iframe src="https://drive.google.com/file/d/1sNkEattgDyBAI9xFPVQsXn8sRi6gYQSG/view?usp=sharing" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>

Screen record of Jetson computer:
<iframe src="https://drive.google.com/file/d/19S8Yceq8t2FPubN8Mly003Up1AbLzCA8/view?usp=sharing" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>