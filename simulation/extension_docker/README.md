# Extension Docker

## Building the Docker Image

This will build the docker image.

```
./build.sh
```

## Running

This will run the docker container

```
run.sh
```

In docker, you can launch IsaacSim by doing

```
/isaac-sim/runapp.sh
```

In IsaacSim's top menu bar go to Window -> Extensions. In the Extensions window, click "THIRD PARTY". There should be extensions called "SPAWN PRIMITIVES" and "TMUX MANAGER" in the list. Click on them and make sure they are "enabled" and "autoload" is checked. You can close the Extensions window. You should see "Spawn Primitives" and "TMUX Manager" tabs on the bottom right pane of IsaacSim where the "Property" tab is.

In the "Spawn Primitives" tab you can click the "Spawn Ascent" button. This will spawn the drone model and run a tmux session in the background which is running the SITL, mavproxy, and mavros. To attach to this you can click the "TMUX Manager" tab and click the "Refresh" button. This should list the "ascent0" tmux session with a "Attach" and "Kill" buttons. Clicking "Attach" will bring up an xterm window with the tmux session and "Kill" will kill the session. If you kill, you will need to click "Refresh" again and it will no longer be listed.