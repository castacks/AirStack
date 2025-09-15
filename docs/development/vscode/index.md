# Visual Studio Code: Docker Integration and Debugger Setup

This guide explains how to set up Visual Studio Code (VSCode) to work with AirStack's Docker containers and use the built-in debugger.

Integration with the Docker containers works by using the [Dev Containers extension](https://code.visualstudio.com/docs/remote/containers). This allows you to open a folder within a container, and use VSCode's features (like the debugger) as if you were working on your local machine.

The two available containers for breakpoint debugging are `robot` and `gcs`.

## Open Container

1. Open AirStack folder in VSCode on your host machine.
2. If it's your first time, VSCode will prompt with a popup to install recommended extensions. Click "Install All". Or, you can manually install the ["Dev Containers" extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
3. Now open the Command Palette (F1) and type "Dev Containers: Reopen in Container". Select "Robot Container" or "GCS Container" depending on which you want to work with.

   This will build the container (if it hasn't been built before) and open the AirStack folder within the container. This may take a few minutes.

   If you want to customize the container (for example, to add additional tools), you can edit `.devcontainer/devcontainer.json` and `.devcontainer/Dockerfile`.

   You can also open a terminal in the container by hitting `Ctrl-`` (backtick).

4. You should now be able to build the workspace and launch the debugger as described below.
5. To switch back to your host machine, you can use the "Dev Containers: Reopen Folder Locally" command from the Command Palette (F1).

## Build ROS Workspace
Hit `Ctrl-Shift-B` to build the project. This is a shortcut for `bws --cmake-args '-DCMAKE_BUILD_TYPE=Debug'`, which adds debug symbols to the build.

Build tasks are defined in `.vscode/tasks.json`.


## Launch and Debug

Debugging uses the [Robot Developers Extension for ROS2](https://ranchhandrobotics.com/rde-ros-2/debug-support.html#). We recommend consulting their docs for more details.

To start debugging, hit `F5` to launch `robot.launch.xml`, or click the "Run and Debug" button on the left side of the screen and click the green play button.

Launch tasks are defined in `.vscode/launch.json`.

![launch](launch.png)

You can now set breakpoints, view variables, step-through code, and debug [as usual in VSCode](https://code.visualstudio.com/docs/editor/debugging).

![debugger](debugger.png)


!!! warning "Warning about file permissions"

    Folders and files created within the attached docker container will be owned by root. This can cause issues when trying to edit files from the host machine, especially when using git to switch branches.
    If you accidentally create files as root, you can change the owner to your user with the following command:
    ```bash
    sudo chown -R $USER:$USER .
    ```