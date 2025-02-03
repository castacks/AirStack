# Robot

## Launch Structure
Each high-level module has a `*_bringup` package that contains the launch files for that module. The launch files are located in the `launch` directory of the `*_bringup` package. The launch files are named `*.launch.(xml/yaml/py)` and can be launched with `ros2 launch <module_name>_bringup <module_name>.launch.(xml/yaml/py)`.
