# RQT Python AirstackControlPanel

If you `colcon build` this package in a workspace and then run "rqt --force-discover" after sourcing the workspace, the plugin should show up as "Airstack Control Panel" in "Miscellaneous Tools" in the "Plugins" menu.

You can use the `generate_rqt_py_package.sh` script to generate a new package by doing the following from the rqt_airstack_control_panel directory

```
./generate_rqt_py_package.sh [package name] [class name] [plugin title]
```

[package name] will be the name of the package and a directory with this name will be created above `rqt_airstack_control_panel/`. [class name] is the name of the class in `src/[package name]/template.py`. [plugin title] is what the plugin will be called in the "Miscellaneous Tools" menu.

For example,

```
cd rqt_airstack_control_panel/
./generate_rqt_py_package.sh new_rqt_package ClassName "Plugin Title"
```

