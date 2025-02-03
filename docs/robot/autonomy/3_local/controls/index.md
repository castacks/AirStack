# Controls

Controls dictate the actuation of the robot. They are responsible for taking in sensor data and producing control commands. 

The controller should publish control commands directly to topics defined by the [Robot Interface](../../0_interface/index.md).

Currently the AirStack uses a custom controller called "Trajectory Controller".
