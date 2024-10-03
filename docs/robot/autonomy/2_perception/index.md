# Perception
These modules process raw sensor data into useful information for the robot. For example: for detecting obstacles, localizing the robot, and recognizing objects.

Perception modules typically output topics in image space or point cloud space. This information then gets aggregated into global and local world models later in the pipeline.

Common perception modules include:

- semantic segmentation
- VIO (Visual Inertial Odometry)