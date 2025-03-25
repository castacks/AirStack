
# **Gimbal Extension**

## **Overview**  
The **Gimbal Extension** provides an easy way to integrate a controllable gimbal into an existing drone model within the scene. This extension is designed to facilitate the attachment and operation of a camera-equipped gimbal, allowing for real-time adjustments to pitch and yaw angles via ROS 2 messages.


## **Installation and Activation**  
To enable the **Gimbal Extension**, follow these steps:

1. Open the **Extensions** window by navigating to:  
   **Window** → **Extensions**
2. Under the **THIRD PARTIES** section, go to the **User** tab.
3. Locate the **Gimbal Extension** and turn it on.
4. Once enabled, a new **Gimbal Extension** window should appear.

## **Adding a Gimbal to a Drone**  
To attach a gimbal to an existing UAV model:

1. Copy the **prim path** of the UAV to which you want to add the gimbal.
2. In the **Gimbal Extension** window, paste the copied path into the **Robot Prim Path** text box.
3. Set the **Robot Index** based on the `DOMAIN_ID` of the drone.  
   - The `DOMAIN_ID` should match the identifier used for the robot to ensure proper communication.

For a step-by-step demonstration, refer to the video tutorial below:

<iframe src="https://drive.google.com/file/d/1pN0Pxe4nYQL1qs40oZTDqsOXsMrApLPM/preview" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>

## **Gimbal Camera Image Topic**
Once the gimbal is successfully added, the camera image feed from the gimbal will be published on the following ROS 2 topic: `/robot_<ID>/gimbal/rgb`.

## **Controlling the Gimbal**
The gimbal pitch and yaw angles can be controled by the ros2 messages `/robot_<ID>/gimbal/desired_gimbal_pitch` and `/robot_<ID>/gimbal/desired_gimbal_yaw` of type `std_msgs/msg/Float64`, respectively.