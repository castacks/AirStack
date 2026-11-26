# Camera geometry

The ambient scene delivered left RGB8 and right 32FC1 depth at 480 × 300. Frame IDs were `camera_left` and `camera_right`. Left CameraInfo shared the image frame and dimensions, with fx and fy approximately 184, cx 240 and cy 150. Runtime encoding and matrix values are in [ROS evidence](evidence/ros-domain-1.json).

These observations establish transport and declared calibration fields. Pixel-center convention, physical optical axis orientation, the camera-to-body transform against an independent metrology target, and axial versus ray-depth semantics require controlled reference captures. Actual resolved USD units, stage up axis, clipping and pixel transfer function were not verified. Segmentation was not selected or qualified. The live 480 × 300 scene was kept intact, and the 33 × 25 qualification fixture was not launched.
