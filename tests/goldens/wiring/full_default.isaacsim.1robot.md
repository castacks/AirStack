# Wiring snapshot: full_default

- **generated-by**: tests/system/test_wiring_snapshot.py
- **date**: 2026-08-21 00:22:30
- **sim**: isaacsim
- **num_robots**: 1
- **source-sha**: 19ec8184f64d

```mermaid
graph LR
  subgraph g0["behavior"]
    n2["/robot_1/behavior/drone_safety_monitor/drone_safety_monitor"]
  end
  subgraph g1["control"]
    n3["/robot_1/control/pid_controller"]
  end
  subgraph g2["droan"]
    n4["/robot_1/droan/disparity_expander_node"]
  end
  subgraph g3["interface"]
    n6["/robot_1/interface/mavros/actuator_control"]
    n7["/robot_1/interface/mavros/adsb"]
    n8["/robot_1/interface/mavros/altitude"]
    n9["/robot_1/interface/mavros/cam_imu_sync"]
    n10["/robot_1/interface/mavros/camera"]
    n11["/robot_1/interface/mavros/cellular_status"]
    n12["/robot_1/interface/mavros/cmd"]
    n13["/robot_1/interface/mavros/companion_process"]
    n14["/robot_1/interface/mavros/debug_value"]
    n15["/robot_1/interface/mavros/esc_status"]
    n16["/robot_1/interface/mavros/esc_telemetry"]
    n17["/robot_1/interface/mavros/fake_gps"]
    n18["/robot_1/interface/mavros/ftp"]
    n19["/robot_1/interface/mavros/geofence"]
    n20["/robot_1/interface/mavros/gimbal_control"]
    n21["/robot_1/interface/mavros/global_position"]
    n22["/robot_1/interface/mavros/gps_input"]
    n23["/robot_1/interface/mavros/gps_rtk"]
    n24["/robot_1/interface/mavros/gpsstatus"]
    n25["/robot_1/interface/mavros/guided_target"]
    n26["/robot_1/interface/mavros/hil"]
    n27["/robot_1/interface/mavros/home_position"]
    n28["/robot_1/interface/mavros/imu"]
    n29["/robot_1/interface/mavros/landing_target"]
    n30["/robot_1/interface/mavros/local_position"]
    n31["/robot_1/interface/mavros/log_transfer"]
    n32["/robot_1/interface/mavros/mag_calibration"]
    n33["/robot_1/interface/mavros/manual_control"]
    n34["/robot_1/interface/mavros/mavros"]
    n35["/robot_1/interface/mavros/mavros_node"]
    n36["/robot_1/interface/mavros/mavros_router"]
    n37["/robot_1/interface/mavros/mission"]
    n38["/robot_1/interface/mavros/mocap"]
    n39["/robot_1/interface/mavros/mount_control"]
    n40["/robot_1/interface/mavros/nav_controller_output"]
    n41["/robot_1/interface/mavros/obstacle"]
    n42["/robot_1/interface/mavros/obstacle_distance_3d"]
    n43["/robot_1/interface/mavros/odometry"]
    n44["/robot_1/interface/mavros/onboard_computer"]
    n45["/robot_1/interface/mavros/open_drone_id"]
    n46["/robot_1/interface/mavros/optical_flow"]
    n47["/robot_1/interface/mavros/param"]
    n48["/robot_1/interface/mavros/play_tune"]
    n49["/robot_1/interface/mavros/px4flow"]
    n50["/robot_1/interface/mavros/rallypoint"]
    n51["/robot_1/interface/mavros/rc"]
    n52["/robot_1/interface/mavros/setpoint_accel"]
    n53["/robot_1/interface/mavros/setpoint_attitude"]
    n54["/robot_1/interface/mavros/setpoint_position"]
    n55["/robot_1/interface/mavros/setpoint_raw"]
    n56["/robot_1/interface/mavros/setpoint_trajectory"]
    n57["/robot_1/interface/mavros/setpoint_velocity"]
    n58["/robot_1/interface/mavros/sim_state"]
    n59["/robot_1/interface/mavros/sys"]
    n60["/robot_1/interface/mavros/tdr_radio"]
    n61["/robot_1/interface/mavros/terrain"]
    n62["/robot_1/interface/mavros/time"]
    n63["/robot_1/interface/mavros/trajectory"]
    n64["/robot_1/interface/mavros/tunnel"]
    n65["/robot_1/interface/mavros/vfr_hud"]
    n66["/robot_1/interface/mavros/vision_pose"]
    n67["/robot_1/interface/mavros/vision_speed"]
    n68["/robot_1/interface/mavros/wind"]
    n69["/robot_1/interface/odom_modifier"]
    n70["/robot_1/interface/robot_interface"]
  end
  subgraph g4["odometry_conversion"]
    n71["/robot_1/odometry_conversion/odometry_conversion"]
  end
  subgraph g5["perception"]
    n72["/robot_1/perception/stereo_image_proc/disparity_node"]
    n73["/robot_1/perception/stereo_pointcloud"]
  end
  subgraph g6["robot_1"]
    n1["/robot_1/Container"]
    n5["/robot_1/gossip_node"]
    n74["/robot_1/random_walk_node"]
    n75["/robot_1/robot_state_publisher"]
    n78["/robot_1/topic_keepalive"]
    n81["/robot_1/vdb_mapping"]
    n82["/robot_1/world_to_map_broadcaster"]
  end
  subgraph g7["root"]
    n0["/action_relay_client"]
  end
  subgraph g8["sensors"]
    n76["/robot_1/sensors/lidar_point_cloud_filter"]
  end
  subgraph g9["takeoff_landing_planner"]
    n77["/robot_1/takeoff_landing_planner/takeoff_landing_task"]
  end
  subgraph g10["trajectory_controller"]
    n79["/robot_1/trajectory_controller/fixed_trajectory_task"]
    n80["/robot_1/trajectory_controller/trajectory_control_node"]
  end
  d0(["/clock (no publishers)"]) -->|"/clock<br/>Clock"| n1
  d0 -->|"/clock<br/>Clock"| n2
  d0 -->|"/clock<br/>Clock"| n3
  d0 -->|"/clock<br/>Clock"| n4
  d0 -->|"/clock<br/>Clock"| n5
  d0 -->|"/clock<br/>Clock"| n6
  d0 -->|"/clock<br/>Clock"| n7
  d0 -->|"/clock<br/>Clock"| n8
  d0 -->|"/clock<br/>Clock"| n9
  d0 -->|"/clock<br/>Clock"| n10
  d0 -->|"/clock<br/>Clock"| n11
  d0 -->|"/clock<br/>Clock"| n12
  d0 -->|"/clock<br/>Clock"| n13
  d0 -->|"/clock<br/>Clock"| n14
  d0 -->|"/clock<br/>Clock"| n15
  d0 -->|"/clock<br/>Clock"| n16
  d0 -->|"/clock<br/>Clock"| n17
  d0 -->|"/clock<br/>Clock"| n18
  d0 -->|"/clock<br/>Clock"| n19
  d0 -->|"/clock<br/>Clock"| n20
  d0 -->|"/clock<br/>Clock"| n21
  d0 -->|"/clock<br/>Clock"| n22
  d0 -->|"/clock<br/>Clock"| n23
  d0 -->|"/clock<br/>Clock"| n24
  d0 -->|"/clock<br/>Clock"| n25
  d0 -->|"/clock<br/>Clock"| n26
  d0 -->|"/clock<br/>Clock"| n27
  d0 -->|"/clock<br/>Clock"| n28
  d0 -->|"/clock<br/>Clock"| n29
  d0 -->|"/clock<br/>Clock"| n30
  d0 -->|"/clock<br/>Clock"| n31
  d0 -->|"/clock<br/>Clock"| n32
  d0 -->|"/clock<br/>Clock"| n33
  d0 -->|"/clock<br/>Clock"| n34
  d0 -->|"/clock<br/>Clock"| n35
  d0 -->|"/clock<br/>Clock"| n36
  d0 -->|"/clock<br/>Clock"| n37
  d0 -->|"/clock<br/>Clock"| n38
  d0 -->|"/clock<br/>Clock"| n39
  d0 -->|"/clock<br/>Clock"| n40
  d0 -->|"/clock<br/>Clock"| n41
  d0 -->|"/clock<br/>Clock"| n42
  d0 -->|"/clock<br/>Clock"| n43
  d0 -->|"/clock<br/>Clock"| n44
  d0 -->|"/clock<br/>Clock"| n45
  d0 -->|"/clock<br/>Clock"| n46
  d0 -->|"/clock<br/>Clock"| n47
  d0 -->|"/clock<br/>Clock"| n48
  d0 -->|"/clock<br/>Clock"| n49
  d0 -->|"/clock<br/>Clock"| n50
  d0 -->|"/clock<br/>Clock"| n51
  d0 -->|"/clock<br/>Clock"| n52
  d0 -->|"/clock<br/>Clock"| n53
  d0 -->|"/clock<br/>Clock"| n54
  d0 -->|"/clock<br/>Clock"| n55
  d0 -->|"/clock<br/>Clock"| n56
  d0 -->|"/clock<br/>Clock"| n57
  d0 -->|"/clock<br/>Clock"| n58
  d0 -->|"/clock<br/>Clock"| n59
  d0 -->|"/clock<br/>Clock"| n60
  d0 -->|"/clock<br/>Clock"| n61
  d0 -->|"/clock<br/>Clock"| n62
  d0 -->|"/clock<br/>Clock"| n63
  d0 -->|"/clock<br/>Clock"| n64
  d0 -->|"/clock<br/>Clock"| n65
  d0 -->|"/clock<br/>Clock"| n66
  d0 -->|"/clock<br/>Clock"| n67
  d0 -->|"/clock<br/>Clock"| n68
  d0 -->|"/clock<br/>Clock"| n69
  d0 -->|"/clock<br/>Clock"| n70
  d0 -->|"/clock<br/>Clock"| n71
  d0 -->|"/clock<br/>Clock"| n72
  d0 -->|"/clock<br/>Clock"| n73
  d0 -->|"/clock<br/>Clock"| n74
  d0 -->|"/clock<br/>Clock"| n75
  d0 -->|"/clock<br/>Clock"| n76
  d0 -->|"/clock<br/>Clock"| n77
  d0 -->|"/clock<br/>Clock"| n78
  d0 -->|"/clock<br/>Clock"| n79
  d0 -->|"/clock<br/>Clock"| n80
  d0 -->|"/clock<br/>Clock"| n81
  d0 -->|"/clock<br/>Clock"| n82
  n34 -->|"/diagnostics<br/>DiagnosticArray"| d1(["/diagnostics (no subscribers)"])
  n36 -->|"/diagnostics<br/>DiagnosticArray"| d1
  n5 -->|"/gossip/peers<br/>PeerProfile"| n5
  n25 -->|"/move_base_simple/goal<br/>PoseStamped"| d2(["/move_base_simple/goal (no subscribers)"])
  d3(["/robot_1/behavior/drone_safety_monitor/command (no publishers)"]) -->|"/robot_1/behavior/drone_safety_monitor/command<br/>String"| n2
  n2 -->|"/robot_1/behavior/drone_safety_monitor/state_estimate_timed_out<br/>Bool"| n77
  n70 -->|"/robot_1/control/reset_integrators<br/>Empty"| n3
  n3 -->|"/robot_1/control/vx_pid_info<br/>PIDInfo"| d4(["/robot_1/control/vx_pid_info (no subscribers)"])
  n3 -->|"/robot_1/control/vy_pid_info<br/>PIDInfo"| d5(["/robot_1/control/vy_pid_info (no subscribers)"])
  n3 -->|"/robot_1/control/vz_pid_info<br/>PIDInfo"| d6(["/robot_1/control/vz_pid_info (no subscribers)"])
  n3 -->|"/robot_1/control/x_pid_info<br/>PIDInfo"| d7(["/robot_1/control/x_pid_info (no subscribers)"])
  n3 -->|"/robot_1/control/y_pid_info<br/>PIDInfo"| d8(["/robot_1/control/y_pid_info (no subscribers)"])
  n3 -->|"/robot_1/control/z_pid_info<br/>PIDInfo"| d9(["/robot_1/control/z_pid_info (no subscribers)"])
  n5 -->|"/robot_1/coordination/peer_registry<br/>PeerProfile"| d10(["/robot_1/coordination/peer_registry (no subscribers)"])
  n69 -->|"/robot_1/cross_track_error<br/>PoseStamped"| d11(["/robot_1/cross_track_error (no subscribers)"])
  n4 -->|"/robot_1/droan/background_expanded<br/>Image"| d12(["/robot_1/droan/background_expanded (no subscribers)"])
  d13(["/robot_1/droan/clear_map (no publishers)"]) -->|"/robot_1/droan/clear_map<br/>Empty"| n4
  d14(["/robot_1/droan/disparity_graph (no publishers)"]) -->|"/robot_1/droan/disparity_graph<br/>MarkerArray"| n78
  d15(["/robot_1/droan/disparity_map_debug (no publishers)"]) -->|"/robot_1/droan/disparity_map_debug<br/>MarkerArray"| n78
  d16(["/robot_1/droan/expansion_cloud (no publishers)"]) -->|"/robot_1/droan/expansion_cloud<br/>PointCloud2"| n78
  d17(["/robot_1/droan/expansion_poly (no publishers)"]) -->|"/robot_1/droan/expansion_poly<br/>MarkerArray"| n78
  n4 -->|"/robot_1/droan/fg_bg_cloud<br/>PointCloud2"| n78
  n4 -->|"/robot_1/droan/foreground_expanded<br/>Image"| d18(["/robot_1/droan/foreground_expanded (no subscribers)"])
  d19(["/robot_1/droan/frustum (no publishers)"]) -->|"/robot_1/droan/frustum<br/>Marker"| n78
  n4 -->|"/robot_1/droan/graph_vis<br/>MarkerArray"| n78
  n4 -->|"/robot_1/droan/local_planner_global_plan_vis<br/>MarkerArray"| n78
  d20(["/robot_1/droan/reset_stuck (no publishers)"]) -->|"/robot_1/droan/reset_stuck<br/>Empty"| n4
  n4 -->|"/robot_1/droan/rewind_info<br/>MarkerArray"| n78
  n4 -->|"/robot_1/droan/stuck<br/>Bool"| d21(["/robot_1/droan/stuck (no subscribers)"])
  n4 -->|"/robot_1/droan/traj_debug<br/>MarkerArray"| n78
  d22(["/robot_1/droan/trajectory_library_vis (no publishers)"]) -->|"/robot_1/droan/trajectory_library_vis<br/>MarkerArray"| n78
  d23(["/robot_1/droan/virtual_obstacles (no publishers)"]) -->|"/robot_1/droan/virtual_obstacles<br/>MarkerArray"| n78
  n69 -->|"/robot_1/global_plan<br/>Path"| n4
  n69 -->|"/robot_1/global_plan<br/>Path"| n5
  n69 -->|"/robot_1/global_plan<br/>Path"| n78
  n74 -->|"/robot_1/global_plan<br/>Path"| n4
  n74 -->|"/robot_1/global_plan<br/>Path"| n5
  n74 -->|"/robot_1/global_plan<br/>Path"| n78
  d24(["/robot_1/interface/attitude_thrust_command (no publishers)"]) -->|"/robot_1/interface/attitude_thrust_command<br/>AttitudeThrust"| n70
  d25(["/robot_1/interface/cmd_attitude_thrust (no publishers)"]) -->|"/robot_1/interface/cmd_attitude_thrust<br/>AttitudeThrust"| n70
  n69 -->|"/robot_1/interface/cmd_pose<br/>PoseStamped"| n70
  d26(["/robot_1/interface/cmd_rate_thrust (no publishers)"]) -->|"/robot_1/interface/cmd_rate_thrust<br/>RateThrust"| n70
  n3 -->|"/robot_1/interface/cmd_roll_pitch_yawrate_thrust<br/>RollPitchYawrateThrust"| n70
  d27(["/robot_1/interface/cmd_torque_thrust (no publishers)"]) -->|"/robot_1/interface/cmd_torque_thrust<br/>TorqueThrust"| n70
  n69 -->|"/robot_1/interface/cmd_velocity<br/>TwistStamped"| n70
  n70 -->|"/robot_1/interface/has_control<br/>Bool"| n77
  n70 -->|"/robot_1/interface/is_armed<br/>Bool"| n77
  d28(["/robot_1/interface/mavros/actuator_control (no publishers)"]) -->|"/robot_1/interface/mavros/actuator_control<br/>ActuatorControl"| n6
  d29(["/robot_1/interface/mavros/adsb/send (no publishers)"]) -->|"/robot_1/interface/mavros/adsb/send<br/>ADSBVehicle"| n7
  n7 -->|"/robot_1/interface/mavros/adsb/vehicle<br/>ADSBVehicle"| d30(["/robot_1/interface/mavros/adsb/vehicle (no subscribers)"])
  n8 -->|"/robot_1/interface/mavros/altitude<br/>Altitude"| d31(["/robot_1/interface/mavros/altitude (no subscribers)"])
  n59 -->|"/robot_1/interface/mavros/battery<br/>BatteryState"| d32(["/robot_1/interface/mavros/battery (no subscribers)"])
  n9 -->|"/robot_1/interface/mavros/cam_imu_sync/cam_imu_stamp<br/>CamIMUStamp"| d33(["/robot_1/interface/mavros/cam_imu_sync/cam_imu_stamp (no subscribers)"])
  n10 -->|"/robot_1/interface/mavros/camera/image_captured<br/>CameraImageCaptured"| d34(["/robot_1/interface/mavros/camera/image_captured (no subscribers)"])
  d35(["/robot_1/interface/mavros/cellular_status/status (no publishers)"]) -->|"/robot_1/interface/mavros/cellular_status/status<br/>CellularStatus"| n11
  d36(["/robot_1/interface/mavros/companion_process/status (no publishers)"]) -->|"/robot_1/interface/mavros/companion_process/status<br/>CompanionProcessStatus"| n13
  n14 -->|"/robot_1/interface/mavros/debug_value/debug<br/>DebugValue"| d37(["/robot_1/interface/mavros/debug_value/debug (no subscribers)"])
  n14 -->|"/robot_1/interface/mavros/debug_value/debug_float_array<br/>DebugValue"| d38(["/robot_1/interface/mavros/debug_value/debug_float_array (no subscribers)"])
  n14 -->|"/robot_1/interface/mavros/debug_value/debug_vector<br/>DebugValue"| d39(["/robot_1/interface/mavros/debug_value/debug_vector (no subscribers)"])
  n14 -->|"/robot_1/interface/mavros/debug_value/named_value_float<br/>DebugValue"| d40(["/robot_1/interface/mavros/debug_value/named_value_float (no subscribers)"])
  n14 -->|"/robot_1/interface/mavros/debug_value/named_value_int<br/>DebugValue"| d41(["/robot_1/interface/mavros/debug_value/named_value_int (no subscribers)"])
  d42(["/robot_1/interface/mavros/debug_value/send (no publishers)"]) -->|"/robot_1/interface/mavros/debug_value/send<br/>DebugValue"| n14
  n15 -->|"/robot_1/interface/mavros/esc_status/info<br/>ESCInfo"| d43(["/robot_1/interface/mavros/esc_status/info (no subscribers)"])
  n15 -->|"/robot_1/interface/mavros/esc_status/status<br/>ESCStatus"| d44(["/robot_1/interface/mavros/esc_status/status (no subscribers)"])
  n16 -->|"/robot_1/interface/mavros/esc_telemetry/telemetry<br/>ESCTelemetry"| d45(["/robot_1/interface/mavros/esc_telemetry/telemetry (no subscribers)"])
  n59 -->|"/robot_1/interface/mavros/estimator_status<br/>EstimatorStatus"| d46(["/robot_1/interface/mavros/estimator_status (no subscribers)"])
  n59 -->|"/robot_1/interface/mavros/extended_state<br/>ExtendedState"| n77
  d47(["/robot_1/interface/mavros/fake_gps/mocap/tf (no publishers)"]) -->|"/robot_1/interface/mavros/fake_gps/mocap/tf<br/>TransformStamped"| n17
  n19 -->|"/robot_1/interface/mavros/geofence/fences<br/>WaypointList"| d48(["/robot_1/interface/mavros/geofence/fences (no subscribers)"])
  n20 -->|"/robot_1/interface/mavros/gimbal_control/device/attitude_status<br/>GimbalDeviceAttitudeStatus"| d49(["/robot_1/interface/mavros/gimbal_control/device/attitude_status (no subscribers)"])
  n20 -->|"/robot_1/interface/mavros/gimbal_control/device/info<br/>GimbalDeviceInformation"| d50(["/robot_1/interface/mavros/gimbal_control/device/info (no subscribers)"])
  d51(["/robot_1/interface/mavros/gimbal_control/device/set_attitude (no publishers)"]) -->|"/robot_1/interface/mavros/gimbal_control/device/set_attitude<br/>GimbalDeviceSetAttitude"| n20
  n20 -->|"/robot_1/interface/mavros/gimbal_control/manager/info<br/>GimbalManagerInformation"| d52(["/robot_1/interface/mavros/gimbal_control/manager/info (no subscribers)"])
  d53(["/robot_1/interface/mavros/gimbal_control/manager/set_attitude (no publishers)"]) -->|"/robot_1/interface/mavros/gimbal_control/manager/set_attitude<br/>GimbalManagerSetAttitude"| n20
  d54(["/robot_1/interface/mavros/gimbal_control/manager/set_manual_control (no publishers)"]) -->|"/robot_1/interface/mavros/gimbal_control/manager/set_manual_control<br/>GimbalManagerSetPitchyaw"| n20
  d55(["/robot_1/interface/mavros/gimbal_control/manager/set_pitchyaw (no publishers)"]) -->|"/robot_1/interface/mavros/gimbal_control/manager/set_pitchyaw<br/>GimbalManagerSetPitchyaw"| n20
  n20 -->|"/robot_1/interface/mavros/gimbal_control/manager/status<br/>GimbalManagerStatus"| d56(["/robot_1/interface/mavros/gimbal_control/manager/status (no subscribers)"])
  n21 -->|"/robot_1/interface/mavros/global_position/compass_hdg<br/>Float64"| n5
  n21 -->|"/robot_1/interface/mavros/global_position/global<br/>NavSatFix"| n54
  n21 -->|"/robot_1/interface/mavros/global_position/global<br/>NavSatFix"| n70
  n21 -->|"/robot_1/interface/mavros/global_position/gp_lp_offset<br/>PoseStamped"| d57(["/robot_1/interface/mavros/global_position/gp_lp_offset (no subscribers)"])
  n21 -->|"/robot_1/interface/mavros/global_position/gp_origin<br/>GeoPointStamped"| n25
  n21 -->|"/robot_1/interface/mavros/global_position/local<br/>Odometry"| n70
  n21 -->|"/robot_1/interface/mavros/global_position/raw/fix<br/>NavSatFix"| n5
  n21 -->|"/robot_1/interface/mavros/global_position/raw/gps_vel<br/>TwistStamped"| d58(["/robot_1/interface/mavros/global_position/raw/gps_vel (no subscribers)"])
  n21 -->|"/robot_1/interface/mavros/global_position/raw/satellites<br/>UInt32"| d59(["/robot_1/interface/mavros/global_position/raw/satellites (no subscribers)"])
  n21 -->|"/robot_1/interface/mavros/global_position/rel_alt<br/>Float64"| d60(["/robot_1/interface/mavros/global_position/rel_alt (no subscribers)"])
  d61(["/robot_1/interface/mavros/global_position/set_gp_origin (no publishers)"]) -->|"/robot_1/interface/mavros/global_position/set_gp_origin<br/>GeoPointStamped"| n21
  d62(["/robot_1/interface/mavros/gps_input/gps_input (no publishers)"]) -->|"/robot_1/interface/mavros/gps_input/gps_input<br/>GPSINPUT"| n22
  n23 -->|"/robot_1/interface/mavros/gps_rtk/rtk_baseline<br/>RTKBaseline"| d63(["/robot_1/interface/mavros/gps_rtk/rtk_baseline (no subscribers)"])
  d64(["/robot_1/interface/mavros/gps_rtk/send_rtcm (no publishers)"]) -->|"/robot_1/interface/mavros/gps_rtk/send_rtcm<br/>RTCM"| n23
  n24 -->|"/robot_1/interface/mavros/gpsstatus/gps1/raw<br/>GPSRAW"| d65(["/robot_1/interface/mavros/gpsstatus/gps1/raw (no subscribers)"])
  n24 -->|"/robot_1/interface/mavros/gpsstatus/gps1/rtk<br/>GPSRTK"| d66(["/robot_1/interface/mavros/gpsstatus/gps1/rtk (no subscribers)"])
  n24 -->|"/robot_1/interface/mavros/gpsstatus/gps2/raw<br/>GPSRAW"| d67(["/robot_1/interface/mavros/gpsstatus/gps2/raw (no subscribers)"])
  n24 -->|"/robot_1/interface/mavros/gpsstatus/gps2/rtk<br/>GPSRTK"| d68(["/robot_1/interface/mavros/gpsstatus/gps2/rtk (no subscribers)"])
  n26 -->|"/robot_1/interface/mavros/hil/actuator_controls<br/>HilActuatorControls"| d69(["/robot_1/interface/mavros/hil/actuator_controls (no subscribers)"])
  n26 -->|"/robot_1/interface/mavros/hil/controls<br/>HilControls"| d70(["/robot_1/interface/mavros/hil/controls (no subscribers)"])
  d71(["/robot_1/interface/mavros/hil/gps (no publishers)"]) -->|"/robot_1/interface/mavros/hil/gps<br/>HilGPS"| n26
  d72(["/robot_1/interface/mavros/hil/imu_ned (no publishers)"]) -->|"/robot_1/interface/mavros/hil/imu_ned<br/>HilSensor"| n26
  d73(["/robot_1/interface/mavros/hil/optical_flow (no publishers)"]) -->|"/robot_1/interface/mavros/hil/optical_flow<br/>OpticalFlowRad"| n26
  d74(["/robot_1/interface/mavros/hil/rc_inputs (no publishers)"]) -->|"/robot_1/interface/mavros/hil/rc_inputs<br/>RCIn"| n26
  d75(["/robot_1/interface/mavros/hil/state (no publishers)"]) -->|"/robot_1/interface/mavros/hil/state<br/>HilStateQuaternion"| n26
  n27 -->|"/robot_1/interface/mavros/home_position/home<br/>HomePosition"| n21
  n27 -->|"/robot_1/interface/mavros/home_position/home<br/>HomePosition"| n70
  n70 -->|"/robot_1/interface/mavros/home_position/set<br/>HomePosition"| n27
  n28 -->|"/robot_1/interface/mavros/imu/data<br/>Imu"| d76(["/robot_1/interface/mavros/imu/data (no subscribers)"])
  n28 -->|"/robot_1/interface/mavros/imu/data_raw<br/>Imu"| d77(["/robot_1/interface/mavros/imu/data_raw (no subscribers)"])
  n28 -->|"/robot_1/interface/mavros/imu/diff_pressure<br/>FluidPressure"| d78(["/robot_1/interface/mavros/imu/diff_pressure (no subscribers)"])
  n28 -->|"/robot_1/interface/mavros/imu/mag<br/>MagneticField"| d79(["/robot_1/interface/mavros/imu/mag (no subscribers)"])
  n28 -->|"/robot_1/interface/mavros/imu/static_pressure<br/>FluidPressure"| d80(["/robot_1/interface/mavros/imu/static_pressure (no subscribers)"])
  n28 -->|"/robot_1/interface/mavros/imu/temperature_baro<br/>Temperature"| d81(["/robot_1/interface/mavros/imu/temperature_baro (no subscribers)"])
  n28 -->|"/robot_1/interface/mavros/imu/temperature_imu<br/>Temperature"| d82(["/robot_1/interface/mavros/imu/temperature_imu (no subscribers)"])
  n29 -->|"/robot_1/interface/mavros/landing_target/lt_marker<br/>Vector3Stamped"| d83(["/robot_1/interface/mavros/landing_target/lt_marker (no subscribers)"])
  d84(["/robot_1/interface/mavros/landing_target/pose (no publishers)"]) -->|"/robot_1/interface/mavros/landing_target/pose<br/>PoseStamped"| n29
  n29 -->|"/robot_1/interface/mavros/landing_target/pose_in<br/>PoseStamped"| d85(["/robot_1/interface/mavros/landing_target/pose_in (no subscribers)"])
  n30 -->|"/robot_1/interface/mavros/local_position/accel<br/>AccelWithCovarianceStamped"| d86(["/robot_1/interface/mavros/local_position/accel (no subscribers)"])
  n30 -->|"/robot_1/interface/mavros/local_position/odom<br/>Odometry"| n71
  n30 -->|"/robot_1/interface/mavros/local_position/pose<br/>PoseStamped"| n54
  n30 -->|"/robot_1/interface/mavros/local_position/pose_cov<br/>PoseWithCovarianceStamped"| d87(["/robot_1/interface/mavros/local_position/pose_cov (no subscribers)"])
  n30 -->|"/robot_1/interface/mavros/local_position/velocity_body<br/>TwistStamped"| d88(["/robot_1/interface/mavros/local_position/velocity_body (no subscribers)"])
  n30 -->|"/robot_1/interface/mavros/local_position/velocity_body_cov<br/>TwistWithCovarianceStamped"| d89(["/robot_1/interface/mavros/local_position/velocity_body_cov (no subscribers)"])
  n30 -->|"/robot_1/interface/mavros/local_position/velocity_local<br/>TwistStamped"| d90(["/robot_1/interface/mavros/local_position/velocity_local (no subscribers)"])
  n31 -->|"/robot_1/interface/mavros/log_transfer/raw/log_data<br/>LogData"| d91(["/robot_1/interface/mavros/log_transfer/raw/log_data (no subscribers)"])
  n31 -->|"/robot_1/interface/mavros/log_transfer/raw/log_entry<br/>LogEntry"| d92(["/robot_1/interface/mavros/log_transfer/raw/log_entry (no subscribers)"])
  n32 -->|"/robot_1/interface/mavros/mag_calibration/report<br/>MagnetometerReporter"| d93(["/robot_1/interface/mavros/mag_calibration/report (no subscribers)"])
  n32 -->|"/robot_1/interface/mavros/mag_calibration/status<br/>UInt8"| d94(["/robot_1/interface/mavros/mag_calibration/status (no subscribers)"])
  n33 -->|"/robot_1/interface/mavros/manual_control/control<br/>ManualControl"| d95(["/robot_1/interface/mavros/manual_control/control (no subscribers)"])
  d96(["/robot_1/interface/mavros/manual_control/send (no publishers)"]) -->|"/robot_1/interface/mavros/manual_control/send<br/>ManualControl"| n33
  n37 -->|"/robot_1/interface/mavros/mission/reached<br/>WaypointReached"| d97(["/robot_1/interface/mavros/mission/reached (no subscribers)"])
  n37 -->|"/robot_1/interface/mavros/mission/waypoints<br/>WaypointList"| d98(["/robot_1/interface/mavros/mission/waypoints (no subscribers)"])
  d99(["/robot_1/interface/mavros/mocap/pose (no publishers)"]) -->|"/robot_1/interface/mavros/mocap/pose<br/>PoseStamped"| n38
  d100(["/robot_1/interface/mavros/mocap/tf (no publishers)"]) -->|"/robot_1/interface/mavros/mocap/tf<br/>TransformStamped"| n38
  d101(["/robot_1/interface/mavros/mount_control/command (no publishers)"]) -->|"/robot_1/interface/mavros/mount_control/command<br/>MountControl"| n39
  n39 -->|"/robot_1/interface/mavros/mount_control/orientation<br/>Quaternion"| d102(["/robot_1/interface/mavros/mount_control/orientation (no subscribers)"])
  n39 -->|"/robot_1/interface/mavros/mount_control/status<br/>Vector3Stamped"| d103(["/robot_1/interface/mavros/mount_control/status (no subscribers)"])
  n40 -->|"/robot_1/interface/mavros/nav_controller_output/output<br/>NavControllerOutput"| d104(["/robot_1/interface/mavros/nav_controller_output/output (no subscribers)"])
  d105(["/robot_1/interface/mavros/obstacle/send (no publishers)"]) -->|"/robot_1/interface/mavros/obstacle/send<br/>LaserScan"| n41
  d106(["/robot_1/interface/mavros/obstacle_distance_3d/send (no publishers)"]) -->|"/robot_1/interface/mavros/obstacle_distance_3d/send<br/>ObstacleDistance3D"| n42
  n43 -->|"/robot_1/interface/mavros/odometry/in<br/>Odometry"| d107(["/robot_1/interface/mavros/odometry/in (no subscribers)"])
  d108(["/robot_1/interface/mavros/odometry/out (no publishers)"]) -->|"/robot_1/interface/mavros/odometry/out<br/>Odometry"| n43
  d109(["/robot_1/interface/mavros/onboard_computer/status (no publishers)"]) -->|"/robot_1/interface/mavros/onboard_computer/status<br/>OnboardComputerStatus"| n44
  d110(["/robot_1/interface/mavros/open_drone_id/basic_id (no publishers)"]) -->|"/robot_1/interface/mavros/open_drone_id/basic_id<br/>OpenDroneIDBasicID"| n45
  d111(["/robot_1/interface/mavros/open_drone_id/operator_id (no publishers)"]) -->|"/robot_1/interface/mavros/open_drone_id/operator_id<br/>OpenDroneIDOperatorID"| n45
  d112(["/robot_1/interface/mavros/open_drone_id/self_id (no publishers)"]) -->|"/robot_1/interface/mavros/open_drone_id/self_id<br/>OpenDroneIDSelfID"| n45
  d113(["/robot_1/interface/mavros/open_drone_id/system (no publishers)"]) -->|"/robot_1/interface/mavros/open_drone_id/system<br/>OpenDroneIDSystem"| n45
  d114(["/robot_1/interface/mavros/open_drone_id/system_update (no publishers)"]) -->|"/robot_1/interface/mavros/open_drone_id/system_update<br/>OpenDroneIDSystemUpdate"| n45
  n46 -->|"/robot_1/interface/mavros/optical_flow/ground_distance<br/>Range"| d115(["/robot_1/interface/mavros/optical_flow/ground_distance (no subscribers)"])
  n46 -->|"/robot_1/interface/mavros/optical_flow/raw/optical_flow<br/>OpticalFlow"| d116(["/robot_1/interface/mavros/optical_flow/raw/optical_flow (no subscribers)"])
  d117(["/robot_1/interface/mavros/optical_flow/raw/send (no publishers)"]) -->|"/robot_1/interface/mavros/optical_flow/raw/send<br/>OpticalFlow"| n46
  n47 -->|"/robot_1/interface/mavros/param/event<br/>ParamEvent"| d118(["/robot_1/interface/mavros/param/event (no subscribers)"])
  d119(["/robot_1/interface/mavros/play_tune (no publishers)"]) -->|"/robot_1/interface/mavros/play_tune<br/>PlayTuneV2"| n48
  n49 -->|"/robot_1/interface/mavros/px4flow/ground_distance<br/>Range"| d120(["/robot_1/interface/mavros/px4flow/ground_distance (no subscribers)"])
  n49 -->|"/robot_1/interface/mavros/px4flow/raw/optical_flow_rad<br/>OpticalFlowRad"| d121(["/robot_1/interface/mavros/px4flow/raw/optical_flow_rad (no subscribers)"])
  d122(["/robot_1/interface/mavros/px4flow/raw/send (no publishers)"]) -->|"/robot_1/interface/mavros/px4flow/raw/send<br/>OpticalFlowRad"| n49
  n49 -->|"/robot_1/interface/mavros/px4flow/temperature<br/>Temperature"| d123(["/robot_1/interface/mavros/px4flow/temperature (no subscribers)"])
  n60 -->|"/robot_1/interface/mavros/radio_status<br/>RadioStatus"| d124(["/robot_1/interface/mavros/radio_status (no subscribers)"])
  n50 -->|"/robot_1/interface/mavros/rallypoint/rallypoints<br/>WaypointList"| d125(["/robot_1/interface/mavros/rallypoint/rallypoints (no subscribers)"])
  n51 -->|"/robot_1/interface/mavros/rc/in<br/>RCIn"| d126(["/robot_1/interface/mavros/rc/in (no subscribers)"])
  n51 -->|"/robot_1/interface/mavros/rc/out<br/>RCOut"| d127(["/robot_1/interface/mavros/rc/out (no subscribers)"])
  d128(["/robot_1/interface/mavros/rc/override (no publishers)"]) -->|"/robot_1/interface/mavros/rc/override<br/>OverrideRCIn"| n51
  d129(["/robot_1/interface/mavros/setpoint_accel/accel (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_accel/accel<br/>Vector3Stamped"| n52
  d130(["/robot_1/interface/mavros/setpoint_attitude/cmd_vel (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_attitude/cmd_vel<br/>TwistStamped"| n53
  d131(["/robot_1/interface/mavros/setpoint_attitude/thrust (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_attitude/thrust<br/>Thrust"| n53
  n70 -->|"/robot_1/interface/mavros/setpoint_position/global<br/>GeoPoseStamped"| n54
  d132(["/robot_1/interface/mavros/setpoint_position/global_to_local (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_position/global_to_local<br/>GeoPoseStamped"| n54
  n70 -->|"/robot_1/interface/mavros/setpoint_position/local<br/>PoseStamped"| n54
  n70 -->|"/robot_1/interface/mavros/setpoint_raw/attitude<br/>AttitudeTarget"| n55
  d133(["/robot_1/interface/mavros/setpoint_raw/global (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_raw/global<br/>GlobalPositionTarget"| n55
  n70 -->|"/robot_1/interface/mavros/setpoint_raw/local<br/>PositionTarget"| n55
  n55 -->|"/robot_1/interface/mavros/setpoint_raw/target_attitude<br/>AttitudeTarget"| d134(["/robot_1/interface/mavros/setpoint_raw/target_attitude (no subscribers)"])
  n55 -->|"/robot_1/interface/mavros/setpoint_raw/target_global<br/>GlobalPositionTarget"| d135(["/robot_1/interface/mavros/setpoint_raw/target_global (no subscribers)"])
  n55 -->|"/robot_1/interface/mavros/setpoint_raw/target_local<br/>PositionTarget"| d136(["/robot_1/interface/mavros/setpoint_raw/target_local (no subscribers)"])
  n56 -->|"/robot_1/interface/mavros/setpoint_trajectory/desired<br/>Path"| d137(["/robot_1/interface/mavros/setpoint_trajectory/desired (no subscribers)"])
  d138(["/robot_1/interface/mavros/setpoint_trajectory/local (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_trajectory/local<br/>MultiDOFJointTrajectory"| n56
  d139(["/robot_1/interface/mavros/setpoint_velocity/cmd_vel (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_velocity/cmd_vel<br/>TwistStamped"| n57
  d140(["/robot_1/interface/mavros/setpoint_velocity/cmd_vel_unstamped (no publishers)"]) -->|"/robot_1/interface/mavros/setpoint_velocity/cmd_vel_unstamped<br/>Twist"| n57
  n58 -->|"/robot_1/interface/mavros/sim_state/acceleration<br/>Vector3Stamped"| d141(["/robot_1/interface/mavros/sim_state/acceleration (no subscribers)"])
  n58 -->|"/robot_1/interface/mavros/sim_state/attitude<br/>Imu"| d142(["/robot_1/interface/mavros/sim_state/attitude (no subscribers)"])
  n58 -->|"/robot_1/interface/mavros/sim_state/global_position<br/>NavSatFix"| d143(["/robot_1/interface/mavros/sim_state/global_position (no subscribers)"])
  n58 -->|"/robot_1/interface/mavros/sim_state/velocity_body<br/>TwistStamped"| d144(["/robot_1/interface/mavros/sim_state/velocity_body (no subscribers)"])
  n58 -->|"/robot_1/interface/mavros/sim_state/velocity_local<br/>TwistStamped"| d145(["/robot_1/interface/mavros/sim_state/velocity_local (no subscribers)"])
  n59 -->|"/robot_1/interface/mavros/state<br/>State"| n70
  n59 -->|"/robot_1/interface/mavros/status_event<br/>StatusEvent"| d146(["/robot_1/interface/mavros/status_event (no subscribers)"])
  n59 -->|"/robot_1/interface/mavros/statustext/recv<br/>StatusText"| d147(["/robot_1/interface/mavros/statustext/recv (no subscribers)"])
  d148(["/robot_1/interface/mavros/statustext/send (no publishers)"]) -->|"/robot_1/interface/mavros/statustext/send<br/>StatusText"| n59
  n59 -->|"/robot_1/interface/mavros/sys_status<br/>SysStatus"| d149(["/robot_1/interface/mavros/sys_status (no subscribers)"])
  n6 -->|"/robot_1/interface/mavros/target_actuator_control<br/>ActuatorControl"| d150(["/robot_1/interface/mavros/target_actuator_control (no subscribers)"])
  n61 -->|"/robot_1/interface/mavros/terrain/report<br/>TerrainReport"| d151(["/robot_1/interface/mavros/terrain/report (no subscribers)"])
  n62 -->|"/robot_1/interface/mavros/time_reference<br/>TimeReference"| d152(["/robot_1/interface/mavros/time_reference (no subscribers)"])
  n62 -->|"/robot_1/interface/mavros/timesync_status<br/>TimesyncStatus"| d153(["/robot_1/interface/mavros/timesync_status (no subscribers)"])
  n63 -->|"/robot_1/interface/mavros/trajectory/desired<br/>Trajectory"| d154(["/robot_1/interface/mavros/trajectory/desired (no subscribers)"])
  d155(["/robot_1/interface/mavros/trajectory/generated (no publishers)"]) -->|"/robot_1/interface/mavros/trajectory/generated<br/>Trajectory"| n63
  d156(["/robot_1/interface/mavros/trajectory/path (no publishers)"]) -->|"/robot_1/interface/mavros/trajectory/path<br/>Path"| n63
  d157(["/robot_1/interface/mavros/tunnel/in (no publishers)"]) -->|"/robot_1/interface/mavros/tunnel/in<br/>Tunnel"| n64
  n64 -->|"/robot_1/interface/mavros/tunnel/out<br/>Tunnel"| d158(["/robot_1/interface/mavros/tunnel/out (no subscribers)"])
  n65 -->|"/robot_1/interface/mavros/vfr_hud<br/>VfrHud"| d159(["/robot_1/interface/mavros/vfr_hud (no subscribers)"])
  d160(["/robot_1/interface/mavros/vision_pose/pose (no publishers)"]) -->|"/robot_1/interface/mavros/vision_pose/pose<br/>PoseStamped"| n66
  d161(["/robot_1/interface/mavros/vision_pose/pose_cov (no publishers)"]) -->|"/robot_1/interface/mavros/vision_pose/pose_cov<br/>PoseWithCovarianceStamped"| n66
  d162(["/robot_1/interface/mavros/vision_speed/speed_twist (no publishers)"]) -->|"/robot_1/interface/mavros/vision_speed/speed_twist<br/>TwistStamped"| n67
  d163(["/robot_1/interface/mavros/vision_speed/speed_twist_cov (no publishers)"]) -->|"/robot_1/interface/mavros/vision_speed/speed_twist_cov<br/>TwistWithCovarianceStamped"| n67
  d164(["/robot_1/interface/mavros/vision_speed/speed_vector (no publishers)"]) -->|"/robot_1/interface/mavros/vision_speed/speed_vector<br/>Vector3Stamped"| n67
  n68 -->|"/robot_1/interface/mavros/wind_estimation<br/>TwistWithCovarianceStamped"| d165(["/robot_1/interface/mavros/wind_estimation (no subscribers)"])
  d166(["/robot_1/interface/pose_command (no publishers)"]) -->|"/robot_1/interface/pose_command<br/>PoseStamped"| n70
  d167(["/robot_1/interface/rate_thrust_command (no publishers)"]) -->|"/robot_1/interface/rate_thrust_command<br/>RateThrust"| n70
  d168(["/robot_1/interface/roll_pitch_yawrate_thrust_command (no publishers)"]) -->|"/robot_1/interface/roll_pitch_yawrate_thrust_command<br/>RollPitchYawrateThrust"| n70
  d169(["/robot_1/interface/torque_thrust_command (no publishers)"]) -->|"/robot_1/interface/torque_thrust_command<br/>TorqueThrust"| n70
  d170(["/robot_1/interface/velocity_command (no publishers)"]) -->|"/robot_1/interface/velocity_command<br/>TwistStamped"| n70
  d171(["/robot_1/joint_states (no publishers)"]) -->|"/robot_1/joint_states<br/>JointState"| n75
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n2
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n3
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n69
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n74
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n77
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n78
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n79
  n71 -->|"/robot_1/odometry_conversion/odometry<br/>Odometry"| n80
  n72 -->|"/robot_1/perception/stereo_image_proc/disparity<br/>DisparityImage"| n4
  n72 -->|"/robot_1/perception/stereo_image_proc/disparity<br/>DisparityImage"| n73
  n73 -->|"/robot_1/perception/stereo_image_proc/point_cloud<br/>PointCloud2"| n78
  n74 -->|"/robot_1/random_walk_node/goal_point_viz<br/>Marker"| d172(["/robot_1/random_walk_node/goal_point_viz (no subscribers)"])
  n74 -->|"/robot_1/random_walk_node/traj_viz<br/>Marker"| d173(["/robot_1/random_walk_node/traj_viz (no subscribers)"])
  n75 -->|"/robot_1/robot_description<br/>String"| d174(["/robot_1/robot_description (no subscribers)"])
  d175(["/robot_1/sensors/front_stereo/left/camera_info (no publishers)"]) -->|"/robot_1/sensors/front_stereo/left/camera_info<br/>CameraInfo"| n72
  d175 -->|"/robot_1/sensors/front_stereo/left/camera_info<br/>CameraInfo"| n73
  d175 -->|"/robot_1/sensors/front_stereo/left/camera_info<br/>CameraInfo"| n78
  d176(["/robot_1/sensors/front_stereo/left/depth_ground_truth (no publishers)"]) -->|"/robot_1/sensors/front_stereo/left/depth_ground_truth<br/>Image"| n78
  d177(["/robot_1/sensors/front_stereo/left/image_rect (no publishers)"]) -->|"/robot_1/sensors/front_stereo/left/image_rect<br/>Image"| n72
  d177 -->|"/robot_1/sensors/front_stereo/left/image_rect<br/>Image"| n73
  d177 -->|"/robot_1/sensors/front_stereo/left/image_rect<br/>Image"| n78
  d178(["/robot_1/sensors/front_stereo/right/camera_info (no publishers)"]) -->|"/robot_1/sensors/front_stereo/right/camera_info<br/>CameraInfo"| n4
  d178 -->|"/robot_1/sensors/front_stereo/right/camera_info<br/>CameraInfo"| n72
  d178 -->|"/robot_1/sensors/front_stereo/right/camera_info<br/>CameraInfo"| n73
  d178 -->|"/robot_1/sensors/front_stereo/right/camera_info<br/>CameraInfo"| n78
  d179(["/robot_1/sensors/front_stereo/right/depth_ground_truth (no publishers)"]) -->|"/robot_1/sensors/front_stereo/right/depth_ground_truth<br/>Image"| n78
  d180(["/robot_1/sensors/front_stereo/right/image_rect (no publishers)"]) -->|"/robot_1/sensors/front_stereo/right/image_rect<br/>Image"| n72
  d180 -->|"/robot_1/sensors/front_stereo/right/image_rect<br/>Image"| n78
  d181(["/robot_1/sensors/lidar/point_cloud (no publishers)"]) -->|"/robot_1/sensors/lidar/point_cloud<br/>PointCloud2"| n78
  n76 -->|"/robot_1/sensors/ouster/point_cloud<br/>PointCloud2"| n81
  d182(["/robot_1/sensors/ouster/point_cloud_raw (no publishers)"]) -->|"/robot_1/sensors/ouster/point_cloud_raw<br/>PointCloud2"| n76
  n77 -->|"/robot_1/takeoff_landing_planner/is_airborne<br/>Bool"| d183(["/robot_1/takeoff_landing_planner/is_airborne (no subscribers)"])
  d184(["/robot_1/takeoff_landing_planner/trajectory_completion_percentage (no publishers)"]) -->|"/robot_1/takeoff_landing_planner/trajectory_completion_percentage<br/>Float32"| n77
  n80 -->|"/robot_1/trajectory_controller/closest_point<br/>Odometry"| d185(["/robot_1/trajectory_controller/closest_point (no subscribers)"])
  n80 -->|"/robot_1/trajectory_controller/look_ahead<br/>Odometry"| n4
  n80 -->|"/robot_1/trajectory_controller/projected_drone_pose<br/>PoseStamped"| n69
  n80 -->|"/robot_1/trajectory_controller/tracking_error<br/>Float32"| d186(["/robot_1/trajectory_controller/tracking_error (no subscribers)"])
  n80 -->|"/robot_1/trajectory_controller/tracking_point<br/>Odometry"| n3
  n80 -->|"/robot_1/trajectory_controller/tracking_point<br/>Odometry"| n4
  n80 -->|"/robot_1/trajectory_controller/tracking_point<br/>Odometry"| n69
  n80 -->|"/robot_1/trajectory_controller/tracking_point<br/>Odometry"| n77
  n80 -->|"/robot_1/trajectory_controller/tracking_point_velocity_magnitude<br/>Float32"| d187(["/robot_1/trajectory_controller/tracking_point_velocity_magnitude (no subscribers)"])
  n80 -->|"/robot_1/trajectory_controller/traj_drone_point<br/>Odometry"| d188(["/robot_1/trajectory_controller/traj_drone_point (no subscribers)"])
  n80 -->|"/robot_1/trajectory_controller/trajectory_completion_percentage<br/>Float32"| n79
  n80 -->|"/robot_1/trajectory_controller/trajectory_controller_debug_markers<br/>MarkerArray"| n78
  n77 -->|"/robot_1/trajectory_controller/trajectory_override<br/>TrajectoryXYZVYaw"| n80
  n79 -->|"/robot_1/trajectory_controller/trajectory_override<br/>TrajectoryXYZVYaw"| n80
  n4 -->|"/robot_1/trajectory_controller/trajectory_segment_to_add<br/>TrajectoryXYZVYaw"| n80
  n80 -->|"/robot_1/trajectory_controller/trajectory_time<br/>Float32"| d189(["/robot_1/trajectory_controller/trajectory_time (no subscribers)"])
  n80 -->|"/robot_1/trajectory_controller/trajectory_vis<br/>MarkerArray"| n78
  n80 -->|"/robot_1/trajectory_controller/virtual_tracking_point<br/>Odometry"| d190(["/robot_1/trajectory_controller/virtual_tracking_point (no subscribers)"])
  n81 -->|"/robot_1/vdb_mapping/vdb_map_overwrites<br/>UpdateGrid"| d191(["/robot_1/vdb_mapping/vdb_map_overwrites (no subscribers)"])
  n81 -->|"/robot_1/vdb_mapping/vdb_map_pointcloud<br/>PointCloud2"| d192(["/robot_1/vdb_mapping/vdb_map_pointcloud (no subscribers)"])
  n81 -->|"/robot_1/vdb_mapping/vdb_map_sections<br/>UpdateGrid"| d193(["/robot_1/vdb_mapping/vdb_map_sections (no subscribers)"])
  n81 -->|"/robot_1/vdb_mapping/vdb_map_updates<br/>UpdateGrid"| d194(["/robot_1/vdb_mapping/vdb_map_updates (no subscribers)"])
  n81 -->|"/robot_1/vdb_mapping/vdb_map_visualization<br/>Marker"| n74
  n81 -->|"/robot_1/vdb_mapping/vdb_map_visualization<br/>Marker"| n78
  n34 -->|"/tf<br/>TFMessage"| n69
  n34 -->|"/tf<br/>TFMessage"| n78
  n71 -->|"/tf<br/>TFMessage"| n69
  n71 -->|"/tf<br/>TFMessage"| n78
  n75 -->|"/tf<br/>TFMessage"| n69
  n75 -->|"/tf<br/>TFMessage"| n78
  n80 -->|"/tf<br/>TFMessage"| n69
  n80 -->|"/tf<br/>TFMessage"| n78
  n34 -->|"/tf_static<br/>TFMessage"| n69
  n34 -->|"/tf_static<br/>TFMessage"| n78
  n75 -->|"/tf_static<br/>TFMessage"| n69
  n75 -->|"/tf_static<br/>TFMessage"| n78
  n82 -->|"/tf_static<br/>TFMessage"| n69
  n82 -->|"/tf_static<br/>TFMessage"| n78
  n34 -->|"/uas2/mavlink_sink<br/>Mavlink"| n36
  n36 -->|"/uas2/mavlink_source<br/>Mavlink"| n34
```

<!-- wiring-graph-v1
{
"edges": [
{
"dir": "sub",
"node": "/robot_1/Container",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/behavior/drone_safety_monitor/drone_safety_monitor",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/gossip_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/actuator_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/adsb",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/altitude",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/cam_imu_sync",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/camera",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/cellular_status",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/cmd",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/companion_process",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/debug_value",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/esc_status",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/esc_telemetry",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/fake_gps",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/ftp",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/geofence",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gps_input",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gps_rtk",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gpsstatus",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/guided_target",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/home_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/landing_target",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/log_transfer",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mag_calibration",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/manual_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mavros",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mavros_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mavros_router",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mission",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mocap",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mount_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/nav_controller_output",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/obstacle",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/obstacle_distance_3d",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/odometry",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/onboard_computer",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/open_drone_id",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/optical_flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/param",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/play_tune",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/px4flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/rallypoint",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/rc",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_accel",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_attitude",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_raw",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_trajectory",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_velocity",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/sim_state",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/tdr_radio",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/terrain",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/time",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/trajectory",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/tunnel",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vfr_hud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vision_pose",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vision_speed",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/wind",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/odometry_conversion/odometry_conversion",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_image_proc/disparity_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_pointcloud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/random_walk_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/robot_state_publisher",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/sensors/lidar_point_cloud_filter",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/trajectory_controller/fixed_trajectory_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/vdb_mapping",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "sub",
"node": "/robot_1/world_to_map_broadcaster",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/clock",
"type": "rosgraph_msgs/msg/Clock"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mavros",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/diagnostics",
"type": "diagnostic_msgs/msg/DiagnosticArray"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mavros_router",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/diagnostics",
"type": "diagnostic_msgs/msg/DiagnosticArray"
},
{
"dir": "pub",
"node": "/robot_1/gossip_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/gossip/peers",
"type": "coordination_msgs/msg/PeerProfile"
},
{
"dir": "sub",
"node": "/robot_1/gossip_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/gossip/peers",
"type": "coordination_msgs/msg/PeerProfile"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/guided_target",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/move_base_simple/goal",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/behavior/drone_safety_monitor/drone_safety_monitor",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/behavior/drone_safety_monitor/command",
"type": "std_msgs/msg/String"
},
{
"dir": "pub",
"node": "/robot_1/behavior/drone_safety_monitor/drone_safety_monitor",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/behavior/drone_safety_monitor/state_estimate_timed_out",
"type": "std_msgs/msg/Bool"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/behavior/drone_safety_monitor/state_estimate_timed_out",
"type": "std_msgs/msg/Bool"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/reset_integrators",
"type": "std_msgs/msg/Empty"
},
{
"dir": "sub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/reset_integrators",
"type": "std_msgs/msg/Empty"
},
{
"dir": "pub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/vx_pid_info",
"type": "pid_controller_msgs/msg/PIDInfo"
},
{
"dir": "pub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/vy_pid_info",
"type": "pid_controller_msgs/msg/PIDInfo"
},
{
"dir": "pub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/vz_pid_info",
"type": "pid_controller_msgs/msg/PIDInfo"
},
{
"dir": "pub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/x_pid_info",
"type": "pid_controller_msgs/msg/PIDInfo"
},
{
"dir": "pub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/y_pid_info",
"type": "pid_controller_msgs/msg/PIDInfo"
},
{
"dir": "pub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/control/z_pid_info",
"type": "pid_controller_msgs/msg/PIDInfo"
},
{
"dir": "pub",
"node": "/robot_1/gossip_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/coordination/peer_registry",
"type": "coordination_msgs/msg/PeerProfile"
},
{
"dir": "pub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/cross_track_error",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/background_expanded",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/clear_map",
"type": "std_msgs/msg/Empty"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/disparity_graph",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/disparity_map_debug",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/droan/expansion_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/expansion_poly",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/fg_bg_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/droan/fg_bg_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/foreground_expanded",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/frustum",
"type": "visualization_msgs/msg/Marker"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/graph_vis",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/graph_vis",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/local_planner_global_plan_vis",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/local_planner_global_plan_vis",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/reset_stuck",
"type": "std_msgs/msg/Empty"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/rewind_info",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/rewind_info",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/stuck",
"type": "std_msgs/msg/Bool"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/traj_debug",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/traj_debug",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/trajectory_library_vis",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/droan/virtual_obstacles",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "pub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/global_plan",
"type": "nav_msgs/msg/Path"
},
{
"dir": "pub",
"node": "/robot_1/random_walk_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/global_plan",
"type": "nav_msgs/msg/Path"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/global_plan",
"type": "nav_msgs/msg/Path"
},
{
"dir": "sub",
"node": "/robot_1/gossip_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/global_plan",
"type": "nav_msgs/msg/Path"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/global_plan",
"type": "nav_msgs/msg/Path"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/attitude_thrust_command",
"type": "mav_msgs/msg/AttitudeThrust"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_attitude_thrust",
"type": "mav_msgs/msg/AttitudeThrust"
},
{
"dir": "pub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_rate_thrust",
"type": "mav_msgs/msg/RateThrust"
},
{
"dir": "pub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_roll_pitch_yawrate_thrust",
"type": "mav_msgs/msg/RollPitchYawrateThrust"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_roll_pitch_yawrate_thrust",
"type": "mav_msgs/msg/RollPitchYawrateThrust"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_torque_thrust",
"type": "mav_msgs/msg/TorqueThrust"
},
{
"dir": "pub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_velocity",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/cmd_velocity",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/has_control",
"type": "std_msgs/msg/Bool"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/has_control",
"type": "std_msgs/msg/Bool"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/is_armed",
"type": "std_msgs/msg/Bool"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/is_armed",
"type": "std_msgs/msg/Bool"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/actuator_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/actuator_control",
"type": "mavros_msgs/msg/ActuatorControl"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/adsb",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/adsb/send",
"type": "mavros_msgs/msg/ADSBVehicle"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/adsb",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/adsb/vehicle",
"type": "mavros_msgs/msg/ADSBVehicle"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/altitude",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/altitude",
"type": "mavros_msgs/msg/Altitude"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/battery",
"type": "sensor_msgs/msg/BatteryState"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/cam_imu_sync",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/cam_imu_sync/cam_imu_stamp",
"type": "mavros_msgs/msg/CamIMUStamp"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/camera",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/camera/image_captured",
"type": "mavros_msgs/msg/CameraImageCaptured"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/cellular_status",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/cellular_status/status",
"type": "mavros_msgs/msg/CellularStatus"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/companion_process",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/companion_process/status",
"type": "mavros_msgs/msg/CompanionProcessStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/debug_value",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/debug_value/debug",
"type": "mavros_msgs/msg/DebugValue"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/debug_value",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/debug_value/debug_float_array",
"type": "mavros_msgs/msg/DebugValue"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/debug_value",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/debug_value/debug_vector",
"type": "mavros_msgs/msg/DebugValue"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/debug_value",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/debug_value/named_value_float",
"type": "mavros_msgs/msg/DebugValue"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/debug_value",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/debug_value/named_value_int",
"type": "mavros_msgs/msg/DebugValue"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/debug_value",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/debug_value/send",
"type": "mavros_msgs/msg/DebugValue"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/esc_status",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/esc_status/info",
"type": "mavros_msgs/msg/ESCInfo"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/esc_status",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/esc_status/status",
"type": "mavros_msgs/msg/ESCStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/esc_telemetry",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/esc_telemetry/telemetry",
"type": "mavros_msgs/msg/ESCTelemetry"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/estimator_status",
"type": "mavros_msgs/msg/EstimatorStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/extended_state",
"type": "mavros_msgs/msg/ExtendedState"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/extended_state",
"type": "mavros_msgs/msg/ExtendedState"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/fake_gps",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/fake_gps/mocap/tf",
"type": "geometry_msgs/msg/TransformStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/geofence",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/geofence/fences",
"type": "mavros_msgs/msg/WaypointList"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/device/attitude_status",
"type": "mavros_msgs/msg/GimbalDeviceAttitudeStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/device/info",
"type": "mavros_msgs/msg/GimbalDeviceInformation"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/device/set_attitude",
"type": "mavros_msgs/msg/GimbalDeviceSetAttitude"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/manager/info",
"type": "mavros_msgs/msg/GimbalManagerInformation"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/manager/set_attitude",
"type": "mavros_msgs/msg/GimbalManagerSetAttitude"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/manager/set_manual_control",
"type": "mavros_msgs/msg/GimbalManagerSetPitchyaw"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/manager/set_pitchyaw",
"type": "mavros_msgs/msg/GimbalManagerSetPitchyaw"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gimbal_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gimbal_control/manager/status",
"type": "mavros_msgs/msg/GimbalManagerStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/compass_hdg",
"type": "std_msgs/msg/Float64"
},
{
"dir": "sub",
"node": "/robot_1/gossip_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/compass_hdg",
"type": "std_msgs/msg/Float64"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/global",
"type": "sensor_msgs/msg/NavSatFix"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/global",
"type": "sensor_msgs/msg/NavSatFix"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/global",
"type": "sensor_msgs/msg/NavSatFix"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/gp_lp_offset",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/global_position/gp_origin",
"type": "geographic_msgs/msg/GeoPointStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/guided_target",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/global_position/gp_origin",
"type": "geographic_msgs/msg/GeoPointStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/local",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/local",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/raw/fix",
"type": "sensor_msgs/msg/NavSatFix"
},
{
"dir": "sub",
"node": "/robot_1/gossip_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/raw/fix",
"type": "sensor_msgs/msg/NavSatFix"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/raw/gps_vel",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/raw/satellites",
"type": "std_msgs/msg/UInt32"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/rel_alt",
"type": "std_msgs/msg/Float64"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/global_position/set_gp_origin",
"type": "geographic_msgs/msg/GeoPointStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gps_input",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gps_input/gps_input",
"type": "mavros_msgs/msg/GPSINPUT"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gps_rtk",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gps_rtk/rtk_baseline",
"type": "mavros_msgs/msg/RTKBaseline"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/gps_rtk",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gps_rtk/send_rtcm",
"type": "mavros_msgs/msg/RTCM"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gpsstatus",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gpsstatus/gps1/raw",
"type": "mavros_msgs/msg/GPSRAW"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gpsstatus",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gpsstatus/gps1/rtk",
"type": "mavros_msgs/msg/GPSRTK"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gpsstatus",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gpsstatus/gps2/raw",
"type": "mavros_msgs/msg/GPSRAW"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/gpsstatus",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/gpsstatus/gps2/rtk",
"type": "mavros_msgs/msg/GPSRTK"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/hil/actuator_controls",
"type": "mavros_msgs/msg/HilActuatorControls"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/hil/controls",
"type": "mavros_msgs/msg/HilControls"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/hil/gps",
"type": "mavros_msgs/msg/HilGPS"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/hil/imu_ned",
"type": "mavros_msgs/msg/HilSensor"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/hil/optical_flow",
"type": "mavros_msgs/msg/OpticalFlowRad"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/hil/rc_inputs",
"type": "mavros_msgs/msg/RCIn"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/hil",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/hil/state",
"type": "mavros_msgs/msg/HilStateQuaternion"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/home_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/home_position/home",
"type": "mavros_msgs/msg/HomePosition"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/global_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/home_position/home",
"type": "mavros_msgs/msg/HomePosition"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/home_position/home",
"type": "mavros_msgs/msg/HomePosition"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/home_position/set",
"type": "mavros_msgs/msg/HomePosition"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/home_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/home_position/set",
"type": "mavros_msgs/msg/HomePosition"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/imu/data",
"type": "sensor_msgs/msg/Imu"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/imu/data_raw",
"type": "sensor_msgs/msg/Imu"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/imu/diff_pressure",
"type": "sensor_msgs/msg/FluidPressure"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/imu/mag",
"type": "sensor_msgs/msg/MagneticField"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/imu/static_pressure",
"type": "sensor_msgs/msg/FluidPressure"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/imu/temperature_baro",
"type": "sensor_msgs/msg/Temperature"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/imu",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/imu/temperature_imu",
"type": "sensor_msgs/msg/Temperature"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/landing_target",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/landing_target/lt_marker",
"type": "geometry_msgs/msg/Vector3Stamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/landing_target",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/landing_target/pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/landing_target",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/landing_target/pose_in",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/accel",
"type": "geometry_msgs/msg/AccelWithCovarianceStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/odom",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/odometry_conversion/odometry_conversion",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/odom",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/pose_cov",
"type": "geometry_msgs/msg/PoseWithCovarianceStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/velocity_body",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/velocity_body_cov",
"type": "geometry_msgs/msg/TwistWithCovarianceStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/local_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/local_position/velocity_local",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/log_transfer",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/log_transfer/raw/log_data",
"type": "mavros_msgs/msg/LogData"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/log_transfer",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/log_transfer/raw/log_entry",
"type": "mavros_msgs/msg/LogEntry"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mag_calibration",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mag_calibration/report",
"type": "mavros_msgs/msg/MagnetometerReporter"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mag_calibration",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mag_calibration/status",
"type": "std_msgs/msg/UInt8"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/manual_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/manual_control/control",
"type": "mavros_msgs/msg/ManualControl"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/manual_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/manual_control/send",
"type": "mavros_msgs/msg/ManualControl"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mission",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mission/reached",
"type": "mavros_msgs/msg/WaypointReached"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mission",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mission/waypoints",
"type": "mavros_msgs/msg/WaypointList"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mocap",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mocap/pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mocap",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mocap/tf",
"type": "geometry_msgs/msg/TransformStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mount_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mount_control/command",
"type": "mavros_msgs/msg/MountControl"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mount_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mount_control/orientation",
"type": "geometry_msgs/msg/Quaternion"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mount_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/mount_control/status",
"type": "geometry_msgs/msg/Vector3Stamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/nav_controller_output",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/nav_controller_output/output",
"type": "mavros_msgs/msg/NavControllerOutput"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/obstacle",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/obstacle/send",
"type": "sensor_msgs/msg/LaserScan"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/obstacle_distance_3d",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/obstacle_distance_3d/send",
"type": "mavros_msgs/msg/ObstacleDistance3D"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/odometry",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/odometry/in",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/odometry",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/odometry/out",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/onboard_computer",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/onboard_computer/status",
"type": "mavros_msgs/msg/OnboardComputerStatus"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/open_drone_id",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/open_drone_id/basic_id",
"type": "mavros_msgs/msg/OpenDroneIDBasicID"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/open_drone_id",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/open_drone_id/operator_id",
"type": "mavros_msgs/msg/OpenDroneIDOperatorID"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/open_drone_id",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/open_drone_id/self_id",
"type": "mavros_msgs/msg/OpenDroneIDSelfID"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/open_drone_id",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/open_drone_id/system",
"type": "mavros_msgs/msg/OpenDroneIDSystem"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/open_drone_id",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/open_drone_id/system_update",
"type": "mavros_msgs/msg/OpenDroneIDSystemUpdate"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/optical_flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/optical_flow/ground_distance",
"type": "sensor_msgs/msg/Range"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/optical_flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/optical_flow/raw/optical_flow",
"type": "mavros_msgs/msg/OpticalFlow"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/optical_flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/optical_flow/raw/send",
"type": "mavros_msgs/msg/OpticalFlow"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/param",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/param/event",
"type": "mavros_msgs/msg/ParamEvent"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/play_tune",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/play_tune",
"type": "mavros_msgs/msg/PlayTuneV2"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/px4flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/px4flow/ground_distance",
"type": "sensor_msgs/msg/Range"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/px4flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/px4flow/raw/optical_flow_rad",
"type": "mavros_msgs/msg/OpticalFlowRad"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/px4flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/px4flow/raw/send",
"type": "mavros_msgs/msg/OpticalFlowRad"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/px4flow",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/px4flow/temperature",
"type": "sensor_msgs/msg/Temperature"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/tdr_radio",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/radio_status",
"type": "mavros_msgs/msg/RadioStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/rallypoint",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/rallypoint/rallypoints",
"type": "mavros_msgs/msg/WaypointList"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/rc",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/rc/in",
"type": "mavros_msgs/msg/RCIn"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/rc",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/rc/out",
"type": "mavros_msgs/msg/RCOut"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/rc",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/rc/override",
"type": "mavros_msgs/msg/OverrideRCIn"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_accel",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_accel/accel",
"type": "geometry_msgs/msg/Vector3Stamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_attitude",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/setpoint_attitude/cmd_vel",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_attitude",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/setpoint_attitude/thrust",
"type": "mavros_msgs/msg/Thrust"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/setpoint_position/global",
"type": "geographic_msgs/msg/GeoPoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_position/global",
"type": "geographic_msgs/msg/GeoPoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_position/global_to_local",
"type": "geographic_msgs/msg/GeoPoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/setpoint_position/local",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_position",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_position/local",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/attitude",
"type": "mavros_msgs/msg/AttitudeTarget"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_raw",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/attitude",
"type": "mavros_msgs/msg/AttitudeTarget"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_raw",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/global",
"type": "mavros_msgs/msg/GlobalPositionTarget"
},
{
"dir": "pub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/local",
"type": "mavros_msgs/msg/PositionTarget"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_raw",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/local",
"type": "mavros_msgs/msg/PositionTarget"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/setpoint_raw",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/target_attitude",
"type": "mavros_msgs/msg/AttitudeTarget"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/setpoint_raw",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/target_global",
"type": "mavros_msgs/msg/GlobalPositionTarget"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/setpoint_raw",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_raw/target_local",
"type": "mavros_msgs/msg/PositionTarget"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/setpoint_trajectory",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_trajectory/desired",
"type": "nav_msgs/msg/Path"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_trajectory",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_trajectory/local",
"type": "trajectory_msgs/msg/MultiDOFJointTrajectory"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_velocity",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_velocity/cmd_vel",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/setpoint_velocity",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/setpoint_velocity/cmd_vel_unstamped",
"type": "geometry_msgs/msg/Twist"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sim_state",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/sim_state/acceleration",
"type": "geometry_msgs/msg/Vector3Stamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sim_state",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/sim_state/attitude",
"type": "sensor_msgs/msg/Imu"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sim_state",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/sim_state/global_position",
"type": "sensor_msgs/msg/NavSatFix"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sim_state",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/sim_state/velocity_body",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sim_state",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/sim_state/velocity_local",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/state",
"type": "mavros_msgs/msg/State"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/state",
"type": "mavros_msgs/msg/State"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/status_event",
"type": "mavros_msgs/msg/StatusEvent"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/statustext/recv",
"type": "mavros_msgs/msg/StatusText"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/statustext/send",
"type": "mavros_msgs/msg/StatusText"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/sys",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/sys_status",
"type": "mavros_msgs/msg/SysStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/actuator_control",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/target_actuator_control",
"type": "mavros_msgs/msg/ActuatorControl"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/terrain",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/terrain/report",
"type": "mavros_msgs/msg/TerrainReport"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/time",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/time_reference",
"type": "sensor_msgs/msg/TimeReference"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/time",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/timesync_status",
"type": "mavros_msgs/msg/TimesyncStatus"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/trajectory",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/trajectory/desired",
"type": "mavros_msgs/msg/Trajectory"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/trajectory",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/trajectory/generated",
"type": "mavros_msgs/msg/Trajectory"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/trajectory",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/trajectory/path",
"type": "nav_msgs/msg/Path"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/tunnel",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/tunnel/in",
"type": "mavros_msgs/msg/Tunnel"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/tunnel",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/tunnel/out",
"type": "mavros_msgs/msg/Tunnel"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/vfr_hud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/vfr_hud",
"type": "mavros_msgs/msg/VfrHud"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vision_pose",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/vision_pose/pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vision_pose",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/vision_pose/pose_cov",
"type": "geometry_msgs/msg/PoseWithCovarianceStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vision_speed",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/vision_speed/speed_twist",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vision_speed",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/vision_speed/speed_twist_cov",
"type": "geometry_msgs/msg/TwistWithCovarianceStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/vision_speed",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/mavros/vision_speed/speed_vector",
"type": "geometry_msgs/msg/Vector3Stamped"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/wind",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/interface/mavros/wind_estimation",
"type": "geometry_msgs/msg/TwistWithCovarianceStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/pose_command",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/rate_thrust_command",
"type": "mav_msgs/msg/RateThrust"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/roll_pitch_yawrate_thrust_command",
"type": "mav_msgs/msg/RollPitchYawrateThrust"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/torque_thrust_command",
"type": "mav_msgs/msg/TorqueThrust"
},
{
"dir": "sub",
"node": "/robot_1/interface/robot_interface",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/interface/velocity_command",
"type": "geometry_msgs/msg/TwistStamped"
},
{
"dir": "sub",
"node": "/robot_1/robot_state_publisher",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/joint_states",
"type": "sensor_msgs/msg/JointState"
},
{
"dir": "pub",
"node": "/robot_1/odometry_conversion/odometry_conversion",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/behavior/drone_safety_monitor/drone_safety_monitor",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/random_walk_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/trajectory_controller/fixed_trajectory_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/odometry_conversion/odometry",
"type": "nav_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/perception/stereo_image_proc/disparity_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/perception/stereo_image_proc/disparity",
"type": "stereo_msgs/msg/DisparityImage"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/perception/stereo_image_proc/disparity",
"type": "stereo_msgs/msg/DisparityImage"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_pointcloud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/perception/stereo_image_proc/disparity",
"type": "stereo_msgs/msg/DisparityImage"
},
{
"dir": "pub",
"node": "/robot_1/perception/stereo_pointcloud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/perception/stereo_image_proc/point_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/perception/stereo_image_proc/point_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "pub",
"node": "/robot_1/random_walk_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/random_walk_node/goal_point_viz",
"type": "visualization_msgs/msg/Marker"
},
{
"dir": "pub",
"node": "/robot_1/random_walk_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/random_walk_node/traj_viz",
"type": "visualization_msgs/msg/Marker"
},
{
"dir": "pub",
"node": "/robot_1/robot_state_publisher",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/robot_description",
"type": "std_msgs/msg/String"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_image_proc/disparity_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/left/camera_info",
"type": "sensor_msgs/msg/CameraInfo"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_pointcloud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/left/camera_info",
"type": "sensor_msgs/msg/CameraInfo"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/sensors/front_stereo/left/camera_info",
"type": "sensor_msgs/msg/CameraInfo"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/left/depth_ground_truth",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_image_proc/disparity_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/left/image_rect",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_pointcloud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/left/image_rect",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/left/image_rect",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/sensors/front_stereo/right/camera_info",
"type": "sensor_msgs/msg/CameraInfo"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_image_proc/disparity_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/right/camera_info",
"type": "sensor_msgs/msg/CameraInfo"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_pointcloud",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/right/camera_info",
"type": "sensor_msgs/msg/CameraInfo"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/sensors/front_stereo/right/camera_info",
"type": "sensor_msgs/msg/CameraInfo"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/right/depth_ground_truth",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/perception/stereo_image_proc/disparity_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/right/image_rect",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/front_stereo/right/image_rect",
"type": "sensor_msgs/msg/Image"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/lidar/point_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "pub",
"node": "/robot_1/sensors/lidar_point_cloud_filter",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/sensors/ouster/point_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "sub",
"node": "/robot_1/vdb_mapping",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/sensors/ouster/point_cloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "sub",
"node": "/robot_1/sensors/lidar_point_cloud_filter",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/sensors/ouster/point_cloud_raw",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "pub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/takeoff_landing_planner/is_airborne",
"type": "std_msgs/msg/Bool"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/takeoff_landing_planner/trajectory_completion_percentage",
"type": "std_msgs/msg/Float32"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/closest_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/look_ahead",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/look_ahead",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/projected_drone_pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "sub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/projected_drone_pose",
"type": "geometry_msgs/msg/PoseStamped"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/tracking_error",
"type": "std_msgs/msg/Float32"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/tracking_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/control/pid_controller",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/tracking_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/tracking_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/tracking_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "sub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/tracking_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/tracking_point_velocity_magnitude",
"type": "std_msgs/msg/Float32"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/traj_drone_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_completion_percentage",
"type": "std_msgs/msg/Float32"
},
{
"dir": "sub",
"node": "/robot_1/trajectory_controller/fixed_trajectory_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_completion_percentage",
"type": "std_msgs/msg/Float32"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_controller_debug_markers",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_controller_debug_markers",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "pub",
"node": "/robot_1/takeoff_landing_planner/takeoff_landing_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_override",
"type": "airstack_msgs/msg/TrajectoryXYZVYaw"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/fixed_trajectory_task",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_override",
"type": "airstack_msgs/msg/TrajectoryXYZVYaw"
},
{
"dir": "sub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_override",
"type": "airstack_msgs/msg/TrajectoryXYZVYaw"
},
{
"dir": "pub",
"node": "/robot_1/droan/disparity_expander_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_segment_to_add",
"type": "airstack_msgs/msg/TrajectoryXYZVYaw"
},
{
"dir": "sub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_segment_to_add",
"type": "airstack_msgs/msg/TrajectoryXYZVYaw"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_time",
"type": "std_msgs/msg/Float32"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_vis",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/trajectory_vis",
"type": "visualization_msgs/msg/MarkerArray"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/trajectory_controller/virtual_tracking_point",
"type": "airstack_msgs/msg/Odometry"
},
{
"dir": "pub",
"node": "/robot_1/vdb_mapping",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/vdb_mapping/vdb_map_overwrites",
"type": "vdb_mapping_interfaces/msg/UpdateGrid"
},
{
"dir": "pub",
"node": "/robot_1/vdb_mapping",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/vdb_mapping/vdb_map_pointcloud",
"type": "sensor_msgs/msg/PointCloud2"
},
{
"dir": "pub",
"node": "/robot_1/vdb_mapping",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/vdb_mapping/vdb_map_sections",
"type": "vdb_mapping_interfaces/msg/UpdateGrid"
},
{
"dir": "pub",
"node": "/robot_1/vdb_mapping",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/robot_1/vdb_mapping/vdb_map_updates",
"type": "vdb_mapping_interfaces/msg/UpdateGrid"
},
{
"dir": "pub",
"node": "/robot_1/vdb_mapping",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/vdb_mapping/vdb_map_visualization",
"type": "visualization_msgs/msg/Marker"
},
{
"dir": "sub",
"node": "/robot_1/random_walk_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/vdb_mapping/vdb_map_visualization",
"type": "visualization_msgs/msg/Marker"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/robot_1/vdb_mapping/vdb_map_visualization",
"type": "visualization_msgs/msg/Marker"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mavros",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "pub",
"node": "/robot_1/odometry_conversion/odometry_conversion",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "pub",
"node": "/robot_1/robot_state_publisher",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "pub",
"node": "/robot_1/trajectory_controller/trajectory_control_node",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "sub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mavros",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf_static",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "pub",
"node": "/robot_1/robot_state_publisher",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf_static",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "pub",
"node": "/robot_1/world_to_map_broadcaster",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf_static",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "sub",
"node": "/robot_1/interface/odom_modifier",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf_static",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "sub",
"node": "/robot_1/topic_keepalive",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"topic": "/tf_static",
"type": "tf2_msgs/msg/TFMessage"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mavros",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/uas2/mavlink_sink",
"type": "mavros_msgs/msg/Mavlink"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mavros_router",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/uas2/mavlink_sink",
"type": "mavros_msgs/msg/Mavlink"
},
{
"dir": "pub",
"node": "/robot_1/interface/mavros/mavros_router",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/uas2/mavlink_source",
"type": "mavros_msgs/msg/Mavlink"
},
{
"dir": "sub",
"node": "/robot_1/interface/mavros/mavros",
"qos_profile": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"topic": "/uas2/mavlink_source",
"type": "mavros_msgs/msg/Mavlink"
}
],
"nodes": [
"/action_relay_client",
"/robot_1/Container",
"/robot_1/behavior/drone_safety_monitor/drone_safety_monitor",
"/robot_1/control/pid_controller",
"/robot_1/droan/disparity_expander_node",
"/robot_1/gossip_node",
"/robot_1/interface/mavros/actuator_control",
"/robot_1/interface/mavros/adsb",
"/robot_1/interface/mavros/altitude",
"/robot_1/interface/mavros/cam_imu_sync",
"/robot_1/interface/mavros/camera",
"/robot_1/interface/mavros/cellular_status",
"/robot_1/interface/mavros/cmd",
"/robot_1/interface/mavros/companion_process",
"/robot_1/interface/mavros/debug_value",
"/robot_1/interface/mavros/esc_status",
"/robot_1/interface/mavros/esc_telemetry",
"/robot_1/interface/mavros/fake_gps",
"/robot_1/interface/mavros/ftp",
"/robot_1/interface/mavros/geofence",
"/robot_1/interface/mavros/gimbal_control",
"/robot_1/interface/mavros/global_position",
"/robot_1/interface/mavros/gps_input",
"/robot_1/interface/mavros/gps_rtk",
"/robot_1/interface/mavros/gpsstatus",
"/robot_1/interface/mavros/guided_target",
"/robot_1/interface/mavros/hil",
"/robot_1/interface/mavros/home_position",
"/robot_1/interface/mavros/imu",
"/robot_1/interface/mavros/landing_target",
"/robot_1/interface/mavros/local_position",
"/robot_1/interface/mavros/log_transfer",
"/robot_1/interface/mavros/mag_calibration",
"/robot_1/interface/mavros/manual_control",
"/robot_1/interface/mavros/mavros",
"/robot_1/interface/mavros/mavros_node",
"/robot_1/interface/mavros/mavros_router",
"/robot_1/interface/mavros/mission",
"/robot_1/interface/mavros/mocap",
"/robot_1/interface/mavros/mount_control",
"/robot_1/interface/mavros/nav_controller_output",
"/robot_1/interface/mavros/obstacle",
"/robot_1/interface/mavros/obstacle_distance_3d",
"/robot_1/interface/mavros/odometry",
"/robot_1/interface/mavros/onboard_computer",
"/robot_1/interface/mavros/open_drone_id",
"/robot_1/interface/mavros/optical_flow",
"/robot_1/interface/mavros/param",
"/robot_1/interface/mavros/play_tune",
"/robot_1/interface/mavros/px4flow",
"/robot_1/interface/mavros/rallypoint",
"/robot_1/interface/mavros/rc",
"/robot_1/interface/mavros/setpoint_accel",
"/robot_1/interface/mavros/setpoint_attitude",
"/robot_1/interface/mavros/setpoint_position",
"/robot_1/interface/mavros/setpoint_raw",
"/robot_1/interface/mavros/setpoint_trajectory",
"/robot_1/interface/mavros/setpoint_velocity",
"/robot_1/interface/mavros/sim_state",
"/robot_1/interface/mavros/sys",
"/robot_1/interface/mavros/tdr_radio",
"/robot_1/interface/mavros/terrain",
"/robot_1/interface/mavros/time",
"/robot_1/interface/mavros/trajectory",
"/robot_1/interface/mavros/tunnel",
"/robot_1/interface/mavros/vfr_hud",
"/robot_1/interface/mavros/vision_pose",
"/robot_1/interface/mavros/vision_speed",
"/robot_1/interface/mavros/wind",
"/robot_1/interface/odom_modifier",
"/robot_1/interface/robot_interface",
"/robot_1/odometry_conversion/odometry_conversion",
"/robot_1/perception/stereo_image_proc/disparity_node",
"/robot_1/perception/stereo_pointcloud",
"/robot_1/random_walk_node",
"/robot_1/robot_state_publisher",
"/robot_1/sensors/lidar_point_cloud_filter",
"/robot_1/takeoff_landing_planner/takeoff_landing_task",
"/robot_1/topic_keepalive",
"/robot_1/trajectory_controller/fixed_trajectory_task",
"/robot_1/trajectory_controller/trajectory_control_node",
"/robot_1/vdb_mapping",
"/robot_1/world_to_map_broadcaster"
],
"topics": {
"/clock": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "rosgraph_msgs/msg/Clock"
},
"/diagnostics": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "diagnostic_msgs/msg/DiagnosticArray"
},
"/gossip/peers": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "coordination_msgs/msg/PeerProfile"
},
"/move_base_simple/goal": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/behavior/drone_safety_monitor/command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/String"
},
"/robot_1/behavior/drone_safety_monitor/state_estimate_timed_out": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Bool"
},
"/robot_1/control/reset_integrators": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Empty"
},
"/robot_1/control/vx_pid_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "pid_controller_msgs/msg/PIDInfo"
},
"/robot_1/control/vy_pid_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "pid_controller_msgs/msg/PIDInfo"
},
"/robot_1/control/vz_pid_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "pid_controller_msgs/msg/PIDInfo"
},
"/robot_1/control/x_pid_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "pid_controller_msgs/msg/PIDInfo"
},
"/robot_1/control/y_pid_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "pid_controller_msgs/msg/PIDInfo"
},
"/robot_1/control/z_pid_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "pid_controller_msgs/msg/PIDInfo"
},
"/robot_1/coordination/peer_registry": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "coordination_msgs/msg/PeerProfile"
},
"/robot_1/cross_track_error": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/droan/background_expanded": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Image"
},
"/robot_1/droan/clear_map": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Empty"
},
"/robot_1/droan/disparity_graph": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/disparity_map_debug": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/expansion_cloud": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/PointCloud2"
},
"/robot_1/droan/expansion_poly": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/fg_bg_cloud": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/PointCloud2"
},
"/robot_1/droan/foreground_expanded": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Image"
},
"/robot_1/droan/frustum": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/Marker"
},
"/robot_1/droan/graph_vis": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/local_planner_global_plan_vis": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/reset_stuck": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Empty"
},
"/robot_1/droan/rewind_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/stuck": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Bool"
},
"/robot_1/droan/traj_debug": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/trajectory_library_vis": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/droan/virtual_obstacles": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/global_plan": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "nav_msgs/msg/Path"
},
"/robot_1/interface/attitude_thrust_command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/AttitudeThrust"
},
"/robot_1/interface/cmd_attitude_thrust": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/AttitudeThrust"
},
"/robot_1/interface/cmd_pose": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/cmd_rate_thrust": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/RateThrust"
},
"/robot_1/interface/cmd_roll_pitch_yawrate_thrust": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/RollPitchYawrateThrust"
},
"/robot_1/interface/cmd_torque_thrust": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/TorqueThrust"
},
"/robot_1/interface/cmd_velocity": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/has_control": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Bool"
},
"/robot_1/interface/is_armed": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Bool"
},
"/robot_1/interface/mavros/actuator_control": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/ActuatorControl"
},
"/robot_1/interface/mavros/adsb/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ADSBVehicle"
},
"/robot_1/interface/mavros/adsb/vehicle": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ADSBVehicle"
},
"/robot_1/interface/mavros/altitude": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/Altitude"
},
"/robot_1/interface/mavros/battery": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/BatteryState"
},
"/robot_1/interface/mavros/cam_imu_sync/cam_imu_stamp": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/CamIMUStamp"
},
"/robot_1/interface/mavros/camera/image_captured": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/CameraImageCaptured"
},
"/robot_1/interface/mavros/cellular_status/status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/CellularStatus"
},
"/robot_1/interface/mavros/companion_process/status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/CompanionProcessStatus"
},
"/robot_1/interface/mavros/debug_value/debug": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/DebugValue"
},
"/robot_1/interface/mavros/debug_value/debug_float_array": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/DebugValue"
},
"/robot_1/interface/mavros/debug_value/debug_vector": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/DebugValue"
},
"/robot_1/interface/mavros/debug_value/named_value_float": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/DebugValue"
},
"/robot_1/interface/mavros/debug_value/named_value_int": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/DebugValue"
},
"/robot_1/interface/mavros/debug_value/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/DebugValue"
},
"/robot_1/interface/mavros/esc_status/info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ESCInfo"
},
"/robot_1/interface/mavros/esc_status/status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ESCStatus"
},
"/robot_1/interface/mavros/esc_telemetry/telemetry": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ESCTelemetry"
},
"/robot_1/interface/mavros/estimator_status": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/EstimatorStatus"
},
"/robot_1/interface/mavros/extended_state": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ExtendedState"
},
"/robot_1/interface/mavros/fake_gps/mocap/tf": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TransformStamped"
},
"/robot_1/interface/mavros/geofence/fences": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/WaypointList"
},
"/robot_1/interface/mavros/gimbal_control/device/attitude_status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalDeviceAttitudeStatus"
},
"/robot_1/interface/mavros/gimbal_control/device/info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalDeviceInformation"
},
"/robot_1/interface/mavros/gimbal_control/device/set_attitude": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalDeviceSetAttitude"
},
"/robot_1/interface/mavros/gimbal_control/manager/info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalManagerInformation"
},
"/robot_1/interface/mavros/gimbal_control/manager/set_attitude": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalManagerSetAttitude"
},
"/robot_1/interface/mavros/gimbal_control/manager/set_manual_control": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalManagerSetPitchyaw"
},
"/robot_1/interface/mavros/gimbal_control/manager/set_pitchyaw": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalManagerSetPitchyaw"
},
"/robot_1/interface/mavros/gimbal_control/manager/status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GimbalManagerStatus"
},
"/robot_1/interface/mavros/global_position/compass_hdg": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "std_msgs/msg/Float64"
},
"/robot_1/interface/mavros/global_position/global": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/NavSatFix"
},
"/robot_1/interface/mavros/global_position/gp_lp_offset": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/mavros/global_position/gp_origin": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geographic_msgs/msg/GeoPointStamped"
},
"/robot_1/interface/mavros/global_position/local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "nav_msgs/msg/Odometry"
},
"/robot_1/interface/mavros/global_position/raw/fix": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/NavSatFix"
},
"/robot_1/interface/mavros/global_position/raw/gps_vel": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/global_position/raw/satellites": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "std_msgs/msg/UInt32"
},
"/robot_1/interface/mavros/global_position/rel_alt": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "std_msgs/msg/Float64"
},
"/robot_1/interface/mavros/global_position/set_gp_origin": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geographic_msgs/msg/GeoPointStamped"
},
"/robot_1/interface/mavros/gps_input/gps_input": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GPSINPUT"
},
"/robot_1/interface/mavros/gps_rtk/rtk_baseline": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/RTKBaseline"
},
"/robot_1/interface/mavros/gps_rtk/send_rtcm": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/RTCM"
},
"/robot_1/interface/mavros/gpsstatus/gps1/raw": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GPSRAW"
},
"/robot_1/interface/mavros/gpsstatus/gps1/rtk": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GPSRTK"
},
"/robot_1/interface/mavros/gpsstatus/gps2/raw": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GPSRAW"
},
"/robot_1/interface/mavros/gpsstatus/gps2/rtk": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/GPSRTK"
},
"/robot_1/interface/mavros/hil/actuator_controls": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/HilActuatorControls"
},
"/robot_1/interface/mavros/hil/controls": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/HilControls"
},
"/robot_1/interface/mavros/hil/gps": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/HilGPS"
},
"/robot_1/interface/mavros/hil/imu_ned": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/HilSensor"
},
"/robot_1/interface/mavros/hil/optical_flow": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpticalFlowRad"
},
"/robot_1/interface/mavros/hil/rc_inputs": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/RCIn"
},
"/robot_1/interface/mavros/hil/state": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/HilStateQuaternion"
},
"/robot_1/interface/mavros/home_position/home": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/HomePosition"
},
"/robot_1/interface/mavros/home_position/set": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/HomePosition"
},
"/robot_1/interface/mavros/imu/data": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/Imu"
},
"/robot_1/interface/mavros/imu/data_raw": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/Imu"
},
"/robot_1/interface/mavros/imu/diff_pressure": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/FluidPressure"
},
"/robot_1/interface/mavros/imu/mag": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/MagneticField"
},
"/robot_1/interface/mavros/imu/static_pressure": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/FluidPressure"
},
"/robot_1/interface/mavros/imu/temperature_baro": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/Temperature"
},
"/robot_1/interface/mavros/imu/temperature_imu": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/Temperature"
},
"/robot_1/interface/mavros/landing_target/lt_marker": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/Vector3Stamped"
},
"/robot_1/interface/mavros/landing_target/pose": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/mavros/landing_target/pose_in": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/mavros/local_position/accel": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/AccelWithCovarianceStamped"
},
"/robot_1/interface/mavros/local_position/odom": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "nav_msgs/msg/Odometry"
},
"/robot_1/interface/mavros/local_position/pose": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/mavros/local_position/pose_cov": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/PoseWithCovarianceStamped"
},
"/robot_1/interface/mavros/local_position/velocity_body": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/local_position/velocity_body_cov": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/TwistWithCovarianceStamped"
},
"/robot_1/interface/mavros/local_position/velocity_local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/log_transfer/raw/log_data": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/LogData"
},
"/robot_1/interface/mavros/log_transfer/raw/log_entry": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/LogEntry"
},
"/robot_1/interface/mavros/mag_calibration/report": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/MagnetometerReporter"
},
"/robot_1/interface/mavros/mag_calibration/status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/UInt8"
},
"/robot_1/interface/mavros/manual_control/control": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ManualControl"
},
"/robot_1/interface/mavros/manual_control/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ManualControl"
},
"/robot_1/interface/mavros/mission/reached": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/WaypointReached"
},
"/robot_1/interface/mavros/mission/waypoints": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/WaypointList"
},
"/robot_1/interface/mavros/mocap/pose": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/mavros/mocap/tf": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TransformStamped"
},
"/robot_1/interface/mavros/mount_control/command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/MountControl"
},
"/robot_1/interface/mavros/mount_control/orientation": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/Quaternion"
},
"/robot_1/interface/mavros/mount_control/status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/Vector3Stamped"
},
"/robot_1/interface/mavros/nav_controller_output/output": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/NavControllerOutput"
},
"/robot_1/interface/mavros/obstacle/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/LaserScan"
},
"/robot_1/interface/mavros/obstacle_distance_3d/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ObstacleDistance3D"
},
"/robot_1/interface/mavros/odometry/in": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "nav_msgs/msg/Odometry"
},
"/robot_1/interface/mavros/odometry/out": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "nav_msgs/msg/Odometry"
},
"/robot_1/interface/mavros/onboard_computer/status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OnboardComputerStatus"
},
"/robot_1/interface/mavros/open_drone_id/basic_id": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpenDroneIDBasicID"
},
"/robot_1/interface/mavros/open_drone_id/operator_id": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpenDroneIDOperatorID"
},
"/robot_1/interface/mavros/open_drone_id/self_id": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpenDroneIDSelfID"
},
"/robot_1/interface/mavros/open_drone_id/system": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpenDroneIDSystem"
},
"/robot_1/interface/mavros/open_drone_id/system_update": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpenDroneIDSystemUpdate"
},
"/robot_1/interface/mavros/optical_flow/ground_distance": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Range"
},
"/robot_1/interface/mavros/optical_flow/raw/optical_flow": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpticalFlow"
},
"/robot_1/interface/mavros/optical_flow/raw/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpticalFlow"
},
"/robot_1/interface/mavros/param/event": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/ParamEvent"
},
"/robot_1/interface/mavros/play_tune": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/PlayTuneV2"
},
"/robot_1/interface/mavros/px4flow/ground_distance": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Range"
},
"/robot_1/interface/mavros/px4flow/raw/optical_flow_rad": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpticalFlowRad"
},
"/robot_1/interface/mavros/px4flow/raw/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OpticalFlowRad"
},
"/robot_1/interface/mavros/px4flow/temperature": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Temperature"
},
"/robot_1/interface/mavros/radio_status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/RadioStatus"
},
"/robot_1/interface/mavros/rallypoint/rallypoints": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/WaypointList"
},
"/robot_1/interface/mavros/rc/in": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/RCIn"
},
"/robot_1/interface/mavros/rc/out": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/RCOut"
},
"/robot_1/interface/mavros/rc/override": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/OverrideRCIn"
},
"/robot_1/interface/mavros/setpoint_accel/accel": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/Vector3Stamped"
},
"/robot_1/interface/mavros/setpoint_attitude/cmd_vel": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/setpoint_attitude/thrust": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/Thrust"
},
"/robot_1/interface/mavros/setpoint_position/global": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geographic_msgs/msg/GeoPoseStamped"
},
"/robot_1/interface/mavros/setpoint_position/global_to_local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geographic_msgs/msg/GeoPoseStamped"
},
"/robot_1/interface/mavros/setpoint_position/local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/mavros/setpoint_raw/attitude": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/AttitudeTarget"
},
"/robot_1/interface/mavros/setpoint_raw/global": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/GlobalPositionTarget"
},
"/robot_1/interface/mavros/setpoint_raw/local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/PositionTarget"
},
"/robot_1/interface/mavros/setpoint_raw/target_attitude": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/AttitudeTarget"
},
"/robot_1/interface/mavros/setpoint_raw/target_global": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/GlobalPositionTarget"
},
"/robot_1/interface/mavros/setpoint_raw/target_local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/PositionTarget"
},
"/robot_1/interface/mavros/setpoint_trajectory/desired": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "nav_msgs/msg/Path"
},
"/robot_1/interface/mavros/setpoint_trajectory/local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "trajectory_msgs/msg/MultiDOFJointTrajectory"
},
"/robot_1/interface/mavros/setpoint_velocity/cmd_vel": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/setpoint_velocity/cmd_vel_unstamped": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/Twist"
},
"/robot_1/interface/mavros/sim_state/acceleration": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/Vector3Stamped"
},
"/robot_1/interface/mavros/sim_state/attitude": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Imu"
},
"/robot_1/interface/mavros/sim_state/global_position": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/NavSatFix"
},
"/robot_1/interface/mavros/sim_state/velocity_body": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/sim_state/velocity_local": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/state": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/State"
},
"/robot_1/interface/mavros/status_event": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/StatusEvent"
},
"/robot_1/interface/mavros/statustext/recv": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/StatusText"
},
"/robot_1/interface/mavros/statustext/send": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/StatusText"
},
"/robot_1/interface/mavros/sys_status": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/SysStatus"
},
"/robot_1/interface/mavros/target_actuator_control": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/ActuatorControl"
},
"/robot_1/interface/mavros/terrain/report": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/TerrainReport"
},
"/robot_1/interface/mavros/time_reference": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/TimeReference"
},
"/robot_1/interface/mavros/timesync_status": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/TimesyncStatus"
},
"/robot_1/interface/mavros/trajectory/desired": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/Trajectory"
},
"/robot_1/interface/mavros/trajectory/generated": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/Trajectory"
},
"/robot_1/interface/mavros/trajectory/path": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "nav_msgs/msg/Path"
},
"/robot_1/interface/mavros/tunnel/in": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/Tunnel"
},
"/robot_1/interface/mavros/tunnel/out": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/Tunnel"
},
"/robot_1/interface/mavros/vfr_hud": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mavros_msgs/msg/VfrHud"
},
"/robot_1/interface/mavros/vision_pose/pose": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/mavros/vision_pose/pose_cov": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseWithCovarianceStamped"
},
"/robot_1/interface/mavros/vision_speed/speed_twist": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/interface/mavros/vision_speed/speed_twist_cov": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TwistWithCovarianceStamped"
},
"/robot_1/interface/mavros/vision_speed/speed_vector": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/Vector3Stamped"
},
"/robot_1/interface/mavros/wind_estimation": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "geometry_msgs/msg/TwistWithCovarianceStamped"
},
"/robot_1/interface/pose_command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/interface/rate_thrust_command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/RateThrust"
},
"/robot_1/interface/roll_pitch_yawrate_thrust_command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/RollPitchYawrateThrust"
},
"/robot_1/interface/torque_thrust_command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "mav_msgs/msg/TorqueThrust"
},
"/robot_1/interface/velocity_command": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/TwistStamped"
},
"/robot_1/joint_states": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/JointState"
},
"/robot_1/odometry_conversion/odometry": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "nav_msgs/msg/Odometry"
},
"/robot_1/perception/stereo_image_proc/disparity": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "stereo_msgs/msg/DisparityImage"
},
"/robot_1/perception/stereo_image_proc/point_cloud": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/PointCloud2"
},
"/robot_1/random_walk_node/goal_point_viz": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/Marker"
},
"/robot_1/random_walk_node/traj_viz": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/Marker"
},
"/robot_1/robot_description": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/String"
},
"/robot_1/sensors/front_stereo/left/camera_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/CameraInfo"
},
"/robot_1/sensors/front_stereo/left/depth_ground_truth": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Image"
},
"/robot_1/sensors/front_stereo/left/image_rect": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Image"
},
"/robot_1/sensors/front_stereo/right/camera_info": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/CameraInfo"
},
"/robot_1/sensors/front_stereo/right/depth_ground_truth": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Image"
},
"/robot_1/sensors/front_stereo/right/image_rect": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/Image"
},
"/robot_1/sensors/lidar/point_cloud": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "sensor_msgs/msg/PointCloud2"
},
"/robot_1/sensors/ouster/point_cloud": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/PointCloud2"
},
"/robot_1/sensors/ouster/point_cloud_raw": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/PointCloud2"
},
"/robot_1/takeoff_landing_planner/is_airborne": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Bool"
},
"/robot_1/takeoff_landing_planner/trajectory_completion_percentage": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Float32"
},
"/robot_1/trajectory_controller/closest_point": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "airstack_msgs/msg/Odometry"
},
"/robot_1/trajectory_controller/look_ahead": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "airstack_msgs/msg/Odometry"
},
"/robot_1/trajectory_controller/projected_drone_pose": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "geometry_msgs/msg/PoseStamped"
},
"/robot_1/trajectory_controller/tracking_error": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Float32"
},
"/robot_1/trajectory_controller/tracking_point": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "airstack_msgs/msg/Odometry"
},
"/robot_1/trajectory_controller/tracking_point_velocity_magnitude": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Float32"
},
"/robot_1/trajectory_controller/traj_drone_point": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "airstack_msgs/msg/Odometry"
},
"/robot_1/trajectory_controller/trajectory_completion_percentage": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Float32"
},
"/robot_1/trajectory_controller/trajectory_controller_debug_markers": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/trajectory_controller/trajectory_override": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "airstack_msgs/msg/TrajectoryXYZVYaw"
},
"/robot_1/trajectory_controller/trajectory_segment_to_add": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "airstack_msgs/msg/TrajectoryXYZVYaw"
},
"/robot_1/trajectory_controller/trajectory_time": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "std_msgs/msg/Float32"
},
"/robot_1/trajectory_controller/trajectory_vis": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/MarkerArray"
},
"/robot_1/trajectory_controller/virtual_tracking_point": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "airstack_msgs/msg/Odometry"
},
"/robot_1/vdb_mapping/vdb_map_overwrites": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "vdb_mapping_interfaces/msg/UpdateGrid"
},
"/robot_1/vdb_mapping/vdb_map_pointcloud": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "sensor_msgs/msg/PointCloud2"
},
"/robot_1/vdb_mapping/vdb_map_sections": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "vdb_mapping_interfaces/msg/UpdateGrid"
},
"/robot_1/vdb_mapping/vdb_map_updates": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "vdb_mapping_interfaces/msg/UpdateGrid"
},
"/robot_1/vdb_mapping/vdb_map_visualization": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "visualization_msgs/msg/Marker"
},
"/tf": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "tf2_msgs/msg/TFMessage"
},
"/tf_static": {
"qos": {
"depth": "UNKNOWN",
"durability": "TRANSIENT_LOCAL",
"history": "UNKNOWN",
"reliability": "RELIABLE"
},
"type": "tf2_msgs/msg/TFMessage"
},
"/uas2/mavlink_sink": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/Mavlink"
},
"/uas2/mavlink_source": {
"qos": {
"depth": "UNKNOWN",
"durability": "VOLATILE",
"history": "UNKNOWN",
"reliability": "BEST_EFFORT"
},
"type": "mavros_msgs/msg/Mavlink"
}
},
"version": 1
}
wiring-graph-v1 -->
