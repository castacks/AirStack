## system.test_liveliness

### Pass rates

| Test                                                    | Pass   | Fail   | Skip   | Rate (baseline → current)   |
|---------------------------------------------------------|--------|--------|--------|-----------------------------|
| test_compute_usage[isaacsim-rob#1]                      | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_gcs_container_running[isaacsim-rob#1]              | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_robot_containers_running[isaacsim-rob#1]           | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sentinel_nodes_present[isaacsim-rob#1]             | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sim_container_running[isaacsim-rob#1]              | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sim_ready_time[isaacsim-rob#1]                     | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_stable[isaacsim-rob#1]                             | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_tmux_panes_have_expected_processes[isaacsim-rob#1] | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |

### Metrics

| Test                                                    | Metric                   | Baseline   | Current   | Change                |
|---------------------------------------------------------|--------------------------|------------|-----------|-----------------------|
| test_robot_containers_running[isaacsim-rob#1]           | duration_s               | 11.51s     | 0.972s    | -91.6% :green_circle: |
| test_robot_containers_running[isaacsim-rob#1]           | airstack_up_duration_s   | 0.55s      | 0.73s     | +32.7% :red_circle:   |
| test_robot_containers_running[isaacsim-rob#1]           | airstack_down_duration_s | 11.15s     | 11.27s    | +1.1%                 |
| test_sim_container_running[isaacsim-rob#1]              | duration_s               | 0.012s     | 0.012s    | +0.0%                 |
| test_gcs_container_running[isaacsim-rob#1]              | duration_s               | 0.022s     | 0.021s    | -4.5%                 |
| test_sim_ready_time[isaacsim-rob#1]                     | duration_s               | 19.34s     | 76.03s    | +293.1% :red_circle:  |
| test_sim_ready_time[isaacsim-rob#1]                     | sim_ready_duration_s     | 19.95s     | 76.82s    | +285.1% :red_circle:  |
| test_tmux_panes_have_expected_processes[isaacsim-rob#1] | duration_s               | 0.112s     | 0.071s    | -36.6% :green_circle: |
| test_compute_usage[isaacsim-rob#1]                      | duration_s               | 2.622s     | 2.578s    | -1.7%                 |
| test_sentinel_nodes_present[isaacsim-rob#1]             | duration_s               | 1.495s     | 6.824s    | +356.5% :red_circle:  |
| test_stable[isaacsim-rob#1]                             | duration_s               | 170.4s     | 169.4s    | -0.6%                 |

### Compute usage (n=1 iterations; baseline → current, per-container and global)

| Test                        | Entity                 | Metric      | mean                                    | start_mean                                         | end_mean                                               | min                                    | max                                         |
|-----------------------------|------------------------|-------------|-----------------------------------------|----------------------------------------------------|--------------------------------------------------------|----------------------------------------|---------------------------------------------|
| test_stable[isaacsim-rob#1] | isaac-sim              | cpu_pct     | 1103 → 795.8 (-27.9%) :green_circle:    | 1407 → 769.3 (t=10-60s, -45.3%) :green_circle:     | 800.1 → 822.3 (t=70-120s, +2.8%)                       | 104.5 → 668.6 (+540.1%) :red_circle:   | 2885 → 857.5 (-70.3%) :green_circle:        |
| test_stable[isaacsim-rob#1] | isaac-sim              | mem_mb      | 1.112e+04 → 1.077e+04 (-3.1%)           | 1.083e+04 → 1.077e+04 (t=10-60s, -0.6%)            | 1.14e+04 → 1.077e+04 (t=70-120s, -5.5%)                | 8007 → 1.077e+04 (+34.5%) :red_circle: | 1.178e+04 → 1.077e+04 (-8.5%)               |
| test_stable[isaacsim-rob#1] | isaac-sim              | disk_io_mb  | 808.2 → 815.7 (+0.9%)                   | 591.8 → 800.8 (t=10-60s, +35.3%) :red_circle:      | 1024 → 830.5 (t=70-120s, -18.9%)                       | 322 → 788 (+144.7%) :red_circle:       | 1037 → 843 (-18.7%)                         |
| test_stable[isaacsim-rob#1] | isaac-sim              | net_io_mb   | 7618 → 2.004e+04 (+163.1%) :red_circle: | 1035 → 1.174e+04 (t=10-60s, +1034.9%) :red_circle: | 1.42e+04 → 2.834e+04 (t=70-120s, +99.6%) :red_circle:  | 29.4 → 4478 (+15130.3%) :red_circle:   | 2.093e+04 → 3.534e+04 (+68.8%) :red_circle: |
| test_stable[isaacsim-rob#1] | host                   | cpu_pct     | 42.19 → 37.49 (-11.1%)                  | 48.08 → 37.57 (t=10-60s, -21.9%) :green_circle:    | 36.3 → 37.42 (t=70-120s, +3.1%)                        | 4.6 → 34.5 (+650.0%) :red_circle:      | 94.7 → 40.2 (-57.6%) :green_circle:         |
| test_stable[isaacsim-rob#1] | host                   | mem_mb      | 2.522e+04 → 2.486e+04 (-1.4%)           | 2.498e+04 → 2.485e+04 (t=10-60s, -0.5%)            | 2.545e+04 → 2.487e+04 (t=70-120s, -2.3%)               | 2.202e+04 → 2.481e+04 (+12.7%)         | 2.601e+04 → 2.49e+04 (-4.3%)                |
| test_stable[isaacsim-rob#1] | host                   | gpu_pct     | 19.33 → 26.92 (+39.3%) :red_circle:     | 10.17 → 27 (t=10-60s, +165.5%) :red_circle:        | 28.5 → 26.83 (t=70-120s, -5.9%)                        | 1 → 24 (+2300.0%) :red_circle:         | 30 → 31 (+3.3%)                             |
| test_stable[isaacsim-rob#1] | host                   | vram_mb     | 4624 → 5528 (+19.5%)                    | 3734 → 5530 (t=10-60s, +48.1%) :red_circle:        | 5514 → 5525 (t=70-120s, +0.2%)                         | 2302 → 5524 (+140.0%) :red_circle:     | 5519 → 5531 (+0.2%)                         |
| test_stable[isaacsim-rob#1] | host                   | gpu_temp_c  | 37.25 → 40.25 (+8.1%)                   | 35.83 → 39.83 (t=10-60s, +11.2%)                   | 38.67 → 40.67 (t=70-120s, +5.2%)                       | 35 → 39 (+11.4%)                       | 39 → 41 (+5.1%)                             |
| test_stable[isaacsim-rob#1] | host                   | gpu_power_w | 114 → 136.8 (+20.0%)                    | 94.48 → 137.3 (t=10-60s, +45.4%) :red_circle:      | 133.6 → 136.3 (t=70-120s, +2.0%)                       | 70.79 → 130.7 (+84.6%) :red_circle:    | 143.4 → 145.2 (+1.3%)                       |
| test_stable[isaacsim-rob#1] | airstack-robot-desktop | cpu_pct     | 191.2 → 325.5 (+70.2%) :red_circle:     | 108.7 → 317.9 (t=10-60s, +192.6%) :red_circle:     | 273.8 → 333.1 (t=70-120s, +21.7%) :red_circle:         | 12.12 → 268.9 (+2118.9%) :red_circle:  | 304.5 → 352.3 (+15.7%)                      |
| test_stable[isaacsim-rob#1] | airstack-robot-desktop | mem_mb      | 1044 → 1101 (+5.5%)                     | 991.7 → 1102 (t=10-60s, +11.1%)                    | 1097 → 1101 (t=70-120s, +0.4%)                         | 934 → 1093 (+17.0%)                    | 1101 → 1110 (+0.8%)                         |
| test_stable[isaacsim-rob#1] | airstack-robot-desktop | disk_io_mb  | 109.5 → 16.14 (-85.3%) :green_circle:   | 109.4 → 16.14 (t=10-60s, -85.2%) :green_circle:    | 109.6 → 16.14 (t=70-120s, -85.3%) :green_circle:       | 109.4 → 16.14 (-85.2%) :green_circle:  | 109.6 → 16.14 (-85.3%) :green_circle:       |
| test_stable[isaacsim-rob#1] | airstack-robot-desktop | net_io_mb   | 7642 → 2.011e+04 (+163.1%) :red_circle: | 1019 → 1.186e+04 (t=10-60s, +1064.7%) :red_circle: | 1.427e+04 → 2.835e+04 (t=70-120s, +98.7%) :red_circle: | 50.42 → 4720 (+9262.2%) :red_circle:   | 2.106e+04 → 3.539e+04 (+68.0%) :red_circle: |
| test_stable[isaacsim-rob#1] | airstack-gcs           | cpu_pct     | 17.92 → 22.07 (+23.2%) :red_circle:     | 14.11 → 21.94 (t=10-60s, +55.5%) :red_circle:      | 21.73 → 22.2 (t=70-120s, +2.2%)                        | 2.51 → 18.45 (+635.1%) :red_circle:    | 27.38 → 24.21 (-11.6%)                      |
| test_stable[isaacsim-rob#1] | airstack-gcs           | mem_mb      | 675.1 → 652.1 (-3.4%)                   | 680.7 → 651.3 (t=10-60s, -4.3%)                    | 669.4 → 652.8 (t=70-120s, -2.5%)                       | 668.4 → 650.6 (-2.7%)                  | 702.7 → 653.6 (-7.0%)                       |
| test_stable[isaacsim-rob#1] | airstack-gcs           | disk_io_mb  | 51.38 → 7.44 (-85.5%) :green_circle:    | 51.35 → 7.41 (t=10-60s, -85.6%) :green_circle:     | 51.42 → 7.46 (t=70-120s, -85.5%) :green_circle:        | 51.3 → 7.39 (-85.6%) :green_circle:    | 51.5 → 7.48 (-85.5%) :green_circle:         |
| test_stable[isaacsim-rob#1] | airstack-gcs           | net_io_mb   | 30.57 → 44.25 (+44.7%) :red_circle:     | 24.6 → 36.66 (t=10-60s, +49.0%) :red_circle:       | 36.53 → 51.85 (t=70-120s, +41.9%) :red_circle:         | 23.12 → 30.19 (+30.6%) :red_circle:    | 42.62 → 58.36 (+36.9%) :red_circle:         |

## system.test_sensors

### Pass rates

| Test                                             | Pass   | Fail   | Skip   | Rate (baseline → current)   |
|--------------------------------------------------|--------|--------|--------|-----------------------------|
| test_lidar_filtered_cloud_sanity[isaacsim-rob#1] | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_robot_filtered_lidar_stream[isaacsim-rob#1] | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_robot_stereo_bridge_rates[isaacsim-rob#1]   | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sensor_streams_stable[isaacsim-rob#1]       | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sentinel_nodes_present[isaacsim-rob#1]      | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sim_clock_available[isaacsim-rob#1]         | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sim_clock_realtime_factor[isaacsim-rob#1]   | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_sim_topic_publish_rates[isaacsim-rob#1]     | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |

### Metrics

| Test                                             | Metric                       | Baseline   | Current   | Change                |
|--------------------------------------------------|------------------------------|------------|-----------|-----------------------|
| test_sim_clock_available[isaacsim-rob#1]         | duration_s                   | 79.84s     | 73.75s    | -7.6%                 |
| test_sim_clock_available[isaacsim-rob#1]         | airstack_up_duration_s       | 0.55s      | 0.76s     | +38.2% :red_circle:   |
| test_sim_clock_available[isaacsim-rob#1]         | sensors_sim_ready_duration_s | 79.63s     | 73.54s    | -7.6%                 |
| test_sim_clock_available[isaacsim-rob#1]         | airstack_down_duration_s     | 11.31s     | 11.19s    | -1.1%                 |
| test_sentinel_nodes_present[isaacsim-rob#1]      | duration_s                   | 6.804s     | 6.733s    | -1.0%                 |
| test_sim_topic_publish_rates[isaacsim-rob#1]     | duration_s                   | 31.35s     | 31.23s    | -0.4%                 |
| test_robot_stereo_bridge_rates[isaacsim-rob#1]   | duration_s                   | 20.85s     | 20.86s    | +0.1%                 |
| test_robot_filtered_lidar_stream[isaacsim-rob#1] | duration_s                   | 2.681s     | 3.642s    | +35.8% :red_circle:   |
| test_sim_clock_realtime_factor[isaacsim-rob#1]   | duration_s                   | 23.12s     | 24.82s    | +7.3%                 |
| test_sim_clock_realtime_factor[isaacsim-rob#1]   | sim.realtime_factor          | 1.276      | 1.301     | +2.0%                 |
| test_lidar_filtered_cloud_sanity[isaacsim-rob#1] | duration_s                   | 6.972s     | 3.62s     | -48.1% :green_circle: |
| test_sensor_streams_stable[isaacsim-rob#1]       | duration_s                   | 478.1s     | 480.1s    | +0.4%                 |

### Sim publishing rates (n=1 iterations; baseline → current, per-topic)

| Test                                       | Topic                                                     | mean                                  | start_mean                                      | end_mean                                        | min                                    | max                   |
|--------------------------------------------|-----------------------------------------------------------|---------------------------------------|-------------------------------------------------|-------------------------------------------------|----------------------------------------|-----------------------|
| test_sensor_streams_stable[isaacsim-rob#1] | sim.clock                                                 | 39.46 → 44.2 (+12.0%)                 | 45.21 → 44.43 (t=10-60s, -1.7%)                 | 33.7 → 43.97 (t=70-120s, +30.5%) :green_circle: | 14.43 → 40.77 (+182.5%) :green_circle: | 47.87 → 46.14 (-3.6%) |
| test_sensor_streams_stable[isaacsim-rob#1] | sim.robot.sensors.front_stereo.left.image_rect            | 44.24 → 44.26 (+0.0%)                 | 44.12 → 44.64 (t=10-60s, +1.2%)                 | 44.35 → 43.88 (t=70-120s, -1.1%)                | 40.58 → 42.11 (+3.8%)                  | 45.59 → 47.71 (+4.6%) |
| test_sensor_streams_stable[isaacsim-rob#1] | sim.robot.sensors.front_stereo.right.image_rect           | 41.6 → 44.3 (+6.5%)                   | 38.95 → 44.32 (t=10-60s, +13.8%)                | 44.25 → 44.28 (t=70-120s, +0.1%)                | 19.66 → 42.55 (+116.4%) :green_circle: | 45.97 → 46.28 (+0.7%) |
| test_sensor_streams_stable[isaacsim-rob#1] | sim.robot.sensors.front_stereo.left.depth_ground_truth    | 39.98 → 43.19 (+8.0%)                 | 37.54 → 41.81 (t=10-60s, +11.4%)                | 42.41 → 44.57 (t=70-120s, +5.1%)                | 16.83 → 36.16 (+114.8%) :green_circle: | 47.09 → 47.64 (+1.2%) |
| test_sensor_streams_stable[isaacsim-rob#1] | sim.robot.sensors.front_stereo.right.depth_ground_truth   | 39.52 → 43.83 (+10.9%)                | 37.17 → 43.42 (t=10-60s, +16.8%)                | 41.87 → 44.23 (t=70-120s, +5.6%)                | 22.47 → 41.9 (+86.5%) :green_circle:   | 45.01 → 45.05 (+0.1%) |
| test_sensor_streams_stable[isaacsim-rob#1] | robot.robot.sensors.front_stereo.left.image_rect          | 42.27 → 44.28 (+4.8%)                 | 44.37 → 44.2 (t=10-60s, -0.4%)                  | 40.17 → 44.36 (t=70-120s, +10.4%)               | 24.22 → 41.94 (+73.1%) :green_circle:  | 46.12 → 47.08 (+2.1%) |
| test_sensor_streams_stable[isaacsim-rob#1] | robot.robot.sensors.front_stereo.right.image_rect         | 40.54 → 43.93 (+8.4%)                 | 43.42 → 44.24 (t=10-60s, +1.9%)                 | 37.67 → 43.62 (t=70-120s, +15.8%)               | 24.29 → 41.17 (+69.5%) :green_circle:  | 45.83 → 45.87 (+0.1%) |
| test_sensor_streams_stable[isaacsim-rob#1] | robot.robot.sensors.front_stereo.left.depth_ground_truth  | 39.41 → 41.11 (+4.3%)                 | 34.88 → 44.61 (t=10-60s, +27.9%) :green_circle: | 43.94 → 37.61 (t=70-120s, -14.4%)               | 16.07 → 24.58 (+52.9%) :green_circle:  | 45.47 → 48.27 (+6.2%) |
| test_sensor_streams_stable[isaacsim-rob#1] | robot.robot.sensors.front_stereo.right.depth_ground_truth | 36.34 → 44.18 (+21.6%) :green_circle: | 32.95 → 44.81 (t=10-60s, +36.0%) :green_circle: | 39.73 → 43.56 (t=70-120s, +9.6%)                | 16.05 → 42.23 (+163.2%) :green_circle: | 46.17 → 48.09 (+4.2%) |

### Compute usage (n=1 iterations; baseline → current, per-container and global)

| Test                                       | Entity                                 | Metric   | mean          | start_mean              | end_mean                 | min           | max           |
|--------------------------------------------|----------------------------------------|----------|---------------|-------------------------|--------------------------|---------------|---------------|
| test_sensor_streams_stable[isaacsim-rob#1] | lidar.robot.sensors.ouster.point_cloud | received | 1 → 1 (+0.0%) | 1 → 1 (t=10-60s, +0.0%) | 1 → 1 (t=70-120s, +0.0%) | 1 → 1 (+0.0%) | 1 → 1 (+0.0%) |

**Regression detected** — some metrics exceeded the threshold.