## system.test_takeoff_hover_land

### Pass rates

| Test                                | Pass   | Fail   | Skip   | Rate (baseline → current)   |
|-------------------------------------|--------|--------|--------|-----------------------------|
| test_hover[isaacsim-rob#1-v1.0]     | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_hover[isaacsim-rob#3-v1.0]     | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_landing[isaacsim-rob#1-v1.0]   | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_landing[isaacsim-rob#3-v1.0]   | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_px4_ready[isaacsim-rob#1-v1.0] | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_px4_ready[isaacsim-rob#3-v1.0] | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_takeoff[isaacsim-rob#1-v1.0]   | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |
| test_takeoff[isaacsim-rob#3-v1.0]   | 1 → 1  | 0 → 0  | 0 → 0  | 100% → 100%                 |

### Metrics

| Test                                | Metric                            | Baseline   | Current   | Change                 |
|-------------------------------------|-----------------------------------|------------|-----------|------------------------|
| test_px4_ready[isaacsim-rob#1-v1.0] | duration_s                        | 90.92s     | 89.53s    | -1.5%                  |
| test_px4_ready[isaacsim-rob#1-v1.0] | airstack_up_duration_s            | 0.56s      | 0.7s      | +25.0% :red_circle:    |
| test_px4_ready[isaacsim-rob#1-v1.0] | airstack_down_duration_s          | 11.19s     | 11.14s    | -0.4%                  |
| test_px4_ready[isaacsim-rob#1-v1.0] | robot.ready_duration_sys_s        | 90.13s     | 88.59s    | -1.7%                  |
| test_takeoff[isaacsim-rob#1-v1.0]   | duration_s                        | 13.82s     | 15.96s    | +15.5%                 |
| test_takeoff[isaacsim-rob#1-v1.0]   | robot.altitude_error_m            | -0.151m    | -0.164m   | +8.6%                  |
| test_takeoff[isaacsim-rob#1-v1.0]   | robot.overshoot_m                 | 0m         | 0m        | +0.0%                  |
| test_takeoff[isaacsim-rob#1-v1.0]   | robot.takeoff_duration_sim_s      | 10.23s     | 10.17s    | -0.6%                  |
| test_takeoff[isaacsim-rob#1-v1.0]   | robot.velocity_rmse_m_sim_s       | 0.227m/s   | 0.228m/s  | +0.4%                  |
| test_hover[isaacsim-rob#1-v1.0]     | duration_s                        | 10.04s     | 10.03s    | -0.1%                  |
| test_hover[isaacsim-rob#1-v1.0]     | robot.hover_altitude_mean_error_m | 0.036m     | 0.021m    | -41.7% :green_circle:  |
| test_hover[isaacsim-rob#1-v1.0]     | robot.hover_position_stddev_m     | 0.075m     | 0.061m    | -18.7%                 |
| test_landing[isaacsim-rob#1-v1.0]   | duration_s                        | 13.93s     | 14.96s    | +7.4%                  |
| test_landing[isaacsim-rob#1-v1.0]   | robot.final_altitude_m            | 0.052m     | -0.001m   | -101.9% :green_circle: |
| test_landing[isaacsim-rob#1-v1.0]   | robot.land_duration_sim_s         | 6s         | 7.41s     | +23.5% :red_circle:    |
| test_landing[isaacsim-rob#1-v1.0]   | robot.velocity_rmse_m_sim_s       | 0.189m/s   | 0.189m/s  | +0.0%                  |
| test_px4_ready[isaacsim-rob#3-v1.0] | duration_s                        | 114.3s     | 114.7s    | +0.3%                  |
| test_px4_ready[isaacsim-rob#3-v1.0] | airstack_up_duration_s            | 1.06s      | 1.17s     | +10.4%                 |
| test_px4_ready[isaacsim-rob#3-v1.0] | airstack_down_duration_s          | 11.24s     | 11.25s    | +0.1%                  |
| test_px4_ready[isaacsim-rob#3-v1.0] | robot.ready_duration_sys_s        | 91.96s     | 92.94s    | +1.1%                  |
| test_takeoff[isaacsim-rob#3-v1.0]   | duration_s                        | 29.07s     | 27.61s    | -5.0%                  |
| test_takeoff[isaacsim-rob#3-v1.0]   | robot.altitude_error_m            | -0.18m     | -0.172m   | -4.4%                  |
| test_takeoff[isaacsim-rob#3-v1.0]   | robot.overshoot_m                 | 0m         | 0m        | +0.0%                  |
| test_takeoff[isaacsim-rob#3-v1.0]   | robot.takeoff_duration_sim_s      | 10.17s     | 10.17s    | +0.0%                  |
| test_takeoff[isaacsim-rob#3-v1.0]   | robot.velocity_rmse_m_sim_s       | 0.229m/s   | 0.226m/s  | -1.3%                  |
| test_hover[isaacsim-rob#3-v1.0]     | duration_s                        | 10.04s     | 10.05s    | +0.1%                  |
| test_hover[isaacsim-rob#3-v1.0]     | robot.hover_altitude_mean_error_m | 0.021m     | 0.021m    | +0.0%                  |
| test_hover[isaacsim-rob#3-v1.0]     | robot.hover_position_stddev_m     | 0.045m     | 0.049m    | +8.9%                  |
| test_landing[isaacsim-rob#3-v1.0]   | duration_s                        | 45.12s     | 45.2s     | +0.2%                  |
| test_landing[isaacsim-rob#3-v1.0]   | robot.final_altitude_m            | 0.015m     | -0.014m   | -193.3% :green_circle: |
| test_landing[isaacsim-rob#3-v1.0]   | robot.land_duration_sim_s         | 7.41s      | 7.41s     | +0.0%                  |
| test_landing[isaacsim-rob#3-v1.0]   | robot.velocity_rmse_m_sim_s       | 0.189m/s   | 0.19m/s   | +0.5%                  |

**Regression detected** — some metrics exceeded the threshold.