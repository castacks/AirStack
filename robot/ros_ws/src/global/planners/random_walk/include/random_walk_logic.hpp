// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <vector>

typedef std::vector<std::tuple<float, float, float, float>> Path;  // x, y, z, yaw

struct init_params {
    float max_start_to_goal_dist_m;
    int checking_point_cnt;
    float max_z_change_m;
    float collision_padding_m;
    float path_end_threshold_m;
    float max_yaw_change_degrees;
    std::tuple<float, float, float> voxel_size_m;
};

class RandomWalkPlanner {
   public:
    RandomWalkPlanner() = default;
    RandomWalkPlanner(init_params params);
    ~RandomWalkPlanner() = default;

    std::optional<Path> generate_straight_rand_path(
        std::tuple<float, float, float, float> start_point,
        float timeout_duration);  // x, y, z, yaw

    // Optional XY footprint that bounds where the planner is allowed to
    // generate goal points. Empty → unbounded (preserves prior behavior).
    // Set with set_search_bounds; coordinates are in the same frame as
    // voxel_points and odometry (the robot's local "map", rooted at takeoff).
    void set_search_bounds(const std::vector<std::pair<float, float>>& xy_polygon);
    void clear_search_bounds();
    bool has_search_bounds() const;
    bool is_inside_search_bounds(float x, float y) const;
    // Returns the closest point on the polygon boundary, then nudged a small
    // distance toward the polygon centroid so the result is firmly inside
    // (avoids edge-case ambiguity with the inside test). If no bounds set
    // returns (x, y) unchanged.
    std::pair<float, float> nearest_inside_point(float x, float y) const;

    float path_end_threshold_m;

    std::vector<std::tuple<float, float, float>> voxel_points;

   private:
    bool point_in_search_bounds(float x, float y) const;
    std::vector<std::pair<float, float>> search_bounds_xy_;

    // Numerical constants
    float max_start_to_goal_dist_m_;
    int checking_point_cnt;
    float max_z_change_m_;
    float collision_padding_m;
    float max_yaw_change_degrees;

    // Variables
    std::tuple<float, float, float> voxel_size_m;

    mutable std::mutex mutex;

    // Functions
    bool check_if_collided_single_voxel(const std::tuple<float, float, float>& point,
                                        const std::tuple<float, float, float>& voxel_center);

    bool check_if_collided(const std::tuple<float, float, float>& point);

    std::tuple<float, float, float> generate_goal_point(
        std::tuple<float, float, float, float> start_point);
};

double get_point_distance(const std::tuple<float, float, float>& point1,
                          const std::tuple<float, float, float>& point2);

double deg2rad(double deg);

double rad2deg(double rad);
