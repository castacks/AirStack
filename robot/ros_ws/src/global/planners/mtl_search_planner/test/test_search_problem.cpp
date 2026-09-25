// Copyright (c) 2026 Carnegie Mellon University
// SPDX-License-Identifier: BSD-3-Clause-Clear
//
// Unit tests for the ROS-free scenario/plan adapter (no rclcpp needed).
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "mtl_search_planner/search_problem.hpp"

namespace {

// Two agents, three tight cell blobs, NED area centred at (50, -20): exercises
// the south-west anchoring of the mtl frame and per-agent homes.
std::string tinyScenario(double budgetS = 120.0, bool singleAxis = true) {
    std::string s = R"({
  "schema": "mtl.scenario/1",
  "mission": {"name": "tiny", "seed": 7, "area": {"size_m": 200.0, "center_ned": [50.0, -20.0], "belief_res_m": 2.0}},
  "aircraft": {"altitude_m": 30.0, "speed_mps": 6.0, "min_turn_radius_m": 12.0, "dt": 0.1, "dubins_step_m": 0.5},
  "mapping": {"target_cell_size_m": 20.0, "mean_information_thresh": 0.05, "max_cluster_radius_m": 45.0,
              "kmeans_replicates": 3, "kmeans_max_iter": 100},
  "sensor": {"fov_deg": 60.0, "single_axis_gimbal": SINGLE, "tilt_deg": 30.0, "max_slant_range_m": 90.0,
             "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0, "p_out_of_range": 1e-6, "threshold": 0.9}},
  "gimbal": {},
  "team": {"max_flight_time_s": BUDGET, "max_flight_distance_m": null,
           "agents": [{"name": "robot_1", "start_ned": [-40.0, -100.0], "home_ned": [-40.0, -100.0]},
                      {"name": "robot_2", "start_ned": [-40.0, -88.0],  "home_ned": [-40.0, -88.0]}]},
  "cells": {"centers": [[60, -30], [60, -10], [80, -30], [80, -10], [120, 40], [120, 60], [0, 50], [20, 50]],
            "mass": [120, 100, 90, 80, 150, 140, 60, 70], "total_map_mass": 900},
  "solver": {}, "verbose": false, "verify_geometry": false, "verify_geometry_verbose": false
})";
    auto rep = [&s](const std::string& from, const std::string& to) {
        s.replace(s.find(from), from.size(), to);
    };
    rep("BUDGET", std::to_string(budgetS));
    rep("SINGLE", singleAxis ? "true" : "false");
    return s;
}

}  // namespace

TEST(Frame, NedMtlRoundTripAndSouthWestAnchor) {
    const auto p = mtl_search::parseScenario(tinyScenario());
    EXPECT_DOUBLE_EQ(p.frame.nMin, 50.0 - 100.0);
    EXPECT_DOUBLE_EQ(p.frame.eMin, -20.0 - 100.0);
    for (double n : {-50.0, 0.0, 33.3, 150.0}) {
        for (double e : {-120.0, -20.0, 80.0}) {
            const mtl::Vec2 xy = p.frame.toMtl(n, e);
            double n2 = 0, e2 = 0;
            p.frame.fromMtl(xy.x(), xy.y(), n2, e2);
            EXPECT_NEAR(n, n2, 1e-12);
            EXPECT_NEAR(e, e2, 1e-12);
        }
    }
    // x East, y North in mtl
    const mtl::Vec2 sw = p.frame.toMtl(p.frame.nMin, p.frame.eMin);
    EXPECT_NEAR(sw.norm(), 0.0, 1e-12);
    EXPECT_NEAR(mtl_search::Frame::yawToNed(0.0), mtl::kPi / 2.0, 1e-12);  // East
}

TEST(Scenario, ParsesTeamCellsAndParams) {
    const auto p = mtl_search::parseScenario(tinyScenario(90.0));
    ASSERT_EQ(p.agents.size(), 2u);
    EXPECT_EQ(p.agentIndex("robot_2"), 1);
    EXPECT_EQ(p.agentIndex("robot_9"), -1);
    EXPECT_EQ(p.cells.size(), 8);
    EXPECT_DOUBLE_EQ(p.params.avgDroneSpeed, 6.0);
    EXPECT_DOUBLE_EQ(p.params.maxFlightTime, 90.0);
    EXPECT_TRUE(std::isinf(p.params.maxFlightDistance));
    EXPECT_NEAR(p.params.sensorTiltAngle, mtl::deg2rad(30.0), 1e-12);
    const mtl::Vec3 home = p.agents[1].homeEnu();
    EXPECT_DOUBLE_EQ(home.x(), -88.0);  // east
    EXPECT_DOUBLE_EQ(home.y(), -40.0);  // north
}

TEST(Scenario, RejectsBadSchemaAndMissingCells) {
    EXPECT_THROW(mtl_search::parseScenario(R"({"schema": "nope"})"), std::runtime_error);
    std::string s = tinyScenario();
    s.replace(s.find("\"cells\""), 7, "\"xcells\"");
    EXPECT_THROW(mtl_search::parseScenario(s), std::runtime_error);
}

TEST(Plan, AltitudeOffsetShiftsOnlyThatAgentsTrack) {
    std::string s = tinyScenario(120.0);
    s.replace(s.find("\"home_ned\": [-40.0, -88.0]}"), 27,
              "\"home_ned\": [-40.0, -88.0], \"altitude_offset_m\": 1.0}");
    const auto p = mtl_search::parseScenario(s);
    const auto r = mtl_search::solve(p);
    const auto t0 = mtl_search::buildAgentTrack(r, p, 0);
    const auto t1 = mtl_search::buildAgentTrack(r, p, 1);
    EXPECT_NEAR(t0.samples.front().z, 30.0, 1e-6);
    EXPECT_NEAR(t0.altitude, 30.0, 1e-9);
    ASSERT_GT(t1.samples.size(), 10u);
    for (const auto& smp : t1.samples) EXPECT_NEAR(smp.z, 31.0, 1e-6);
    EXPECT_NEAR(t1.altitude, 31.0, 1e-9);
    // the boresight ground points do not move with the layer
    EXPECT_NEAR(t1.samples[t1.samples.size() / 2].bz, 0.0, 1e-9);
}

TEST(Plan, RunOutDistanceIsConfigurable) {
    std::string s = tinyScenario(120.0);
    s.replace(s.find("\"solver\": {}"), 12, "\"solver\": {\"extend_dist_m\": 24.0}");
    const auto p = mtl_search::parseScenario(s);
    EXPECT_NEAR(p.params.extension.extendDist, 24.0, 1e-12);
    EXPECT_NEAR(mtl_search::parseScenario(tinyScenario()).params.extension.extendDist,
                mtl::ExtensionParams{}.extendDist, 1e-12);
}

TEST(Plan, AgentTrackIsInMapFrameStartsAtHomeAndRespectsBudget) {
    const auto p = mtl_search::parseScenario(tinyScenario(120.0));
    const auto r = mtl_search::solve(p);
    for (int a = 0; a < 2; ++a) {
        const auto tr = mtl_search::buildAgentTrack(r, p, a);
        ASSERT_GT(tr.samples.size(), 10u);
        // starts where the robot spawned: map origin
        EXPECT_NEAR(tr.samples.front().x, 0.0, 1e-6);
        EXPECT_NEAR(tr.samples.front().y, 0.0, 1e-6);
        EXPECT_NEAR(tr.samples.front().z, 30.0, 1e-6);
        // arc length is monotone and within the endurance budget
        for (std::size_t k = 1; k < tr.samples.size(); ++k) {
            EXPECT_GE(tr.samples[k].arc, tr.samples[k - 1].arc);
        }
        EXPECT_LE(tr.totalArc(), tr.budget * 1.01 + 1.0);
        // boresight is on the ground plane (map z = -homeUp = 0)
        EXPECT_NEAR(tr.samples[tr.samples.size() / 2].bz, 0.0, 1e-9);
        // the tail padding was trimmed: last sample still moves
        const auto& l = tr.samples.back();
        const auto& m = tr.samples[tr.samples.size() - 2];
        EXPECT_GT(std::hypot(l.x - m.x, l.y - m.y), 0.05);
    }
}

TEST(Plan, MapFrameIsWorldMinusHome) {
    const auto p = mtl_search::parseScenario(tinyScenario(120.0));
    const auto r = mtl_search::solve(p);
    const auto tr = mtl_search::buildAgentTrack(r, p, 1);
    const std::size_t k = tr.samples.size() / 3;
    double n = 0, e = 0;
    p.frame.fromMtl(r.trajectories[1].drone(static_cast<mtl::Index>(k), 0),
                    r.trajectories[1].drone(static_cast<mtl::Index>(k), 1), n, e);
    EXPECT_NEAR(tr.samples[k].x, e - (-88.0), 1e-9);
    EXPECT_NEAR(tr.samples[k].y, n - (-40.0), 1e-9);
}

TEST(Plan, DeterministicAcrossSolves) {
    // Every robot container solves the same team problem independently; they
    // must agree bit for bit or the team would fly inconsistent plans.
    const auto p = mtl_search::parseScenario(tinyScenario(120.0));
    const auto r1 = mtl_search::solve(p);
    const auto r2 = mtl_search::solve(mtl_search::parseScenario(tinyScenario(120.0)));
    EXPECT_EQ(mtl_search::teamPlanJson(r1, p), mtl_search::teamPlanJson(r2, p));
}

TEST(Plan, MultiAxisModeHasNoSchedule) {
    const auto p = mtl_search::parseScenario(tinyScenario(200.0, /*singleAxis=*/false));
    const auto r = mtl_search::solve(p);
    const auto tr = mtl_search::buildAgentTrack(r, p, 0);
    EXPECT_FALSE(tr.singleAxis);
    EXPECT_DOUBLE_EQ(tr.tilt, 0.0);
    for (const auto& s : tr.samples) EXPECT_DOUBLE_EQ(s.gimbalPhi, 0.0);
}

TEST(Plan, JsonOutputsParse) {
    const auto p = mtl_search::parseScenario(tinyScenario());
    const auto r = mtl_search::solve(p);
    const auto team = jsonmini::parse(mtl_search::teamPlanJson(r, p));
    EXPECT_EQ(team["schema"].text(""), "mtl.plan/1");
    EXPECT_EQ(team["agents"].array().size(), 2u);
    const auto tr = mtl_search::buildAgentTrack(r, p, 0);
    const auto one = jsonmini::parse(mtl_search::agentTrackJson(tr, p));
    EXPECT_EQ(one["agent"].text(""), "robot_1");
    EXPECT_EQ(one["samples"]["x_map"].array().size(), tr.samples.size());
}
