#include <gtest/gtest.h>
#include "../include/random_walk_logic.hpp"

namespace {
init_params parameters() {
    return {3.f, 100, 2.f, .3f, .5f, 180.f, {.1f, .1f, .1f}};
}
}

TEST(AltitudeBounds, RejectsInvalidEnvelope) {
    RandomWalkPlanner planner(parameters());
    EXPECT_THROW(planner.set_altitude_bounds(-1.f, 3.f), std::invalid_argument);
    EXPECT_THROW(planner.set_altitude_bounds(3.f, 1.f), std::invalid_argument);
    EXPECT_THROW(planner.set_altitude_bounds(NAN, 3.f), std::invalid_argument);
    EXPECT_THROW(planner.set_altitude_bounds(1.f, INFINITY), std::invalid_argument);
}

TEST(AltitudeBounds, EveryChainedSegmentRespectsEnvelope) {
    RandomWalkPlanner planner(parameters());
    planner.set_altitude_bounds(1.f, 3.f);
    std::tuple<float, float, float, float> start(0.f, 0.f, 1.5f, 0.f);
    for (int segment = 0; segment < 30; ++segment) {
        auto path = planner.generate_straight_rand_path(start, 1.f);
        ASSERT_TRUE(path.has_value());
        ASSERT_GT(path->size(), 1u);
        for (const auto& point : *path) {
            EXPECT_GE(std::get<2>(point), 1.f);
            EXPECT_LE(std::get<2>(point), 3.f);
        }
        start = path->back();
    }
}

TEST(AltitudeBounds, ToleratedStartBelowMinimumHasBoundedTransition) {
    RandomWalkPlanner planner(parameters());
    planner.set_altitude_bounds(1.f, 3.f);
    for (int trial = 0; trial < 10; ++trial) {
        auto path = planner.generate_straight_rand_path({0.f, 0.f, .95f, 0.f}, 1.f);
        ASSERT_TRUE(path.has_value());
        for (const auto& point : *path) {
            EXPECT_GE(std::get<2>(point), .95f);
            EXPECT_LE(std::get<2>(point), 3.f);
        }
        EXPECT_GE(std::get<2>(path->back()), 1.f - 1e-5f);
    }
}

TEST(AltitudeBounds, LaterTaskReplacesEarlierEnvelope) {
    RandomWalkPlanner planner(parameters());
    planner.set_altitude_bounds(1.f, 3.f);
    planner.set_altitude_bounds(2.f, 2.f);
    auto path = planner.generate_straight_rand_path({0.f, 0.f, 2.f, 0.f}, 1.f);
    ASSERT_TRUE(path.has_value());
    for (const auto& point : *path) EXPECT_FLOAT_EQ(std::get<2>(point), 2.f);
}
