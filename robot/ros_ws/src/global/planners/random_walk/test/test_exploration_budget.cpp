#include <gtest/gtest.h>
#include "../include/exploration_budget.hpp"

TEST(ExplorationBudget, PreparationIsWallBoundedWithoutConsumingActiveTime) {
    ExplorationBudget budget(100.0, 4.0);
    EXPECT_DOUBLE_EQ(budget.remaining(107.0, 30'000'000'000), 3.0);
    EXPECT_DOUBLE_EQ(budget.elapsed(30'000'000'000), 0.0);
    EXPECT_LE(budget.remaining(110.0, 30'000'000'000), 0.0);
}

TEST(ExplorationBudget, AcceptedNavigationStartsExactSimulatorIntervalOnlyOnce) {
    ExplorationBudget budget(100.0, 4.0);
    budget.accepted(30'000'000'000, 107.0);
    EXPECT_DOUBLE_EQ(budget.remaining(200.0, 31'000'000'000), 3.0);
    budget.accepted(33'000'000'000, 108.0);  // replanning cannot reset the timer
    EXPECT_EQ(budget.first_accept_ns(), 30'000'000'000);
    EXPECT_DOUBLE_EQ(budget.elapsed(34'000'000'000), 4.0);
    EXPECT_LE(budget.remaining(201.0, 34'000'000'000), 0.0);
}

TEST(ExplorationBudget, RollbackInvalidLimitsAndMissingAdmissionFailClosed) {
    ExplorationBudget budget(100.0, 4.0);
    budget.accepted(0, 101.0);
    EXPECT_EQ(budget.first_accept_ns(), 0);
    budget.accepted(30'000'000'000, 101.0);
    EXPECT_LE(budget.remaining(101.0, 29'000'000'000), 0.0);
    EXPECT_LE(budget.remaining(NAN, 31'000'000'000), 0.0);
    ExplorationBudget invalid(100.0, NAN);
    EXPECT_LE(invalid.remaining(101.0, 1), 0.0);
}

TEST(ExplorationBudget, LateAcceptanceCannotReviveExpiredPreparation) {
    ExplorationBudget budget(100.0, 4.0);
    budget.accepted(30'000'000'000, 111.0);
    EXPECT_EQ(budget.first_accept_ns(), 0);
    EXPECT_LE(budget.remaining(111.0, 30'000'000'000), 0.0);
}

TEST(ExplorationBudget, RollbackWithinActiveIntervalIsLatched) {
    ExplorationBudget budget(100.0, 4.0);
    budget.accepted(30'000'000'000, 101.0);
    EXPECT_DOUBLE_EQ(budget.remaining(102.0, 33'000'000'000), 1.0);
    EXPECT_LE(budget.remaining(103.0, 32'000'000'000), 0.0);
    EXPECT_TRUE(budget.clock_rolled_back());
    EXPECT_LE(budget.remaining(104.0, 33'000'000'000), 0.0);
}
