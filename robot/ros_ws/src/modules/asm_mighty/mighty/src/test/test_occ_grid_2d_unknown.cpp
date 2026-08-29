// Tests for the tristate `OccGrid2D::fromOccupancyGrid` round-trip and the
// `unknownData()` accessor used by frontier detection.

#include <gtest/gtest.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "mighty/occ_grid_2d.hpp"

namespace {

nav_msgs::msg::OccupancyGrid MakeMsg(int w, int h, double res,
                                     const std::vector<int8_t>& data) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = static_cast<float>(res);
  msg.info.width  = w;
  msg.info.height = h;
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data = data;
  return msg;
}

}  // namespace

TEST(OccGrid2DUnknownTest, RoundTripFromOccupancyGrid) {
  // 3x3 grid with all three states.
  // Layout (ix, iy):
  //   (0,0)=-1   (1,0)=0    (2,0)=100
  //   (0,1)=0    (1,1)=-1   (2,1)=0
  //   (0,2)=100  (1,2)=0    (2,2)=-1
  std::vector<int8_t> data = {
      -1, 0, 100,
      0, -1, 0,
      100, 0, -1,
  };
  auto msg = MakeMsg(3, 3, 0.1, data);
  auto grid = OccGrid2D::fromOccupancyGrid(msg);
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(grid->width(), 3);
  EXPECT_EQ(grid->height(), 3);

  for (int iy = 0; iy < 3; ++iy) {
    for (int ix = 0; ix < 3; ++ix) {
      const int8_t expected = data[iy * 3 + ix];
      const bool exp_unknown = (expected < 0);
      const bool exp_occ     = (expected >= 100);
      const bool exp_free    = (!exp_unknown && !exp_occ);
      EXPECT_EQ(grid->isUnknown(ix, iy), exp_unknown)
          << "(" << ix << "," << iy << ") expected=" << int(expected);
      EXPECT_EQ(grid->isOccupied(ix, iy), exp_occ)
          << "(" << ix << "," << iy << ") expected=" << int(expected);
      EXPECT_EQ(grid->isFree(ix, iy), exp_free)
          << "(" << ix << "," << iy << ") expected=" << int(expected);
    }
  }
}

TEST(OccGrid2DUnknownTest, OutOfBoundsIsUnknownAndOccupied) {
  // OOB is conceptually unknown for the frontier detector (the mapper window
  // doesn't cover it). For HGP A*, OOB is "occupied" so the planner refuses
  // to step outside the map. Both behaviors are required for the same grid.
  std::vector<int8_t> data(9, 0);  // all free
  auto grid = OccGrid2D::fromOccupancyGrid(MakeMsg(3, 3, 0.1, data));
  EXPECT_TRUE(grid->isUnknown(-1, 0));
  EXPECT_TRUE(grid->isUnknown(3, 0));
  EXPECT_TRUE(grid->isOccupied(-1, 0));
  EXPECT_FALSE(grid->isFree(-1, 0));
}

TEST(OccGrid2DUnknownTest, ComputeDistanceFieldOnlyFromOccupied) {
  // Single occupied cell at (1,1), rest unknown. Distance field should treat
  // unknown as not-an-obstacle: dist[1,1]=0; everything else has positive
  // distance (or saturates at max_dist_m).
  std::vector<int8_t> data = {
      -1, -1, -1,
      -1, 100, -1,
      -1, -1, -1,
  };
  auto grid = OccGrid2D::fromOccupancyGrid(MakeMsg(3, 3, 0.1, data));
  auto dist = grid->computeDistanceField(1.0);
  ASSERT_EQ(dist.size(), 9u);
  EXPECT_FLOAT_EQ(dist[1 * 3 + 1], 0.0f);
  // Adjacent cell (0,1) should be at distance ~0.1m (one cell @ 0.1m res).
  EXPECT_NEAR(dist[1 * 3 + 0], 0.1f, 1e-4);
  // Diagonal cell (0,0) should be at ~0.1414m.
  EXPECT_NEAR(dist[0 * 3 + 0], 0.1414f, 1e-3);
}

TEST(OccGrid2DUnknownTest, FromTristateMatchesFromOccupancyGrid) {
  std::vector<int8_t> data = {
      -1, 0, 100,
      0, -1, 0,
  };
  auto from_msg = OccGrid2D::fromOccupancyGrid(MakeMsg(3, 2, 0.1, data));
  auto from_raw = OccGrid2D::fromTristate(3, 2, 0.1, 0.0, 0.0, data);
  for (int iy = 0; iy < 2; ++iy) {
    for (int ix = 0; ix < 3; ++ix) {
      EXPECT_EQ(from_msg->isUnknown(ix, iy), from_raw->isUnknown(ix, iy));
      EXPECT_EQ(from_msg->isOccupied(ix, iy), from_raw->isOccupied(ix, iy));
      EXPECT_EQ(from_msg->isFree(ix, iy), from_raw->isFree(ix, iy));
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
