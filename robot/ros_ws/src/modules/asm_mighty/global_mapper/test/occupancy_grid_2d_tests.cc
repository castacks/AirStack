// Tests for the tristate (UNKNOWN/FREE/OCCUPIED) OccupancyGrid2D and the
// marked_/cleared_ diff semantics that DistanceGrid2D depends on.

#include "gtest/gtest.h"
#include "occupancy_grid_2d/occupancy_grid_2d.h"

using occupancy_grid_2d::OccupancyGrid2D;
using occupancy_grid_2d::kFree;
using occupancy_grid_2d::kOccupied;
using occupancy_grid_2d::kUnknown;

namespace {

OccupancyGrid2D MakeGrid() {
  // 10m x 10m grid centered at origin, 0.1m resolution -> 100 x 100 cells.
  const double origin[2] = {0.0, 0.0};
  const double dims[2] = {10.0, 10.0};
  return OccupancyGrid2D(origin, dims, 0.1);
}

}  // namespace

TEST(OccupancyGrid2DTest, DefaultIsAllUnknown) {
  OccupancyGrid2D grid = MakeGrid();
  int dims[2];
  grid.GetGridDimensions(dims);
  for (int iy = 0; iy < dims[1]; iy++) {
    for (int ix = 0; ix < dims[0]; ix++) {
      ASSERT_TRUE(grid.IsUnknown(ix, iy)) << "cell (" << ix << "," << iy << ")";
      ASSERT_FALSE(grid.IsFree(ix, iy));
      ASSERT_FALSE(grid.IsOccupied(ix, iy));
      ASSERT_EQ(grid.GetCell(ix, iy), kUnknown);
    }
  }
  EXPECT_EQ(grid.GetMarked().size(), 0u);
  EXPECT_EQ(grid.GetCleared().size(), 0u);
}

TEST(OccupancyGrid2DTest, SetOccupiedPushesMarked) {
  OccupancyGrid2D grid = MakeGrid();
  grid.SetOccupied(1.0, 2.0);

  int ix, iy;
  grid.WorldToGrid(1.0, 2.0, ix, iy);
  EXPECT_TRUE(grid.IsOccupied(ix, iy));
  EXPECT_FALSE(grid.IsFree(ix, iy));
  EXPECT_FALSE(grid.IsUnknown(ix, iy));
  EXPECT_EQ(grid.GetCell(ix, iy), kOccupied);
  EXPECT_EQ(grid.GetMarked().size(), 1u);
  EXPECT_EQ(grid.GetCleared().size(), 0u);

  // Idempotent
  grid.SetOccupied(1.0, 2.0);
  EXPECT_EQ(grid.GetMarked().size(), 1u);
}

TEST(OccupancyGrid2DTest, SetFreeAfterOccupiedPushesCleared) {
  OccupancyGrid2D grid = MakeGrid();
  grid.SetOccupied(1.0, 2.0);
  grid.SetFree(1.0, 2.0);

  int ix, iy;
  grid.WorldToGrid(1.0, 2.0, ix, iy);
  EXPECT_TRUE(grid.IsFree(ix, iy));
  EXPECT_FALSE(grid.IsOccupied(ix, iy));
  EXPECT_EQ(grid.GetMarked().size(), 1u);
  EXPECT_EQ(grid.GetCleared().size(), 1u);
}

TEST(OccupancyGrid2DTest, SetUnknownAfterOccupiedPushesCleared) {
  OccupancyGrid2D grid = MakeGrid();
  grid.SetOccupied(1.0, 2.0);
  grid.SetUnknown(1.0, 2.0);

  int ix, iy;
  grid.WorldToGrid(1.0, 2.0, ix, iy);
  EXPECT_TRUE(grid.IsUnknown(ix, iy));
  EXPECT_FALSE(grid.IsOccupied(ix, iy));
  EXPECT_EQ(grid.GetMarked().size(), 1u);
  EXPECT_EQ(grid.GetCleared().size(), 1u);
}

TEST(OccupancyGrid2DTest, SetUnknownAfterFreeIsSilent) {
  // FREE <-> UNKNOWN transitions are invisible to the ESDF pipeline.
  OccupancyGrid2D grid = MakeGrid();
  grid.SetFree(1.0, 2.0);
  EXPECT_EQ(grid.GetMarked().size(), 0u);
  EXPECT_EQ(grid.GetCleared().size(), 0u);
  grid.SetUnknown(1.0, 2.0);
  EXPECT_EQ(grid.GetMarked().size(), 0u);
  EXPECT_EQ(grid.GetCleared().size(), 0u);
}

TEST(OccupancyGrid2DTest, SetFreeAfterUnknownIsSilent) {
  OccupancyGrid2D grid = MakeGrid();
  // Default is unknown.
  grid.SetFree(1.0, 2.0);
  EXPECT_EQ(grid.GetMarked().size(), 0u);
  EXPECT_EQ(grid.GetCleared().size(), 0u);

  int ix, iy;
  grid.WorldToGrid(1.0, 2.0, ix, iy);
  EXPECT_TRUE(grid.IsFree(ix, iy));
}

TEST(OccupancyGrid2DTest, UpdateOriginShiftWipesOccupiedToUnknownAndPushesCleared) {
  // Place an occupied cell, then shift origin enough to push that cell out of
  // the window. The cell should be wiped to UNKNOWN and pushed to cleared_ so
  // the ESDF can drop the obstacle.
  OccupancyGrid2D grid = MakeGrid();
  grid.SetOccupied(0.5, 0.5);
  grid.ResetDiffs();

  int ix, iy;
  grid.WorldToGrid(0.5, 0.5, ix, iy);
  ASSERT_TRUE(grid.IsOccupied(ix, iy));

  // Shift the origin far enough so the original cell is out of the window.
  // Window is 10x10, so move by 12m in x to definitely evict it.
  grid.UpdateOrigin(12.0, 0.0);

  // The world location (0.5, 0.5) is now out of the window — the underlying
  // cell that previously held it has been wiped.
  // We can't query the same world coord (it's now far from the window center).
  // Instead, verify that the slice cleanup pushed something to cleared_.
  EXPECT_GT(grid.GetCleared().size(), 0u)
      << "expected at least one cleared diff after wiping an occupied cell";
}

TEST(OccupancyGrid2DTest, UpdateOriginShiftWipesFreeSilently) {
  OccupancyGrid2D grid = MakeGrid();
  grid.SetFree(0.5, 0.5);
  grid.ResetDiffs();

  grid.UpdateOrigin(12.0, 0.0);

  // FREE cells leaving the window are wiped to UNKNOWN with no ESDF event.
  EXPECT_EQ(grid.GetCleared().size(), 0u);
  EXPECT_EQ(grid.GetMarked().size(), 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
