#include <chrono>
#include <algorithm>
#include "gtest/gtest.h"
#include "voxel_grid/voxel_grid.h"

/******************************************************************************/
//                              GridToIndex
/******************************************************************************/
// (ixyz) [ind]
// | (0, 2, 0) [5] | (1, 2, 0) [3] | (2, 2, 0) [4] |
// | (0, 1, 0) [2] | (1, 1, 0) [0] | (2, 1, 0) [1] |
// | (0, 0, 0) [8] | (1, 0, 0) [6] | (2, 0, 0) [7] |

void SimpleGridToIndexTest(voxel_grid::VoxelGrid<int>& grid) {
  int ixyz[3];

  ixyz[0] = 2;
  ixyz[1] = 2;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 4);

  ixyz[0] = 0;
  ixyz[1] = 2;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 5);

  ixyz[0] = 1;
  ixyz[1] = 2;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 3);

  ixyz[0] = 2;
  ixyz[1] = 0;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 7);

  ixyz[0] = 0;
  ixyz[1] = 0;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 8);

  ixyz[0] = 1;
  ixyz[1] = 0;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 6);
  
  ixyz[0] = 2;
  ixyz[1] = 1;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 1);

  ixyz[0] = 0;
  ixyz[1] = 1;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 2);

  ixyz[0] = 1;
  ixyz[1] = 1;
  ixyz[2] = 0;
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);
}

TEST(VoxelGrid, GridToIndexStatic) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);

  SimpleGridToIndexTest(grid);
}

TEST(VoxelGrid, GridToIndexSubVoxelShiftX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3];

  // just inside negative border
  origin[0] = 0;
  grid.UpdateOrigin(origin);
  SimpleGridToIndexTest(grid);
  
  // just inside positive border
  origin[0] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  SimpleGridToIndexTest(grid);
}

TEST(VoxelGrid, GridToIndexVoxelShiftX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {1, 1, 0};

  // just inside of voxel 2 negative border
  origin[0] = -resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 2);

  // just inside of voxel 2 positive border
  origin[0] = -1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 2);

  // no shift at origin
  origin[0] = 0;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // ignoring origin shift
  origin[0] = 0.5 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // ignoring origin shift
  origin[0] = 0.5 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // not quite enough to move grid over
  origin[0] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // just enough to move grid over
  origin[0] = resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 1);

  // just inside of voxel 1 positive border
  origin[0] = 2 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 1);
}

TEST(VoxelGrid, GridToIndexVoxelWrapX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {1, 1, 0};

  // just inside of voxel 1 negative border after wrapping
  origin[0] = -2.0 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 1);

  // just inside of voxel 1 positive border after wrapping
  origin[0] = -resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 1);

  // just inside of voxel 2 negative border after wrapping
  origin[0] = 2.0 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 2);

  // just inside of voxel 2 positive border after wrapping
  origin[0] = 3.0 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 2);
}

TEST(VoxelGrid, GridToIndexSubVoxelShiftY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3];

  // just inside of negative border
  origin[1] = 0;
  grid.UpdateOrigin(origin);
  SimpleGridToIndexTest(grid);
  
  // just inside positive border
  origin[1] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  SimpleGridToIndexTest(grid);
}

TEST(VoxelGrid, GridToIndexVoxelShiftY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {1, 1, 0};

  // just inside of voxel 6 negative border
  origin[1] = -resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 6);

  // just inside of voxel 6 positive border
  origin[1] = -1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 6);

  // no shift at origin
  origin[1] = 0;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // ignoring origin shift
  origin[1] = 0.5 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // ignoring origin shift
  origin[1] = 0.5 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // not quite enough to move grid over
  origin[1] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 0);

  // just inside of voxel 5 negative border
  origin[1] = resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 3);

  // just inside of voxel 2 positive border
  origin[1] = 2 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 3);
}

TEST(VoxelGrid, GridToIndexVoxelWrapY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {1, 1, 0};

  // just inside of voxel 3 negative border after wrapping
  origin[1] = -2.0 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 3);

  // just inside of voxel 3 positive border after wrapping
  origin[1] = -resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 3);

  // just inside of voxel 6 negative border after wrapping
  origin[1] = 2.0 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 6);

  // just inside of voxel 6 positive border after wrapping
  origin[1] = 3.0 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  EXPECT_EQ(grid.GridToIndex(ixyz), 6);
}
 
TEST(VoxelGrid, GridToIndexSubVoxelShiftZ) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3];

  // just inside of negative border
  origin[2] = 0;
  grid.UpdateOrigin(origin);
  SimpleGridToIndexTest(grid);
  
  // just inside of positive border
  origin[2] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  SimpleGridToIndexTest(grid);
}

/******************************************************************************/
//                              IndexToGrid
/******************************************************************************/
// (ixyz) [ind]
// | (0, 2, 0) [5] | (1, 2, 0) [3] | (2, 2, 0) [4] |
// | (0, 1, 0) [2] | (1, 1, 0) [0] | (2, 1, 0) [1] |
// | (0, 0, 0) [8] | (1, 0, 0) [6] | (2, 0, 0) [7] |

void SimpleIndexToGridTest(voxel_grid::VoxelGrid<int>& grid) {
  int ixyz[3];

  grid.IndexToGrid(0, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  grid.IndexToGrid(1, ixyz);
  EXPECT_EQ(ixyz[0], 2);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  grid.IndexToGrid(2, ixyz);
  EXPECT_EQ(ixyz[0], 0);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  grid.IndexToGrid(3, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 2);
  EXPECT_EQ(ixyz[2], 0);

  grid.IndexToGrid(4, ixyz);
  EXPECT_EQ(ixyz[0], 2);
  EXPECT_EQ(ixyz[1], 2);
  EXPECT_EQ(ixyz[2], 0);

  grid.IndexToGrid(5, ixyz);
  EXPECT_EQ(ixyz[0], 0);
  EXPECT_EQ(ixyz[1], 2);
  EXPECT_EQ(ixyz[2], 0);

  grid.IndexToGrid(6, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 0);
  EXPECT_EQ(ixyz[2], 0);

  grid.IndexToGrid(7, ixyz);
  EXPECT_EQ(ixyz[0], 2);
  EXPECT_EQ(ixyz[1], 0);
  EXPECT_EQ(ixyz[2], 0);
}

TEST(VoxelGrid, IndexToGridStatic) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3];

  SimpleIndexToGridTest(grid);
}

TEST(VoxelGrid, IndexToGridSubVoxelShiftX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3];

  // just inside of negative border
  origin[0] = 0;
  grid.UpdateOrigin(origin);
  SimpleIndexToGridTest(grid);

  // just inside of positive border
  origin[0] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  SimpleIndexToGridTest(grid);
}

TEST(VoxelGrid, IndexToGridVoxelShiftX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {0, 0, 0};

  // just inside of voxel 2 negative border
  origin[0] = -resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(2, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 2 positive border
  origin[0] = -1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(2, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 1 negative border
  origin[0] = resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(1, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 1 negative border
  origin[0] = 2.0 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(1, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

TEST(VoxelGrid, IndexToGridVoxelWrapX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {0, 0, 0};

  // just inside of voxel 1 negative border after wrapping
  origin[0] = -2.0 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(1, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 1 positive border after wrapping
  origin[0] = -resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(1, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 2 negative border after wrapping
  origin[0] = 2 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(2, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 2 positive border after wrapping
  origin[0] = 3 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(2, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

TEST(VoxelGrid, IndexToGridSubVoxelShiftY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3];

  // just inside of negative border
  origin[1] = 0;
  grid.UpdateOrigin(origin);
  SimpleIndexToGridTest(grid);

  // just inside of positive border
  origin[1] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  SimpleIndexToGridTest(grid);
}

TEST(VoxelGrid, IndexToGridVoxelShiftY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {0, 0, 0};

  // just inside of voxel 6 negative border
  origin[1] = -resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(6, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 6 positive border
  origin[1] = -1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(6, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 3 negative border
  origin[1] = resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(3, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 3 negative border
  origin[1] = 2.0 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(3, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

TEST(VoxelGrid, IndexToGridVoxelWrapY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  int ixyz[3] = {0, 0, 0};

  // just inside of voxel 3 negative border after wrapping
  origin[1] = -2.0 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(3, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 3 positive border after wrapping
  origin[1] = -resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(3, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 6 negative border after wrapping
  origin[1] = 2 * resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(6, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 6 positive border after wrapping
  origin[1] = 3 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(6, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

/******************************************************************************/
//                              WorldToGrid
/******************************************************************************/
// (ixyz) [ind]
// | (0, 2, 0) [5] | (1, 2, 0) [3] | (2, 2, 0) [4] |
// | (0, 1, 0) [2] | (1, 1, 0) [0] | (2, 1, 0) [1] |
// | (0, 0, 0) [8] | (1, 0, 0) [6] | (2, 0, 0) [7] |

void SimpleWorldToGridTest(voxel_grid::VoxelGrid<int>& grid, const double resolution) {
  double xyz[3];
  int ixyz[3];

  // just outside voxel 2 negative border
  xyz[0] = -resolution - 1e-5;
  xyz[1] = 0;
  xyz[2] = 0;
  EXPECT_EQ(grid.IsInMap(xyz), false);

  // just outside voxel 2 negative border
  xyz[0] = -resolution + 1e-5;
  xyz[1] = 0;
  xyz[2] = 0;
  EXPECT_EQ(grid.IsInMap(xyz), true);

  // just inside of voxel 2 positive border
  xyz[0] = -1e-5;
  xyz[1] = 0;
  xyz[2] = 0;
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 0);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 0 negative border
  xyz[0] = 0;
  xyz[1] = 0;
  xyz[2] = 0;
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 0 negative border
  xyz[0] = resolution - 1e-5;
  xyz[1] = 0;
  xyz[2] = 0;
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside voxel 1 negative border
  xyz[0] = resolution + 1e-5;
  xyz[1] = 0;
  xyz[2] = 0;
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 2);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside voxel 1 positive border
  xyz[0] = 2 * resolution - 1e-5;
  xyz[1] = 0;
  xyz[2] = 0;
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 2);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

TEST(VoxelGrid, WorldToGridStatic) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);

  SimpleWorldToGridTest(grid, resolution);
}

TEST(VoxelGrid, WorldToGridSubVoxelShiftX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  double xyz[3] = {0, 0, 0};
  int ixyz[3] = {0, 0, 0};

  // just inside of negative border
  origin[0] = 0;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside positive border
  origin[0] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

TEST(VoxelGrid, WorldToGridVoxelShiftX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  double xyz[3] = {0, 0, 0};
  int ixyz[3] = {0, 0, 0};

  // just inside of voxel 6 positive border
  origin[0] = -resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 2);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 6 negative border
  origin[0] = -1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 2);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 7 positive border
  origin[0] = resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 0);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 7 negative border
  origin[0] = 2 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 0);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

// TEST(VoxelGrid, WorldToGridVoxelWrapX) {
//   double origin[3] = {0, 0, 0};
//   double world_dimensions[3] = {3, 3, 1};
//   double resolution = 1;
//   voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
//   double xyz[3] = {-0.5 * resolution, -0.5 * resolution, -0.5 * resolution};
//   int ixyz[3] = {0, 0, 0};

//   // not yet implemented
// }

TEST(VoxelGrid, WorldToGridSubVoxelShiftY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  double xyz[3] = {0, 0, 0};
  int ixyz[3] = {0, 0, 0};

  // just inside of negative border
  origin[1] = 0;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside positive border
  origin[1] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

TEST(VoxelGrid, WorldToGridVoxelShiftY) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
  double xyz[3] = {0, 0, 0};
  int ixyz[3] = {0, 0, 0};

  // just inside of voxel 2 positive border
  origin[1] = -resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 2);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 2 negative border
  origin[1] = -1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 2);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 5 positive border
  origin[1] = resolution + 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 0);
  EXPECT_EQ(ixyz[2], 0);

  // just inside of voxel 5 negative border
  origin[1] = 2 * resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 0);
  EXPECT_EQ(ixyz[2], 0);
}

// TEST(VoxelGrid, WorldToGridVoxelWrapY) {
//   double origin[3] = {0, 0, 0};
//   double world_dimensions[3] = {3, 3, 1};
//   double resolution = 1;
//   voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);
//   int ixyz[3] = {0, 0, 0};

//   // not yet implemented
// }

TEST(VoxelGrid, WorldToGridSubVoxelShiftZ) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  double xyz[3] = {0, 0, 0};
  int ixyz[3] = {0, 0, 0};

    // just inside of negative border
  origin[2] = 0;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);

  // just inside positive border
  origin[2] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.WorldToGrid(xyz, ixyz);
  EXPECT_EQ(ixyz[0], 1);
  EXPECT_EQ(ixyz[1], 1);
  EXPECT_EQ(ixyz[2], 0);
}

/******************************************************************************/
//                              GridToWorld
/******************************************************************************/
// (ixyz) [ind]
// | (0, 2, 0) [5] | (1, 2, 0) [3] | (2, 2, 0) [4] |
// | (0, 1, 0) [2] | (1, 1, 0) [0] | (2, 1, 0) [1] |
// | (0, 0, 0) [8] | (1, 0, 0) [6] | (2, 0, 0) [7] |


void SimpleGridToWorldTest(voxel_grid::VoxelGrid<int>& grid) {
  double xyz[3];
  int ixyz[3];

  // voxel 4
  ixyz[0] = 2;
  ixyz[1] = 2;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], 1);
  EXPECT_EQ(xyz[1], 1);
  EXPECT_EQ(xyz[2], 0);

  // voxel 5
  ixyz[0] = 0;
  ixyz[1] = 2;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], -1);
  EXPECT_EQ(xyz[1], 1);
  EXPECT_EQ(xyz[2], 0);

  // voxel 3
  ixyz[0] = 1;
  ixyz[1] = 2;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], 0);
  EXPECT_EQ(xyz[1], 1);
  EXPECT_EQ(xyz[2], 0);

  // voxel 7
  ixyz[0] = 2;
  ixyz[1] = 0;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], 1);
  EXPECT_EQ(xyz[1], -1);
  EXPECT_EQ(xyz[2], 0);

  // voxel 8
  ixyz[0] = 0;
  ixyz[1] = 0;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], -1);
  EXPECT_EQ(xyz[1], -1);
  EXPECT_EQ(xyz[2], 0);

  // voxel 6
  ixyz[0] = 1;
  ixyz[1] = 0;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], 0);
  EXPECT_EQ(xyz[1], -1);
  EXPECT_EQ(xyz[2], 0);

  // voxel 1
  ixyz[0] = 2;
  ixyz[1] = 1;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], 1);
  EXPECT_EQ(xyz[1], 0);
  EXPECT_EQ(xyz[2], 0);

  // voxel 2
  ixyz[0] = 0;
  ixyz[1] = 1;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], -1);
  EXPECT_EQ(xyz[1], 0);
  EXPECT_EQ(xyz[2], 0);

  // voxel 0
  ixyz[0] = 1;
  ixyz[1] = 1;
  ixyz[2] = 0;
  grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(xyz[0], 0);
  EXPECT_EQ(xyz[1], 0);
  EXPECT_EQ(xyz[2], 0);
}

TEST(VoxelGrid, GridToWorldStatic) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);

  SimpleGridToWorldTest(grid);
}

TEST(VoxelGrid, GridToWorldSubVoxelShiftX) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;
  voxel_grid::VoxelGrid<int> grid(origin, world_dimensions, resolution);

  // just inside of voxel 8 positive border
  origin[0] = 0;
  grid.UpdateOrigin(origin);
  SimpleGridToWorldTest(grid);
  
  // just inside of voxel 8 negative border
  origin[0] = resolution - 1e-5;
  grid.UpdateOrigin(origin);
  SimpleGridToWorldTest(grid);
}

/******************************************************************************/
//                              Integration
/******************************************************************************/
// (ixyz) [ind]
// | (0, 2, 0) [1] | (1, 2, 0) [2] | (2, 2, 0) [0] |
// | (0, 1, 0) [7] | (1, 1, 0) [8] | (2, 1, 0) [6] |
// | (0, 0, 0) [4] | (1, 0, 0) [5] | (2, 0, 0) [3] |

// World Coordinates:   0     1R    2R    3R
// Grid Divisions:      |  0  |  1  |  2  |
// TEST(VoxelGrid, IndexToWorldToGrid) {
//   double origin[3] = {0, 0, 0};
//   double world_dimensions[3] = {1.6, 1, 1};
//   double resolution[3] = {0.2, 1.0, 1.0};

//   auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);

//   int index = 0;
//   double xyz[3];
//   int ixyz[3];
//   grid.IndexToWorld(index, xyz);
//   grid.WorldToGrid(xyz, ixyz);

//   EXPECT_EQ(0, ixyz[0]);
//   EXPECT_EQ(0, ixyz[1]);
//   EXPECT_EQ(0, ixyz[2]);
 
//   index = 1;
//   grid.IndexToWorld(index, xyz);
//   grid.WorldToGrid(xyz, ixyz);
 
//   EXPECT_EQ(1, ixyz[0]);
//   EXPECT_EQ(0, ixyz[1]);
//   EXPECT_EQ(0, ixyz[2]);

//   index = 2;
//   grid.IndexToWorld(index, xyz);
//   grid.WorldToGrid(xyz, ixyz);

//   EXPECT_EQ(2, ixyz[0]);
//   EXPECT_EQ(0, ixyz[1]);
//   EXPECT_EQ(0, ixyz[2]);
// }

// TEST(VoxelGrid, IndexToWorldWrapping) {
//   double origin[3] = {0, 0, 0};
//   double world_dimensions[3] = {1.5, 1.5, 0.5};
//   double resolution[3] = {0.5, 0.5, 0.5};

//   auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);

//   int index = 3;
//   double xyz[3];
//   grid.IndexToWorld(index, xyz);

//   std::cout << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << std::endl;
//   // EXPECT_EQ(xyz[0], -0.75);
//   // EXPECT_EQ(xyz[1], -0.25);
//   // EXPECT_EQ(xyz[2], -0.25);

//   origin[0] = -0.49;
//   grid.UpdateOrigin(origin);

//   grid.IndexToWorld(index, xyz);
//   // EXPECT_EQ(xyz[0], 0.75);
//   // EXPECT_EQ(xyz[1], -0.25);
//   // EXPECT_EQ(xyz[2], -0.25);
//   std::cout << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << std::endl;

//   int ixyz[3] = {2, 1, 0};
//   grid.GridToWorld(ixyz, xyz);  
//   std::cout << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << std::endl;
// }

// TEST(VoxelGrid, IndexToGridToWorldToGridToIndexOneCell) {
//   double origin[3] = {0, 0, 0};
//   double world_dimensions[3] = {1.5, 1.5, 0.5};
//   double resolution[3] = {0.5, 0.5, 0.5};

//   auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
//   double xyz[3];
//   int ixyz[3];

//   origin[0] = 1e-6;
//   int ind = 3;
//   grid.UpdateOrigin(origin);

//   grid.IndexToGrid(ind, ixyz);
//   EXPECT_EQ(ixyz[0], 0);
//   EXPECT_EQ(ixyz[1], 1);
//   EXPECT_EQ(ixyz[2], 0);

//   grid.GridToWorld(ixyz, xyz);
//   EXPECT_EQ(xyz[0], -0.75);
//   EXPECT_EQ(xyz[1], -0.25);
//   EXPECT_EQ(xyz[2], -0.25);

//   grid.WorldToGrid(xyz, ixyz);
//   EXPECT_EQ(ixyz[0], 0);
//   EXPECT_EQ(ixyz[1], 1);
//   EXPECT_EQ(ixyz[2], 0);

//   EXPECT_EQ(grid.GridToIndex(ixyz), ind);
// }

/******************************************************************************/
//                              Self Consistency
/******************************************************************************/

TEST(VoxelGrid, IndexToWorldToIndexMinimalOdd) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {2.0, 2.0, 1.0};
  double resolution = 1;
  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  double xyz[3];
  int ixyz[3] = {0, 0, 0};
  double updated_origin[3] = {world_dimensions[0] / 2 + resolution, 0, 0};
  grid.UpdateOrigin(updated_origin);

  grid.IndexToWorld(0, xyz);
  EXPECT_EQ(grid.WorldToIndex(xyz), 0);
}


TEST(VoxelGrid, IndexToGridToWorldToGridToIndexAllCells) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3.0, 3.0, 3.0};
  double resolution = 1;

  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  double xyz[3];
  int ixyz[3];

  for (float dx = -2.0 * resolution; dx <= 2.0 * resolution; dx += 0.1) {
    for (float dy = -2.0 * resolution; dy <= 2.0 * resolution; dy += 0.1) {
      for (float dz = -2.0 * resolution; dz <= 2.0 * resolution; dz += 0.1) {
        origin[0] = dx;
        origin[1] = dy;
        origin[2] = dz;
        grid.UpdateOrigin(origin);
        for(int i = 0; i < grid.GetNumCells() ; i++) {
          grid.IndexToGrid(i, ixyz);
          grid.GridToWorld(ixyz, xyz);
          grid.WorldToGrid(xyz, ixyz);
          EXPECT_EQ(grid.GridToIndex(ixyz), i);
        }
      }
    }
  }
}

TEST(VoxelGrid, IndexToGridToWorldToGridToIndexAllCellsWrapOdd) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3.0, 3.0, 3.0};
  double resolution = 1;

  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  double xyz[3];
  int ixyz[3];

  for (float dx = -2.0 * world_dimensions[0]; dx <= 2.0 * world_dimensions[0]; dx += 0.1) {
    for (float dy = -2.0 * world_dimensions[1]; dy <= 2.0 * world_dimensions[1]; dy += 0.1) {
      for (float dz = -2.0 * world_dimensions[2]; dz <= 2.0 * world_dimensions[2]; dz += 0.1) {
        origin[0] = dx;
        origin[1] = dy;
        origin[2] = dz;
        grid.UpdateOrigin(origin);
        for(int i = 0; i < grid.GetNumCells() ; i++) {
          grid.IndexToGrid(i, ixyz);
          grid.GridToWorld(ixyz, xyz);
          grid.WorldToGrid(xyz, ixyz);
          EXPECT_EQ(grid.GridToIndex(ixyz), i);
        }
      }
    }
  }
}

TEST(VoxelGrid, IndexToGridToWorldToGridToIndexAllCellsWrapEven) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {2.0, 2.0, 2.0};
  double resolution = 1;

  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  double xyz[3];
  int ixyz[3];

  for (float dx = -2.0 * world_dimensions[0]; dx <= 2.0 * world_dimensions[0]; dx += 0.1) {
    for (float dy = -2.0 * world_dimensions[1]; dy <= 2.0 * world_dimensions[1]; dy += 0.1) {
      for (float dz = -2.0 * world_dimensions[2]; dz <= 2.0 * world_dimensions[2]; dz += 0.1) {
        origin[0] = dx;
        origin[1] = dy;
        origin[2] = dz;
        grid.UpdateOrigin(origin);
        for(int i = 0; i < grid.GetNumCells() ; i++) {
          grid.IndexToGrid(i, ixyz);
          grid.GridToWorld(ixyz, xyz);
          grid.WorldToGrid(xyz, ixyz);
          EXPECT_EQ(grid.GridToIndex(ixyz), i);
        }
      }
    }
  }
}

TEST(VoxelGrid, UpdateOrigin1) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {5.0, 5.0, 1.0};
  double resolution = 1;

  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);
  int ixyz[3];

  origin[0] = -1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(11, ixyz);
  EXPECT_EQ(ixyz[0], 4);
  EXPECT_EQ(ixyz[1], 4);

  origin[0] = -resolution - 1e-5;
  grid.UpdateOrigin(origin);
  grid.IndexToGrid(11, ixyz);
  EXPECT_EQ(ixyz[0], 0);
  EXPECT_EQ(ixyz[1], 4);
}

TEST(VoxelGrid, UpdateOrigin2) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {5.0, 5.0, 1.0};
  double resolution = 1;

  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);

  origin[0] = -6;
  grid.UpdateOrigin(origin);
  int ixyz[3];
  grid.IndexToGrid(0, ixyz);
  EXPECT_EQ(ixyz[0], 3);
  EXPECT_EQ(ixyz[1], 2);
}

TEST(VoxelGrid, ExternalData) {
  double origin[3] = {0, 0, 0};
  int grid_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  std::vector<int> data = {0, 1, 2, 3, 4, 5, 6, 7, 8};

  auto grid = voxel_grid::VoxelGrid<int>(origin, grid_dimensions, resolution, data);

  for(int i = 0; i < grid.GetNumCells(); i++) {
    EXPECT_EQ(grid.ReadValue(i), i);
  }
}

TEST(VoxelGrid, SliceIndexes) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {5.0, 5.0, 1.0};
  double resolution = 1;

  auto grid = voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution);

  origin[0] = -2;
  std::vector<int> slice_inds = grid.UpdateOrigin(origin);

  EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), 11) != slice_inds.end());
  EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), 12) != slice_inds.end());
  EXPECT_EQ(slice_inds.size(), 10);

  origin[0] = 2;
  slice_inds = grid.UpdateOrigin(origin);

  EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), 11) != slice_inds.end());
  EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), 12) != slice_inds.end());
  EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), 13) != slice_inds.end());
  EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), 14) != slice_inds.end());
  EXPECT_EQ(slice_inds.size(), 20);

  origin[0] = 8;
  slice_inds = grid.UpdateOrigin(origin);
  EXPECT_EQ(slice_inds.size(), 25);

  for(int index = 0; index < grid.GetNumCells(); index++) {
    EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), index) != slice_inds.end());
  }

  origin[0] = -100;
  slice_inds = grid.UpdateOrigin(origin);
  EXPECT_EQ(slice_inds.size(), 25);

  for(int index = 0; index < grid.GetNumCells(); index++) {
    EXPECT_TRUE(std::find(slice_inds.begin(), slice_inds.end(), index) != slice_inds.end());
  }
}
