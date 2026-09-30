// =============================================================================
//  mtl_curve/optimization/fast_grid.hpp
//
//  The coarse grid the fast objective integrates over - the port of
//  buildFastGrid.m.  The optimiser never touches the full-resolution belief:
//  the prior is block-SUMMED (mass preserving) into `downsample` x `downsample`
//  blocks and each coarse pixel sits at the centre of its block.  The stencil
//  lists every coarse-pixel offset that can lie within Rk of a kernel centre
//  whose nearest coarse pixel is the stencil origin.
//
//  rasterizeCells is the planFromCells path: a host that only has CELLS (the
//  mtl.scenario/1 interchange carries centres and masses, not the grid) gets a
//  prior with each cell's mass spread uniformly over its targetCellSize block.
// =============================================================================
#ifndef MTLC_OPTIMIZATION_FAST_GRID_HPP
#define MTLC_OPTIMIZATION_FAST_GRID_HPP

#include "mtl_curve/types.hpp"

namespace mtl::curve::optimization {

/// @param downsample belief pixels per coarse pixel, per side (>= 1)
/// @param Rk         stencil radius [m]; <= 0 leaves the stencil empty
FastGrid buildFastGrid(const BeliefField& belief, int downsample, double Rk);

/// (Re)build the stencil of `G` for radius Rk.
void setStencil(FastGrid& G, double Rk);

/// Coarse grid of pixel size `hg` over [0, mapSize]^2 with each cell's mass
/// spread uniformly over the pixels whose centres fall in its block of side
/// `cellSize` (a cell whose block holds no pixel centre puts its mass on the
/// nearest pixel).  Sums to cells.mass.sum() normalised to 1.
FastGrid rasterizeCells(const CellSet& cells, double mapSize, double hg, double cellSize, double Rk);

}  // namespace mtl::curve::optimization

#endif  // MTLC_OPTIMIZATION_FAST_GRID_HPP
