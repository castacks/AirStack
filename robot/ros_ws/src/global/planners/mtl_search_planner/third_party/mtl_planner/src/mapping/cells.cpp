#include "mtl/mapping/cells.hpp"

#include <cstdio>
#include <vector>

#include "mtl/core/kmeans.hpp"

namespace mtl::mapping {

CellSet extractValidCells(const BeliefField& belief, double targetCellSize,
                          double minBeliefMass, bool verbose) {
    CellSet out;
    out.cellSize      = targetCellSize;
    out.minBeliefMass = minBeliefMass;
    if (belief.empty()) return out;

    // The prior as a probability mass function.  A no-op (scale 1) on
    // mapgen::generateBeliefMap's output, which already sums to 1; a host grid
    // that does not is normalised here so the masses are still probabilities
    // and the threshold still means the same thing.
    const double rawTotal = belief.values.sum();
    if (!(rawTotal > 0.0)) return out;
    const double norm = 1.0 / rawTotal;

    const Index rows = belief.rows();
    const Index cols = belief.cols();

    const double gridResX = belief.gridResX();
    const double gridResY = belief.gridResY();
    out.gridRes = Vec2(gridResX, gridResY);

    const auto pixelsPerCellX =
        std::max<Index>(1, static_cast<Index>(std::lround(targetCellSize / gridResX)));
    const auto pixelsPerCellY =
        std::max<Index>(1, static_cast<Index>(std::lround(targetCellSize / gridResY)));

    // CEIL, so the partial blocks at the far edge are not dropped.
    const Index nRows = (rows + pixelsPerCellY - 1) / pixelsPerCellY;
    const Index nCols = (cols + pixelsPerCellX - 1) / pixelsPerCellX;

    const double pixelArea = gridResX * gridResY;  // only for the reported cell area

    std::vector<Vec2>   centers;
    std::vector<double> mass, meanB, peakB, nPix, area;

    for (Index r = 0; r < nRows; ++r) {
        const Index r0 = r * pixelsPerCellY;
        const Index r1 = std::min((r + 1) * pixelsPerCellY, rows);  // exclusive
        for (Index c = 0; c < nCols; ++c) {
            const Index c0 = c * pixelsPerCellX;
            const Index c1 = std::min((c + 1) * pixelsPerCellX, cols);  // exclusive

            const auto block = belief.values.block(r0, c0, r1 - r0, c1 - c0);
            // Probability that the target is in this block.
            const double cellMass = block.sum() * norm;
            const double n        = static_cast<double>(block.size());
            if (!(cellMass > minBeliefMass)) continue;

            // Physical centre of the pixels actually evaluated, so the anchor
            // sits in the middle of its block even where the block is cut off.
            const double cy = 0.5 * static_cast<double>(r0 + r1 - 1) * gridResY;
            const double cx = 0.5 * static_cast<double>(c0 + c1 - 1) * gridResX;

            centers.emplace_back(cx, cy);
            mass.push_back(cellMass);
            meanB.push_back(cellMass / n);
            peakB.push_back(block.maxCoeff() * norm);
            nPix.push_back(n);
            area.push_back(n * pixelArea);
        }
    }

    const auto m = static_cast<Index>(centers.size());
    out.centers.resize(m, 2);
    out.mass.resize(m);
    out.meanBelief.resize(m);
    out.peakBelief.resize(m);
    out.nPix.resize(m);
    out.area.resize(m);
    for (Index i = 0; i < m; ++i) {
        out.centers.row(i) = centers[static_cast<std::size_t>(i)].transpose();
        out.mass(i)        = mass[static_cast<std::size_t>(i)];
        out.meanBelief(i)  = meanB[static_cast<std::size_t>(i)];
        out.peakBelief(i)  = peakB[static_cast<std::size_t>(i)];
        out.nPix(i)        = nPix[static_cast<std::size_t>(i)];
        out.area(i)        = area[static_cast<std::size_t>(i)];
    }

    out.totalMapMass = rawTotal * norm;  // 1, up to round-off
    out.retainedMass = out.mass.sum();
    out.massNorm     = (out.retainedMass > 0.0) ? (out.mass / out.retainedMass).eval()
                                                : VecX::Zero(m);

    if (verbose) {
        std::printf("Extracted %lld valid cells (belief mass > %.3g per cell).\n",
                    static_cast<long long>(m), minBeliefMass);
        std::printf(
            "  belief accounting: retained %.4f of %.4f total prior mass (%.1f%%) over %lld "
            "cells.\n",
            out.retainedMass, out.totalMapMass,
            100.0 * out.retainedMass / std::max(out.totalMapMass, 1e-300),
            static_cast<long long>(m));
    }
    return out;
}

ClusterSet clusterCells(const Path2& validCenters, double maxRadius,
                        const ClusterParams& opts, std::uint64_t seed, bool verbose) {
    ClusterSet out;
    const Index m = validCenters.rows();
    if (m == 0) return out;

    core::KMeansOptions ko;
    ko.maxIter    = opts.kmeansMaxIter;
    ko.replicates = opts.kmeansReplicates;

    for (int k = 1;; ++k) {
        if (k >= static_cast<int>(m)) {
            // One cluster per cell: the radius condition is satisfied trivially.
            out.centroids = validCenters;
            out.cellCluster.resize(static_cast<std::size_t>(m));
            for (Index i = 0; i < m; ++i) out.cellCluster[static_cast<std::size_t>(i)] = static_cast<int>(i);
            out.maxRadius = 0.0;
            break;
        }

        ko.seed = seed + static_cast<std::uint64_t>(k) * 7919ULL;
        const core::KMeansResult r = core::kmeans(validCenters, k, ko);
        const double maxDist = core::maxClusterRadius(validCenters, r);

        if (maxDist <= maxRadius) {
            out.centroids   = r.centroids;
            out.cellCluster = r.assignment;
            out.maxRadius   = maxDist;
            break;
        }
    }

    if (verbose) {
        std::printf("Clustered into %lld drone waypoints (max radius: %.1f m).\n",
                    static_cast<long long>(out.centroids.rows()), out.maxRadius);
    }
    return out;
}

void computeClusterRewards(const VecX& cellMass, ClusterSet& clusters) {
    const auto k = static_cast<Index>(clusters.centroids.rows());
    clusters.reward = VecX::Zero(k);
    clusters.cellIdx.assign(static_cast<std::size_t>(k), {});
    if (k == 0) return;

    for (std::size_t i = 0; i < clusters.cellCluster.size(); ++i) {
        const int c = clusters.cellCluster[i];
        if (c < 0 || c >= static_cast<int>(k)) continue;
        clusters.cellIdx[static_cast<std::size_t>(c)].push_back(static_cast<Index>(i));
        clusters.reward(c) += cellMass(static_cast<Index>(i));
    }
}

}  // namespace mtl::mapping
