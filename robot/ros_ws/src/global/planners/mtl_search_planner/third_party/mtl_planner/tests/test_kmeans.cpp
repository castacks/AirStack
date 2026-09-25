// k-means, and the cell -> cluster abstraction that stands on it.
#include <random>

#include "mtl/core/kmeans.hpp"
#include "mtl/mapping/cells.hpp"
#include "test_util.hpp"

using namespace mtl;

int main() {
    // --- three well-separated blobs must come back as three clusters -------
    {
        std::mt19937_64 rng(7);
        std::normal_distribution<double> noise(0.0, 5.0);
        const Vec2 centres[3] = {Vec2(0, 0), Vec2(1000, 0), Vec2(0, 1000)};

        Path2 pts(150, 2);
        for (Index i = 0; i < 150; ++i) {
            const Vec2& c = centres[i % 3];
            pts(i, 0) = c.x() + noise(rng);
            pts(i, 1) = c.y() + noise(rng);
        }

        core::KMeansOptions ko;
        ko.replicates = 5;
        ko.seed       = 42;
        const core::KMeansResult r = core::kmeans(pts, 3, ko);

        CHECK(r.centroids.rows() == 3);
        CHECK(core::maxClusterRadius(pts, r) < 50.0);
        // Every point lands in the cluster whose centroid is its own blob.
        for (Index i = 0; i < 150; ++i) {
            const int c = r.assignment[static_cast<std::size_t>(i)];
            CHECK((r.centroids.row(c).transpose() - centres[i % 3]).norm() < 50.0);
        }
    }

    // --- k above the point count is clamped, not an error ------------------
    {
        Path2 pts(3, 2);
        pts << 0, 0, 1, 1, 2, 2;
        const core::KMeansResult r = core::kmeans(pts, 10, {});
        CHECK(r.centroids.rows() == 3);
    }

    // --- clusterCells honours the radius guarantee the gimbal depends on ---
    {
        std::mt19937_64 rng(11);
        std::uniform_real_distribution<double> u(0.0, 5000.0);
        Path2 cells(200, 2);
        for (Index i = 0; i < 200; ++i) {
            cells(i, 0) = u(rng);
            cells(i, 1) = u(rng);
        }
        const double maxRadius = 500.0;
        ClusterSet cl = mapping::clusterCells(cells, maxRadius, ClusterParams{}, 3, false);

        CHECK(cl.size() > 0);
        CHECK(cl.maxRadius <= maxRadius + 1e-9);
        CHECK(static_cast<Index>(cl.cellCluster.size()) == cells.rows());
        // Every cell really is within the radius of ITS centroid - the property
        // the cluster abstraction's reward exactness rests on.
        for (Index i = 0; i < cells.rows(); ++i) {
            const int c = cl.cellCluster[static_cast<std::size_t>(i)];
            CHECK((cells.row(i) - cl.centroids.row(c)).norm() <= maxRadius + 1e-9);
        }

        // Rewards are exactly the summed mass of the owned cells.
        VecX mass = VecX::Ones(cells.rows());
        mapping::computeClusterRewards(mass, cl);
        double total = 0.0;
        Index  owned = 0;
        for (Index k = 0; k < cl.size(); ++k) {
            total += cl.reward(k);
            owned += static_cast<Index>(cl.cellIdx[static_cast<std::size_t>(k)].size());
        }
        CHECK_NEAR(total, mass.sum(), 1e-9);
        CHECK(owned == cells.rows());
    }

    return test::report("test_kmeans");
}
