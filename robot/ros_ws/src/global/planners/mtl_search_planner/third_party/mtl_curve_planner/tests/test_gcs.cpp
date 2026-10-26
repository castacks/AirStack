// The GCS seed (mtl_curve/gcs):
//   1. convex sets: each contains its centre, its vertices satisfy H v <= h, it
//      stays inside the disc and the map, and no two sets overlap (Voronoi);
//   2. the conic solver: a projection onto a box (known optimum);
//   3. fixed-order route: points lie in their sets; a single set on the
//      straight line from start to goal costs exactly the straight line;
//   4. tour: visits every set exactly once, points in their sets, never longer
//      than the fixed-order route of the nearest-neighbour order, and the
//      relaxation is a lower bound;
//   5. kernel clusters: the info adds up to the mass taken, decreasing order;
//   6. planAgentGcs: the trimmed route fits the budget; dropping keeps the
//      highest-information clusters under the 'info' rule;
//   7. the centre pull moves the points toward the centres.
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <set>

#include "mtl_curve/gcs/gcs.hpp"
#include "test_util.hpp"

using namespace mtl::curve;
using namespace mtl::curve::gcs;

int main() {
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> U(300.0, 4700.0);
    const Index K = 12;
    Path2 C(K, 2);
    for (Index k = 0; k < K; ++k) C.row(k) << U(rng), U(rng);
    const double r = 520.0;
    const Eigen::Vector4d box(0, 5000, 0, 5000);
    const std::vector<ConvexSet> S = convexSets(C, r, box, 16);
    GcsParams o;

    // ---- 1. sets ----------------------------------------------------------------------
    for (Index k = 0; k < K; ++k) {
        const ConvexSet& s = S[static_cast<std::size_t>(k)];
        CHECK((s.H * s.c - s.h).maxCoeff() <= 1e-9);
        CHECK((s.H * s.V.transpose() - s.h.replicate(1, s.V.rows())).maxCoeff() <= 1e-6);
        for (Index i = 0; i < s.V.rows(); ++i) {
            CHECK((Vec2(s.V.row(i).transpose()) - s.c).norm() <= r + 1e-6);
            CHECK(s.V(i, 0) >= -1e-9 && s.V(i, 0) <= 5000 + 1e-9);
            // Voronoi: no vertex is strictly closer to another centre
            for (Index j = 0; j < K; ++j)
                CHECK((Vec2(s.V.row(i).transpose()) - Vec2(C.row(j).transpose())).norm() >=
                      (Vec2(s.V.row(i).transpose()) - s.c).norm() - 1e-6);
        }
    }

    // ---- 2. conic solver: min |x - (3,4)| over the unit box -----------------------------
    {
        ConicProblem P;
        P.c = Eigen::Vector3d(1, 0, 0);
        std::vector<Eigen::Triplet<double>> t = {{0, 1, 1}, {1, 2, 1}, {2, 1, -1}, {3, 2, -1},
                                                 {4, 0, -1}, {5, 1, -1}, {6, 2, -1}};
        P.A.resize(7, 3);
        P.A.setFromTriplets(t.begin(), t.end());
        P.b.resize(7);
        P.b << 1, 1, 0, 0, 0, -3, -4;
        P.l = 4;
        P.q = {3};
        GcsParams oo;
        oo.tol = 1e-8;
        const ConicResult R = solveConic(P, oo);
        CHECK(R.ok);
        CHECK_NEAR(R.x(0), std::sqrt(13.0), 1e-4);
        CHECK_NEAR(R.x(1), 1.0, 1e-4);
    }

    // ---- 3. fixed order ------------------------------------------------------------------
    {
        Path2 c1(1, 2);
        c1 << 2500, 2600;
        const std::vector<ConvexSet> s1 = convexSets(c1, 300, box, 16);
        const FixedOrderResult F = fixedOrderRoute(s1, {0}, Vec2(1000, 2500), Vec2(4000, 2500), {}, o);
        CHECK_NEAR(F.len, 3000.0, 1.0);
        std::vector<Index> ord(static_cast<std::size_t>(K));
        std::iota(ord.begin(), ord.end(), 0);
        const FixedOrderResult F2 = fixedOrderRoute(S, ord, Vec2(2000, 2000), std::nullopt, {}, o);
        for (Index k = 0; k < K; ++k)
            CHECK((S[static_cast<std::size_t>(k)].H * Vec2(F2.pts.row(k).transpose()) - S[static_cast<std::size_t>(k)].h).maxCoeff() <= 1.0);
    }

    // ---- 4. tour ---------------------------------------------------------------------------
    for (const bool closed : {false, true}) {
        const Vec2 s0(2000, 2000);
        const std::optional<Vec2> g = closed ? std::optional<Vec2>(s0) : std::nullopt;
        const TourResult T = tour(S, s0, g, {}, o);
        std::set<Index> vis(T.order.begin(), T.order.end());
        CHECK(static_cast<Index>(vis.size()) == K && static_cast<Index>(T.order.size()) == K);
        for (Index k = 0; k < K; ++k) {
            const ConvexSet& s = S[static_cast<std::size_t>(T.order[static_cast<std::size_t>(k)])];
            CHECK((s.H * Vec2(T.pts.row(k).transpose()) - s.h).maxCoeff() <= 1.0);
        }
        CHECK(T.lb <= T.len + 1.0);
        // nearest-neighbour order on the centres, re-solved exactly
        std::vector<Index> nn;
        std::vector<char> used(static_cast<std::size_t>(K), 0);
        Vec2 pos = s0;
        for (Index i = 0; i < K; ++i) {
            Index b = -1;
            for (Index k = 0; k < K; ++k)
                if (!used[static_cast<std::size_t>(k)] &&
                    (b < 0 || (Vec2(C.row(k).transpose()) - pos).norm() < (Vec2(C.row(b).transpose()) - pos).norm()))
                    b = k;
            nn.push_back(b);
            used[static_cast<std::size_t>(b)] = 1;
            pos = C.row(b).transpose();
        }
        const FixedOrderResult Fn = fixedOrderRoute(S, nn, s0, g, {}, o);
        std::printf("  tour (%s): len %.1f, nearest-neighbour %.1f, lower bound %.1f, %d cuts, %d solves\n",
                    closed ? "closed" : "open", T.len, Fn.len, T.lb, T.nCuts, T.nSolves);
        CHECK(T.len <= Fn.len + 1.0);
    }

    // ---- 5. kernel clusters ----------------------------------------------------------------
    {
        FastGrid G;
        G.hg = 25;
        G.nx = G.ny = 201;
        G.x0 = G.y0 = 0;
        G.prior = MatX::Zero(G.ny, G.nx);
        for (Index c = 0; c < G.nx; ++c)
            for (Index rr = 0; rr < G.ny; ++rr) {
                const double x = G.x(c), y = G.y(rr);
                G.prior(rr, c) = std::exp(-((x - 1500) * (x - 1500) + (y - 3000) * (y - 3000)) / (2 * 300.0 * 300.0)) +
                                 0.5 * std::exp(-((x - 3500) * (x - 3500) + (y - 1200) * (y - 1200)) / (2 * 400.0 * 400.0));
            }
        G.prior /= G.prior.sum();
        KernelClusterParams kp;
        const KernelClusters KC = kernelClusters(G, 520, kp);
        CHECK(KC.centroids.rows() >= 2);
        CHECK(KC.info.sum() <= 1.0 + 1e-9 && KC.info.sum() >= 0.9);
        for (Index k = 1; k < KC.info.size(); ++k) CHECK(KC.info(k) <= KC.info(k - 1) + 1e-12);
        CHECK((Vec2(KC.centroids.row(0).transpose()) - Vec2(1500, 3000)).norm() < 100);
    }

    // ---- 6. budget trim -----------------------------------------------------------------------
    {
        VecX info(K);
        for (Index k = 0; k < K; ++k) info(k) = 1.0 + static_cast<double>(k);
        std::vector<Index> ids(static_cast<std::size_t>(K));
        std::iota(ids.begin(), ids.end(), 0);
        for (const GcsDropRule rule : {GcsDropRule::Info, GcsDropRule::Ratio}) {
            GcsParams oo = o;
            oo.dropRule = rule;
            const AgentGcsResult A = planAgentGcs(S, info, ids, Vec2(2000, 2000), std::nullopt, 4750, 100, oo);
            std::printf("  budget trim (%s): kept %zu, dropped %zu, route %.0f m, %d solves\n", toString(rule),
                        A.clusters.size(), A.dropped.size(), A.len, A.nSolves);
            CHECK(A.len <= 4750 + 1e-6);
            CHECK(A.clusters.size() + A.dropped.size() == static_cast<std::size_t>(K));
            CHECK_NEAR(A.len, routeLength(A.wp, Vec2(2000, 2000), std::nullopt) +
                                  turnOverhead(A.wp, Vec2(2000, 2000), std::nullopt, 100), 1e-6);
            if (rule == GcsDropRule::Info) {
                double minKept = kInf, maxDrop = -kInf;
                for (const Index k : A.clusters) minKept = std::min(minKept, info(k));
                for (const Index k : A.dropped) maxDrop = std::max(maxDrop, info(k));
                CHECK(maxDrop <= minKept);
            }
        }
    }

    // ---- 7. centre pull -----------------------------------------------------------------------
    {
        const TourResult T0 = tour(S, Vec2(2000, 2000), std::nullopt, {}, o);
        const TourResult T1 = tour(S, Vec2(2000, 2000), std::nullopt, std::vector<double>(static_cast<std::size_t>(K), 2.0), o);
        double d0 = 0, d1 = 0;
        for (Index k = 0; k < K; ++k) {
            d0 += (Vec2(T0.pts.row(k).transpose()) - S[static_cast<std::size_t>(T0.order[static_cast<std::size_t>(k)])].c).norm();
            d1 += (Vec2(T1.pts.row(k).transpose()) - S[static_cast<std::size_t>(T1.order[static_cast<std::size_t>(k)])].c).norm();
        }
        std::printf("  centre pull: mean offset %.0f m -> %.0f m, route %.0f -> %.0f m\n", d0 / K, d1 / K, T0.len, T1.len);
        CHECK(d1 < d0);
    }
    return test::report("test_gcs");
}
