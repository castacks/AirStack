#include "mtl/routing/tsp.hpp"

#include <algorithm>
#include <limits>

namespace mtl::routing {
namespace {

double dist(const Path2& p, Index i, Index j) { return (p.row(i) - p.row(j)).norm(); }

/// Greedy nearest neighbour from node 0, then 2-opt over the open path.
std::vector<Index> nearestNeighbourThen2Opt(const Path2& nodes) {
    const Index n = nodes.rows();
    std::vector<Index> order;
    order.reserve(static_cast<std::size_t>(n));
    if (n == 0) return order;

    std::vector<char> visited(static_cast<std::size_t>(n), 0);
    order.push_back(0);
    visited[0] = 1;
    for (Index step = 1; step < n; ++step) {
        const Index last = order.back();
        double best  = std::numeric_limits<double>::infinity();
        Index  bestI = -1;
        for (Index i = 0; i < n; ++i) {
            if (visited[static_cast<std::size_t>(i)]) continue;
            const double d = dist(nodes, last, i);
            if (d < best) {
                best  = d;
                bestI = i;
            }
        }
        order.push_back(bestI);
        visited[static_cast<std::size_t>(bestI)] = 1;
    }

    // 2-opt on the open path: the final node may move, so the j == n-1 case
    // compares only the single edge that exists there.
    bool improved = true;
    while (improved) {
        improved = false;
        for (Index i = 0; i + 1 < n; ++i) {
            for (Index j = i + 2; j < n; ++j) {
                double cur, next;
                if (j + 1 < n) {
                    cur  = dist(nodes, order[static_cast<std::size_t>(i)],
                                order[static_cast<std::size_t>(i + 1)]) +
                           dist(nodes, order[static_cast<std::size_t>(j)],
                                order[static_cast<std::size_t>(j + 1)]);
                    next = dist(nodes, order[static_cast<std::size_t>(i)],
                                order[static_cast<std::size_t>(j)]) +
                           dist(nodes, order[static_cast<std::size_t>(i + 1)],
                                order[static_cast<std::size_t>(j + 1)]);
                } else {
                    cur  = dist(nodes, order[static_cast<std::size_t>(i)],
                                order[static_cast<std::size_t>(i + 1)]);
                    next = dist(nodes, order[static_cast<std::size_t>(i)],
                                order[static_cast<std::size_t>(j)]);
                }
                if (next < cur - 1e-5) {
                    std::reverse(order.begin() + static_cast<long>(i + 1),
                                 order.begin() + static_cast<long>(j + 1));
                    improved = true;
                }
            }
        }
    }
    return order;
}

}  // namespace

std::vector<Index> tspVisitOrder(const Path2& points, const Vec2& start) {
    const Index n = points.rows();
    std::vector<Index> visit;
    if (n == 0) return visit;

    Path2 nodes(n + 1, 2);
    nodes.row(0) = start.transpose();
    nodes.bottomRows(n) = points;

    const std::vector<Index> order = nearestNeighbourThen2Opt(nodes);
    visit.reserve(order.size());
    for (const Index idx : order) {
        if (idx == 0) continue;  // the launch point is not a visit
        visit.push_back(idx - 1);
    }
    return visit;
}

Path2 calculateTSPPath(const Path2& points, const Vec2& start) {
    const std::vector<Index> visit = tspVisitOrder(points, start);
    Path2 route(static_cast<Index>(visit.size()) + 1, 2);
    route.row(0) = start.transpose();
    for (std::size_t i = 0; i < visit.size(); ++i)
        route.row(static_cast<Index>(i) + 1) = points.row(visit[i]);
    return route;
}

std::vector<Index> calculateMicroTSPOrder(const Path2& cells, const Vec2& current,
                                          const std::optional<Vec2>& next) {
    const Index n = cells.rows();
    std::vector<Index> order;
    if (n <= 1) {
        if (n == 1) order.push_back(0);
        return order;
    }

    // 1. entry node: closest to the previous cluster's exit.
    VecX dStart(n);
    for (Index i = 0; i < n; ++i) dStart(i) = (cells.row(i).transpose() - current).norm();
    Index entry = 0;
    dStart.minCoeff(&entry);

    // 2. exit node: closest to the next cluster's entry.
    Index exit = 0;
    if (next.has_value() && n > 2) {
        VecX dNext(n);
        for (Index i = 0; i < n; ++i) dNext(i) = (cells.row(i).transpose() - *next).norm();
        dNext(entry) = std::numeric_limits<double>::infinity();
        dNext.minCoeff(&exit);
    } else if (n == 2) {
        exit = (entry == 0) ? 1 : 0;
    } else {
        dStart.maxCoeff(&exit);
        if (exit == entry) exit = (entry + 1) % n;
    }

    // 3. lock both boundaries, 4. greedy nearest neighbour for the middle.
    order.assign(static_cast<std::size_t>(n), -1);
    std::vector<char> visited(static_cast<std::size_t>(n), 0);
    order.front() = entry;
    order.back()  = exit;
    visited[static_cast<std::size_t>(entry)] = 1;
    visited[static_cast<std::size_t>(exit)]  = 1;

    for (Index step = 1; step + 1 < n; ++step) {
        const Index last = order[static_cast<std::size_t>(step - 1)];
        double best  = std::numeric_limits<double>::infinity();
        Index  bestI = -1;
        for (Index i = 0; i < n; ++i) {
            if (visited[static_cast<std::size_t>(i)]) continue;
            const double d = dist(cells, last, i);
            if (d < best) {
                best  = d;
                bestI = i;
            }
        }
        if (bestI < 0) break;
        order[static_cast<std::size_t>(step)] = bestI;
        visited[static_cast<std::size_t>(bestI)] = 1;
    }

    // 5. 2-opt, strictly bounded so the endpoints are never flipped.
    bool improved = true;
    while (improved) {
        improved = false;
        for (Index i = 0; i + 2 < n; ++i) {
            for (Index j = i + 2; j + 1 < n; ++j) {
                const double cur = dist(cells, order[static_cast<std::size_t>(i)],
                                        order[static_cast<std::size_t>(i + 1)]) +
                                   dist(cells, order[static_cast<std::size_t>(j)],
                                        order[static_cast<std::size_t>(j + 1)]);
                const double nxt = dist(cells, order[static_cast<std::size_t>(i)],
                                        order[static_cast<std::size_t>(j)]) +
                                   dist(cells, order[static_cast<std::size_t>(i + 1)],
                                        order[static_cast<std::size_t>(j + 1)]);
                if (nxt < cur - 1e-5) {
                    std::reverse(order.begin() + static_cast<long>(i + 1),
                                 order.begin() + static_cast<long>(j + 1));
                    improved = true;
                }
            }
        }
    }
    return order;
}

void buildSensorPathOrdered(const Path2& validCenters,
                            const std::vector<std::vector<Index>>& entityCells,
                            const Path2& routeXY, Path2& sensorTargets,
                            std::vector<Index>& sensorMacroRow,
                            std::vector<Index>& sensorCellIdx) {
    sensorTargets.resize(0, 2);
    sensorMacroRow.clear();
    sensorCellIdx.clear();

    const Index nRows = routeXY.rows();
    if (nRows < 2 || entityCells.empty()) return;

    std::vector<Vec2>  pts;
    Vec2 current = routeXY.row(0).transpose();

    for (Index row = 1; row < nRows; ++row) {
        const auto e = static_cast<std::size_t>(row - 1);
        if (e >= entityCells.size()) break;
        const std::vector<Index>& idx = entityCells[e];
        if (idx.empty()) continue;

        Path2 cells(static_cast<Index>(idx.size()), 2);
        for (std::size_t i = 0; i < idx.size(); ++i)
            cells.row(static_cast<Index>(i)) = validCenters.row(idx[i]);

        std::optional<Vec2> next;
        if (row + 1 < nRows) next = routeXY.row(row + 1).transpose();

        std::vector<Index> perm;
        if (cells.rows() >= 2) {
            perm = calculateMicroTSPOrder(cells, current, next);
        } else {
            perm.push_back(0);
        }

        for (const Index p : perm) {
            pts.emplace_back(cells(p, 0), cells(p, 1));
            sensorMacroRow.push_back(row);
            sensorCellIdx.push_back(idx[static_cast<std::size_t>(p)]);
        }
        if (!perm.empty()) current = cells.row(perm.back()).transpose();
    }

    sensorTargets.resize(static_cast<Index>(pts.size()), 2);
    for (std::size_t i = 0; i < pts.size(); ++i)
        sensorTargets.row(static_cast<Index>(i)) = pts[i].transpose();
}

}  // namespace mtl::routing
