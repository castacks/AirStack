// Copyright (c) 2026 Carnegie Mellon University
//
// Licensed under the Apache License, Version 2.0 (the "License").
// See coverage_planner_logic.hpp for full license text.

#include "coverage_planner_logic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <utility>

namespace coverage_planner {

namespace {

constexpr double kEps = 1e-9;

constexpr double kCoordEpsRel = 1e-12;
constexpr double kCoordEpsAbs = 1e-10;

inline double hypot2(const Point2D& u, const Point2D& v) {
    const double dx = u.x - v.x;
    const double dy = u.y - v.y;
    return dx * dx + dy * dy;
}

inline double hypot_len(const Point2D& u, const Point2D& v) {
    return std::hypot(u.x - v.x, u.y - v.y);
}

double point_segment_distance_squared(const Point2D& p, const Point2D& a,
                                      const Point2D& b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double wx = p.x - a.x;
    const double wy = p.y - a.y;
    const double vv = vx * vx + vy * vy;
    if (vv < kCoordEpsAbs * kCoordEpsAbs)
        return hypot2(p, a);
    double t = (wx * vx + wy * vy) / vv;
    t = std::clamp(t, 0.0, 1.0);
    const double sx = a.x + t * vx - p.x;
    const double sy = a.y + t * vy - p.y;
    return sx * sx + sy * sy;
}

// True if closed segment ab contains p within tolerance (for boundary tests).
bool point_on_closed_segment_tol(const Point2D& p, const Point2D& a,
                                 const Point2D& b) {
    // Cross product ~= 0 AND projection within [min,max].
    const double cross = std::fabs((p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x));
    const double lx = std::max(std::fabs(b.x - a.x), std::fabs(b.y - a.y));
    const double tol_cross = std::max(kCoordEpsAbs * std::max(1.0, lx), lx * kCoordEpsRel);
    if (cross > tol_cross)
        return false;
    const double minx = std::min(a.x, b.x) - kCoordEpsAbs;
    const double maxx = std::max(a.x, b.x) + kCoordEpsAbs;
    const double miny = std::min(a.y, b.y) - kCoordEpsAbs;
    const double maxy = std::max(a.y, b.y) + kCoordEpsAbs;
    return p.x >= minx && p.x <= maxx && p.y >= miny && p.y <= maxy;
}

Polygon2D rotate_polygon(const Polygon2D& polygon, double angle_rad) {
    Polygon2D out;
    out.reserve(polygon.size());
    for (const auto& p : polygon) out.push_back(rotate(p, angle_rad));
    return out;
}

// Axis-aligned bounding box of a polygon.
struct BBox {
    double x_min, x_max, y_min, y_max;
};
BBox bbox(const Polygon2D& polygon) {
    BBox b{std::numeric_limits<double>::infinity(),
           -std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           -std::numeric_limits<double>::infinity()};
    for (const auto& p : polygon) {
        b.x_min = std::min(b.x_min, p.x);
        b.x_max = std::max(b.x_max, p.x);
        b.y_min = std::min(b.y_min, p.y);
        b.y_max = std::max(b.y_max, p.y);
    }
    return b;
}

// Intersect the horizontal line y = y_sweep with all polygon edges.
// Returns the x-values of intersection points, sorted ascending.
//
// This correctly handles non-convex (but simple) polygons: the set of
// intersections comes out in pairs (entry, exit, entry, exit, ...),
// so the result is split into alternating segments before building
// the sweep row.
std::vector<double> sweep_line_intersections(const Polygon2D& polygon, double y_sweep) {
    std::vector<double> xs;
    const std::size_t n = polygon.size();
    if (n < 3) return xs;

    for (std::size_t i = 0; i < n; ++i) {
        const Point2D& a = polygon[i];
        const Point2D& b = polygon[(i + 1) % n];

        const double y1 = a.y;
        const double y2 = b.y;

        // Skip edges that don't straddle the sweep line. Use a half-open
        // convention (include lower endpoint, exclude upper) so a sweep
        // line passing exactly through a vertex is counted once, not twice.
        const double y_lo = std::min(y1, y2);
        const double y_hi = std::max(y1, y2);
        if (y_sweep < y_lo - kEps || y_sweep >= y_hi - kEps) continue;

        if (std::abs(y2 - y1) < kEps) continue;  // horizontal edge

        const double t = (y_sweep - y1) / (y2 - y1);
        const double x = a.x + t * (b.x - a.x);
        xs.push_back(x);
    }
    std::sort(xs.begin(), xs.end());
    return xs;
}

// Shrink the polygon inward by `inset` meters. This is a very simple
// implementation that offsets each edge along its inward normal and
// intersects consecutive offset edges. It assumes the polygon is simple
// and (mostly) convex. For complex concave polygons this may yield
// artifacts — users should prefer `boundary_inset_m = 0` and rely on
// the local planner's obstacle avoidance for safety margin.
Polygon2D inset_polygon(const Polygon2D& polygon, double inset) {
    if (inset <= 0.0 || polygon.size() < 3) return polygon;

    const bool ccw = polygon_signed_area(polygon) > 0.0;
    const std::size_t n = polygon.size();

    // Offset each edge by `inset` along the inward normal.
    struct Line {
        Point2D p;  // a point on the line
        Point2D d;  // direction vector (not necessarily unit)
    };
    std::vector<Line> lines;
    lines.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Point2D& a = polygon[i];
        const Point2D& b = polygon[(i + 1) % n];
        double ex = b.x - a.x;
        double ey = b.y - a.y;
        double len = std::hypot(ex, ey);
        if (len < kEps) continue;
        // Inward normal: rotate edge 90° towards the interior.
        double nx = ccw ? -ey / len : ey / len;
        double ny = ccw ? ex / len : -ex / len;
        lines.push_back({{a.x + nx * inset, a.y + ny * inset}, {ex, ey}});
    }
    if (lines.size() < 3) return polygon;

    // Intersect consecutive offset lines to get the new polygon vertices.
    Polygon2D out;
    out.reserve(lines.size());
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const Line& l1 = lines[i];
        const Line& l2 = lines[(i + 1) % lines.size()];
        // Solve l1.p + t * l1.d = l2.p + u * l2.d
        const double denom = l1.d.x * l2.d.y - l1.d.y * l2.d.x;
        if (std::abs(denom) < kEps) {
            // Parallel edges — skip; use the endpoint of l1.
            out.push_back({l1.p.x + l1.d.x, l1.p.y + l1.d.y});
            continue;
        }
        const double dx = l2.p.x - l1.p.x;
        const double dy = l2.p.y - l1.p.y;
        const double t = (dx * l2.d.y - dy * l2.d.x) / denom;
        out.push_back({l1.p.x + t * l1.d.x, l1.p.y + t * l1.d.y});
    }
    return out;
}

double boundary_tol_squared_threshold(const Polygon2D& poly) {
    const BBox box = bbox(poly);
    const double span =
        std::max(box.x_max - box.x_min, box.y_max - box.y_min);
    return std::max(
        100.0 * kCoordEpsAbs * kCoordEpsAbs * std::max(1.0, span), 1e-12);
}

bool point_in_polygon_closed(const Point2D& p, const Polygon2D& poly) {
    const std::size_t n = poly.size();
    if (n < 3) return false;

    const BBox box = bbox(poly);
    const double span = std::max(box.x_max - box.x_min, box.y_max - box.y_min);
    const double boundary_tol =
        std::max(5.0 * kCoordEpsAbs * std::max(1.0, span / 1024.0), 5e-7);
    const double boundary_tol_sq = boundary_tol * boundary_tol;

    for (std::size_t i = 0; i < n; ++i) {
        const Point2D& a = poly[i];
        const Point2D& b = poly[(i + 1) % n];
        if (point_on_closed_segment_tol(p, a, b)) return true;
        if (point_segment_distance_squared(p, a, b) <= boundary_tol_sq) return true;
    }

    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point2D& pi = poly[i];
        const Point2D& pj = poly[j];
        if (std::abs(pi.y - pj.y) < kCoordEpsAbs &&
            std::abs(p.y - pi.y) < kCoordEpsAbs) {
            continue;  // ignore almost-horizontal grazing at this latitude
        }
        const bool intersect = ((pi.y > p.y) != (pj.y > p.y)) &&
                               (p.x < (pj.x - pi.x) * (double(p.y) - pi.y) /
                                                 ((pj.y - pi.y) + kCoordEpsAbs) +
                                    pi.x);
        if (intersect) inside = !inside;
    }
    return inside;
}

std::string format_xy(const Point2D& point) {
    std::ostringstream out;
    out << "(" << point.x << ", " << point.y << ")";
    return out.str();
}

Point2D waypoint_xy(const Waypoint& waypoint) {
    return {waypoint.x, waypoint.y};
}

void add_boundary_intersection_params(const Point2D& start, const Point2D& end,
                                      const Point2D& edge_start,
                                      const Point2D& edge_end,
                                      std::vector<double>& params) {
    const Point2D r{end.x - start.x, end.y - start.y};
    const Point2D s{edge_end.x - edge_start.x, edge_end.y - edge_start.y};
    const Point2D edge_delta{edge_start.x - start.x,
                             edge_start.y - start.y};
    const double denominator = r.x * s.y - r.y * s.x;

    auto add_if_on_segment = [&](double t) {
        if (t >= -kCoordEpsAbs && t <= 1.0 + kCoordEpsAbs) {
            params.push_back(std::clamp(t, 0.0, 1.0));
        }
    };

    if (std::fabs(denominator) > kCoordEpsAbs) {
        const double t =
            (edge_delta.x * s.y - edge_delta.y * s.x) / denominator;
        const double u =
            (edge_delta.x * r.y - edge_delta.y * r.x) / denominator;
        if (t >= -kCoordEpsAbs && t <= 1.0 + kCoordEpsAbs &&
            u >= -kCoordEpsAbs && u <= 1.0 + kCoordEpsAbs) {
            add_if_on_segment(t);
        }
        return;
    }

    if (std::fabs(edge_delta.x * r.y - edge_delta.y * r.x) > kCoordEpsAbs)
        return;

    const double r_len_sq = r.x * r.x + r.y * r.y;
    if (r_len_sq < kCoordEpsAbs) return;

    const Point2D from_edge_start{edge_start.x - start.x,
                                  edge_start.y - start.y};
    const Point2D from_edge_end{edge_end.x - start.x,
                                edge_end.y - start.y};
    add_if_on_segment((from_edge_start.x * r.x + from_edge_start.y * r.y) /
                      r_len_sq);
    add_if_on_segment((from_edge_end.x * r.x + from_edge_end.y * r.y) /
                      r_len_sq);
}

std::string segment_containment_error(const Point2D& start,
                                      const Point2D& end,
                                      const Polygon2D& polygon,
                                      std::size_t segment_index) {
    std::vector<double> params{0.0, 1.0};
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        add_boundary_intersection_params(start, end, polygon[i],
                                         polygon[(i + 1) % polygon.size()],
                                         params);
    }

    std::sort(params.begin(), params.end());
    params.erase(
        std::unique(params.begin(), params.end(),
                    [](double a, double b) {
                        return std::fabs(a - b) < kCoordEpsAbs;
                    }),
        params.end());

    for (std::size_t i = 1; i < params.size(); ++i) {
        if (params[i] - params[i - 1] < kCoordEpsAbs) continue;
        const double t = 0.5 * (params[i - 1] + params[i]);
        const Point2D midpoint{start.x + t * (end.x - start.x),
                               start.y + t * (end.y - start.y)};
        if (!point_in_polygon_closed(midpoint, polygon)) {
            return "segment " + std::to_string(segment_index - 1) + "->" +
                   std::to_string(segment_index) + " leaves polygon near " +
                   format_xy(midpoint);
        }
    }

    return "";
}

bool chord_fully_inside_polygon(const Point2D& a, const Point2D& b,
                                const Polygon2D& poly, double sample_spacing) {
    if (!point_in_polygon_closed(a, poly) || !point_in_polygon_closed(b, poly))
        return false;
    const double len = hypot_len(a, b);
    if (len < boundary_tol_squared_threshold(poly))
        return point_in_polygon_closed({(a.x + b.x) * 0.5, (a.y + b.y) * 0.5},
                                       poly);

    const int divisions =
        static_cast<int>(std::ceil(len / std::max(sample_spacing, 1e-6))) + 1;
    const int steps = std::clamp(divisions, 8, 2048);
    for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        Point2D q{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
        if (!point_in_polygon_closed(q, poly)) return false;
    }
    return true;
}

// Shortest 2D path inside a simple polygon using a visibility graph over
// polygon vertices plus the two endpoints. Used to replace inter-row
// shortcuts that would leave a concave polygon.
std::optional<std::vector<Point2D>> shortest_path_in_polygon(
    const Point2D& from,
    const Point2D& to,
    const Polygon2D& poly,
    double sample_spacing) {
    if (!point_in_polygon_closed(from, poly) ||
        !point_in_polygon_closed(to, poly))
        return std::nullopt;
    if (hypot2(from, to) < boundary_tol_squared_threshold(poly))
        return std::vector<Point2D>{from, to};
    if (chord_fully_inside_polygon(from, to, poly, sample_spacing))
        return std::vector<Point2D>{from, to};

    std::vector<Point2D> nodes;
    nodes.reserve(poly.size() + 2);
    for (const auto& v : poly) nodes.push_back(v);
    nodes.push_back(from);
    nodes.push_back(to);
    const std::size_t nv = nodes.size();
    const std::size_t i_from = nv - 2;
    const std::size_t i_to = nv - 1;

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(nv, INF);
    std::vector<int> prev(static_cast<int>(nv), -1);
    using QItem = std::pair<double, std::size_t>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    dist[i_from] = 0.0;
    pq.push({0.0, i_from});

    auto try_relax = [&](std::size_t u, std::size_t v) {
        const double du = hypot_len(nodes[u], nodes[v]);
        if (!chord_fully_inside_polygon(nodes[u], nodes[v], poly,
                                         sample_spacing))
            return;
        const double cand = dist[u] + du;
        if (cand + kEps < dist[v]) {
            dist[v] = cand;
            prev[v] = static_cast<int>(u);
            pq.push({cand, v});
        }
    };

    while (!pq.empty()) {
        auto [du, u] = pq.top();
        pq.pop();
        if (du > dist[u] + kEps) continue;
        if (u == i_to) break;
        for (std::size_t v = 0; v < nv; ++v) {
            if (v == u) continue;
            try_relax(u, v);
        }
    }

    if (!(dist[i_to] < INF)) {
        // Sampling-based visibility occasionally misses portals; fall back to
        // chains through polygon vertices before giving up entirely.
        double best_one = INF;
        std::size_t best_k_one = poly.size();
        for (std::size_t k = 0; k < poly.size(); ++k) {
            const Point2D& vk = poly[k];
            if (!chord_fully_inside_polygon(from, vk, poly, sample_spacing))
                continue;
            if (!chord_fully_inside_polygon(vk, to, poly, sample_spacing))
                continue;
            const double cost = hypot_len(from, vk) + hypot_len(vk, to);
            if (cost < best_one) {
                best_one = cost;
                best_k_one = k;
            }
        }

        double best_two = INF;
        std::size_t best_k2 = poly.size();
        std::size_t best_m2 = poly.size();
        for (std::size_t k = 0; k < poly.size(); ++k) {
            for (std::size_t m = 0; m < poly.size(); ++m) {
                const Point2D& vk = poly[k];
                const Point2D& vm = poly[m];
                if (!chord_fully_inside_polygon(from, vk, poly,
                                                sample_spacing) ||
                    !chord_fully_inside_polygon(vk, vm, poly, sample_spacing) ||
                    !chord_fully_inside_polygon(vm, to, poly, sample_spacing))
                    continue;
                const double cost = hypot_len(from, vk) + hypot_len(vk, vm) +
                                      hypot_len(vm, to);
                if (cost < best_two) {
                    best_two = cost;
                    best_k2 = k;
                    best_m2 = m;
                }
            }
        }

        if (best_one < INF &&
            (best_two >= INF || best_one <= best_two + kEps)) {
            return std::vector<Point2D>{from, poly[best_k_one], to};
        }
        if (best_two < INF) {
            return std::vector<Point2D>{from, poly[best_k2], poly[best_m2], to};
        }
        return std::nullopt;
    }

    std::vector<Point2D> out_rev;
    for (std::size_t cur = i_to;;) {
        out_rev.push_back(nodes[cur]);
        const int pv = prev[cur];
        if (pv < 0) break;
        cur = static_cast<std::size_t>(pv);
    }
    std::reverse(out_rev.begin(), out_rev.end());
    // Remove duplicate successive points (epsilon).
    std::vector<Point2D> cleaned;
    cleaned.reserve(out_rev.size());
    for (const auto& q : out_rev) {
        if (cleaned.empty() ||
            hypot2(cleaned.back(), q) > boundary_tol_squared_threshold(poly))
            cleaned.push_back(q);
    }
    return cleaned;
}

bool append_bridged(std::vector<Point2D>* path_ptr,
                    const Point2D& next,
                    const Polygon2D& poly,
                    double sample_spacing) {
    auto& path = *path_ptr;
    if (path.empty()) {
        path.push_back(next);
        return point_in_polygon_closed(next, poly);
    }
    const Point2D& prev = path.back();
    if (hypot2(prev, next) < boundary_tol_squared_threshold(poly))
        return true;
    std::optional<std::vector<Point2D>> seg;
    if (chord_fully_inside_polygon(prev, next, poly, sample_spacing))
        seg = std::vector<Point2D>{prev, next};
    else
        seg = shortest_path_in_polygon(prev, next, poly, sample_spacing);
    if (!seg || seg->size() < 2) return false;
    const double dedupe2 = boundary_tol_squared_threshold(poly);
    for (std::size_t i = 1; i < seg->size(); ++i) {
        if (hypot2(path.back(), (*seg)[i]) > dedupe2) path.push_back((*seg)[i]);
    }
    return true;
}

// Yaw for travelling prev→p under orientation_mode (REP-103 yaw about +Z).
double segment_yaw_rad(double tangent_rad,
                       uint8_t orientation_mode,
                       double heading_rad,
                       double fixed_rad) {
    switch (orientation_mode) {
        case CoverageParams::kOrientAlignedToSweep: {
            const double ctx = std::cos(tangent_rad);
            const double sty = std::sin(tangent_rad);
            const double ch = std::cos(heading_rad);
            const double sh = std::sin(heading_rad);
            const bool forward = ch * ctx + sh * sty >= 0.0;
            double y = forward ? heading_rad : heading_rad + M_PI;
            if (y > M_PI)
                y -= 2.0 * M_PI;
            else if (y < -M_PI)
                y += 2.0 * M_PI;
            return y;
        }
        case CoverageParams::kOrientFixed:
            return fixed_rad;
        default:
            return tangent_rad;
    }
}

}  // namespace

Point2D rotate(const Point2D& p, double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {p.x * c - p.y * s, p.x * s + p.y * c};
}

double polygon_signed_area(const Polygon2D& polygon) {
    const std::size_t n = polygon.size();
    if (n < 3) return 0.0;
    double a = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Point2D& p = polygon[i];
        const Point2D& q = polygon[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * a;
}

double path_length(const std::vector<Waypoint>& path) {
    double len = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const double dx = path[i].x - path[i - 1].x;
        const double dy = path[i].y - path[i - 1].y;
        const double dz = path[i].z - path[i - 1].z;
        len += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return len;
}

std::string path_containment_error(const std::vector<Waypoint>& path,
                                   const Polygon2D& polygon) {
    if (polygon.size() < 3) {
        return "coverage polygon has fewer than 3 vertices";
    }

    for (std::size_t i = 0; i < path.size(); ++i) {
        const Point2D point = waypoint_xy(path[i]);
        if (!point_in_polygon_closed(point, polygon)) {
            return "waypoint " + std::to_string(i) +
                   " is outside polygon at " + format_xy(point);
        }
    }

    for (std::size_t i = 1; i < path.size(); ++i) {
        const std::string error = segment_containment_error(
            waypoint_xy(path[i - 1]), waypoint_xy(path[i]), polygon, i);
        if (!error.empty()) return error;
    }

    return "";
}

std::optional<std::vector<Waypoint>> generate_coverage_path(
    const Polygon2D& polygon_in,
    const Point2D& start_xy,
    const CoverageParams& params) {
    if (polygon_in.size() < 3) return std::nullopt;
    if (params.line_spacing_m <= 0.0) return std::nullopt;
    if (std::abs(polygon_signed_area(polygon_in)) < kEps) return std::nullopt;

    // Step 1: optional safety inset.
    Polygon2D polygon = inset_polygon(polygon_in, params.boundary_inset_m);
    if (polygon.size() < 3) return std::nullopt;

    // Step 2: rotate the polygon into the sweep frame so passes are
    // along +X. Sweep direction = `heading` → rotate polygon by -heading.
    const double heading_rad = params.heading_deg * M_PI / 180.0;
    const Polygon2D rotated = rotate_polygon(polygon, -heading_rad);

    // Step 3: for each y = y_min + k*spacing, compute polygon intersections
    // and emit alternating-direction waypoint rows.
    const BBox b = bbox(rotated);
    if (b.y_max - b.y_min < kEps) return std::nullopt;

    std::vector<std::vector<Point2D>> rows;  // each row in rotated frame
    bool flip = false;
    // Offset the first sweep half a spacing inside the polygon so the
    // swept strips cover the full area.
    const double y0 = b.y_min + 0.5 * params.line_spacing_m;
    for (double y = y0; y <= b.y_max - 0.5 * params.line_spacing_m + kEps;
         y += params.line_spacing_m) {
        auto xs = sweep_line_intersections(rotated, y);
        if (xs.size() < 2) continue;

        // Pair up intersections into segments (handles non-convex polygons).
        std::vector<Point2D> row;
        for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
            Point2D l{xs[i], y};
            Point2D r{xs[i + 1], y};
            if (flip) std::swap(l, r);
            row.push_back(l);
            row.push_back(r);
        }
        if (!row.empty()) rows.push_back(std::move(row));
        flip = !flip;
    }
    if (rows.empty()) return std::nullopt;

    // Step 4: optionally pick which end of the overall sweep to enter
    // from so we start near the robot's current position. We compare
    // the start pose (transformed into the rotated frame) to the first
    // and last row endpoints, and reverse the whole sweep if that's
    // closer.
    if (params.start_from_nearest) {
        const Point2D start_rot = rotate(start_xy, -heading_rad);
        auto dist2 = [](const Point2D& a, const Point2D& b) {
            const double dx = a.x - b.x;
            const double dy = a.y - b.y;
            return dx * dx + dy * dy;
        };
        const Point2D first = rows.front().front();
        const Point2D last = rows.back().back();
        if (dist2(start_rot, last) < dist2(start_rot, first)) {
            std::reverse(rows.begin(), rows.end());
            for (auto& row : rows) std::reverse(row.begin(), row.end());
        }
    }

    // Step 5: stitch sweep rows inside the rotated polygon (concave-safe
    // row-to-row bridging), rotate to world, and apply orientation policy.
    const double bx = b.x_max - b.x_min;
    const double by = b.y_max - b.y_min;
    const double diag_bbox =
        std::hypot(std::max(bx, kEps), std::max(by, kEps));
    const double chord_spacing = std::clamp(
        std::min(params.line_spacing_m * 0.2, diag_bbox / 500.0), 0.05, 2.0);

    std::vector<Point2D> rot_path;
    rot_path.reserve(rows.size() * 8);
    for (const auto& row : rows) {
        for (const auto& pr : row) {
            if (!append_bridged(&rot_path, pr, rotated, chord_spacing))
                return std::nullopt;
        }
    }

    std::vector<Waypoint> path;
    path.reserve(rot_path.size());
    Point2D prev{0.0, 0.0};
    bool has_prev = false;
    const double fixed_rad = params.orientation_deg * M_PI / 180.0;
    for (const auto& p_rot : rot_path) {
        const Point2D p = rotate(p_rot, heading_rad);
        double yaw = 0.0;
        if (has_prev) {
            const double tangent_rad =
                std::atan2(p.y - prev.y, p.x - prev.x);
            yaw = segment_yaw_rad(tangent_rad, params.orientation_mode,
                                  heading_rad, fixed_rad);
            path.back().yaw = yaw;
        }
        path.push_back({p.x, p.y, params.altitude_m, yaw});
        prev = p;
        has_prev = true;
    }
    return path;
}

}  // namespace coverage_planner
