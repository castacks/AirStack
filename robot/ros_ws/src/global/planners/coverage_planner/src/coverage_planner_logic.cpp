// Copyright (c) 2026 Carnegie Mellon University
//
// Licensed under the Apache License, Version 2.0 (the "License").
// See coverage_planner_logic.hpp for full license text.

#include "coverage_planner_logic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace coverage_planner {

namespace {

constexpr double kEps = 1e-9;

// Rotate a polygon by angle_rad about the origin.
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

    // Step 5: rotate back to world frame, compute yaw from each segment
    // direction, and emit waypoints.
    std::vector<Waypoint> path;
    path.reserve(rows.size() * 2);
    Point2D prev{0.0, 0.0};
    bool has_prev = false;
    for (const auto& row : rows) {
        for (const auto& p_rot : row) {
            const Point2D p = rotate(p_rot, heading_rad);
            double yaw = 0.0;
            if (has_prev) {
                yaw = std::atan2(p.y - prev.y, p.x - prev.x);
                // Fix up the yaw of the previously pushed waypoint so
                // it points toward the current one.
                path.back().yaw = yaw;
            }
            path.push_back({p.x, p.y, params.altitude_m, yaw});
            prev = p;
            has_prev = true;
        }
    }
    return path;
}

}  // namespace coverage_planner
