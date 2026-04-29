// Copyright (c) 2026 Carnegie Mellon University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace coverage_planner {

// A 2D point used internally by the planner.
struct Point2D {
    double x{0.0};
    double y{0.0};
};

// A 3D waypoint (x, y, z) with a yaw heading (rad).
struct Waypoint {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
};

// Polygon: ordered list of vertices. The polygon is treated as closed
// (last vertex connects back to first). Winding order is irrelevant.
using Polygon2D = std::vector<Point2D>;

struct CoverageParams {
    // Target flight altitude (absolute world z). The planner does not
    // perform AGL→world conversion — the node picks the altitude from
    // the goal's min/max AGL bounds and passes world z in here.
    double altitude_m{5.0};

    // Perpendicular distance between successive sweep lines (m).
    // Must be > 0. Typical values are a fraction of the sensor FOV.
    double line_spacing_m{5.0};

    // Direction of the sweep lines (degrees, CCW from +X / east).
    // Successive passes are perpendicular to this direction.
    double heading_deg{0.0};

    // If true, the first waypoint is the polygon entry point closest
    // to the start pose (reduces deadhead flight). If false, the
    // sweep always starts at the minimum-v side.
    bool start_from_nearest{true};

    // If > 0, inset the polygon by this amount before sweeping, to
    // keep the UAV a safe distance from the boundary (m).
    double boundary_inset_m{0.0};
};

// Generate a boustrophedon (lawn-mower) coverage path over `polygon`.
//
// Returns an empty optional if the polygon is degenerate (fewer than 3
// vertices, zero area, or no valid sweep line hits the polygon).
//
// `start_xy` is used for choosing the closer entry corner when
// `params.start_from_nearest` is true. Ignored otherwise.
std::optional<std::vector<Waypoint>> generate_coverage_path(
    const Polygon2D& polygon,
    const Point2D& start_xy,
    const CoverageParams& params);

// ---- Utilities exposed for unit testing -------------------------------------

// Signed area of a polygon (shoelace). Positive if CCW.
double polygon_signed_area(const Polygon2D& polygon);

// Total path length (sum of consecutive Euclidean distances).
double path_length(const std::vector<Waypoint>& path);

// Rotate `p` by `angle_rad` about the origin.
Point2D rotate(const Point2D& p, double angle_rad);

}  // namespace coverage_planner
