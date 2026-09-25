// =============================================================================
//  mtl/trajectory/lateral_coverage.hpp
//
//  Append fly-by lanes so every cell comes ABEAM.
//
//  WHY THIS EXISTS
//  ---------------
//  With a SINGLE-AXIS gimbal the boresight can only be swung left/right, never
//  forward/back (forward pointing costs pitch, i.e. altitude).  So a cell centre
//  can only be observed at the instant its ALONG-TRACK offset crosses the swept
//  line, and only if the cross-track offset at that instant is inside the sensor
//  reach.
//
//  The Dubins route stops dead at the last macro-centroid.  Every cell of the
//  final cluster lying ahead of (or off the side of) that last heading therefore
//  NEVER has its along-track offset cross zero: the aircraft simply runs out of
//  trajectory first.  The scheduler reports these as "passed within reach but
//  never abeam (fix the ground track)" - and it is right, the fix is the ground
//  track, not the gimbal.
//
//  WHAT IT DOES
//  ------------
//  Keeps flying.  The aircraft holds the heading it ended the route on and
//  continues in a straight line for ExtensionParams::extendDist metres - that is
//  the whole of it.  No lanes, no orbits, no search: the cells left over sit
//  ahead of the last leg, so their offset along it falls and then rises again as
//  the aircraft flies on, and they cross the swept line on the way past.
//
//  The length is a SET DISTANCE, not something derived from the cells, so the
//  extension costs the same predictable amount on every sortie and the endurance
//  budget left for the route itself never moves around underneath the planner.
//
//  The abeam audit still runs, but only to REPORT which centres the nominal
//  track abandoned (generateTrajectories aims the advisory boresight at those)
//  and which the run-out rescued.  It does not set the length.  The one thing it
//  decides is whether to append anything at all: if the nominal route already
//  brings every centre abeam, the run-out would be wasted budget, so nothing is
//  added.
//
//  A TILTED MOUNT shifts the swept line h*tan(tau) along track, which only makes
//  the run-out cheaper - the line is already ahead of the aircraft, so fewer
//  centres are stranded to begin with.  Pass ExtensionParams::alongOffset and the
//  audit shifts with it.
// =============================================================================
#ifndef MTL_TRAJECTORY_LATERAL_COVERAGE_HPP
#define MTL_TRAJECTORY_LATERAL_COVERAGE_HPP

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::trajectory {

struct ExtensionResult {
    Path2         extXY;  ///< points to APPEND to the nominal track (empty if none needed).
                          ///  The first sample already steps away from the track's end.
    ExtensionInfo info;
};

/// @param droneTraj2D   K-by-2 nominal ground track
/// @param cellCenters   M-by-2 centres this agent must observe
/// @param droneAltitude [m] cruise altitude, sets the geometric sensor reach
ExtensionResult extendTrajForLateralCoverage(const Path2& droneTraj2D, const Path2& cellCenters,
                                             double droneAltitude, double minTurnRadius,
                                             const ExtensionParams& opts);

}  // namespace mtl::trajectory

#endif  // MTL_TRAJECTORY_LATERAL_COVERAGE_HPP
