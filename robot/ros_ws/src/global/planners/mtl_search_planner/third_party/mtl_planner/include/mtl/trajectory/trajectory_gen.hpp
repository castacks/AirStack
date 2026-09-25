// =============================================================================
//  mtl/trajectory/trajectory_gen.hpp
//
//  Drone Dubins track + advisory sensor ground path, time-parameterised.
//
//  The drone track is the Dubins path through the macro route, optionally
//  extended with lateral-coverage lanes, resampled at constant speed onto the
//  simulation time step.  Everything downstream is parameterised by ARC LENGTH,
//  so appending to the track is all the extension has to do - the time vector,
//  the nadir default and the micro sweeps all follow automatically.
//
//  The sensor path is ADVISORY: it is what the boresight would trace if it swept
//  each cluster's micro-TSP as the aircraft passed through the cluster's
//  activation bubble.  The single-axis scheduler treats it as a suggestion (only
//  the cell centres are a hard constraint); the multi-axis mode flies it as is.
// =============================================================================
#ifndef MTL_TRAJECTORY_TRAJECTORY_GEN_HPP
#define MTL_TRAJECTORY_TRAJECTORY_GEN_HPP

#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::trajectory {

struct TrajectoryResult {
    Path3         drone;    ///< N-by-3 [x y h]
    Path2         sensor;   ///< N-by-2 advisory boresight ground point
    VecX          time;     ///< N-by-1 [s]
    ExtensionInfo extInfo;
};

struct TrajectoryInputs {
    Path2              macroRoute;      ///< (1+n)-by-2, row 0 = launch point
    Path2              sensorTargets;   ///< S-by-2 cell centres in service order
    std::vector<Index> sensorMacroRow;  ///< S-by-1 route row each centre belongs to
    double             droneAltitude = 300.0;
    double             avgDroneSpeed = 20.0;
    double             dt            = 0.1;
    double             minTurnRadius = 100.0;
    double             maxClusterRadius = 500.0;
    bool               extendForLateralCoverage = false;
};

TrajectoryResult generateTrajectories(const TrajectoryInputs& in, const TrajectoryParams& opts,
                                      const ExtensionParams& extendOpts,
                                      const DubinsParams& dubinsOpts);

}  // namespace mtl::trajectory

#endif  // MTL_TRAJECTORY_TRAJECTORY_GEN_HPP
