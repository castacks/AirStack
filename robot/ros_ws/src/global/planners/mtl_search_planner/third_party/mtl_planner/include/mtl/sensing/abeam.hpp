// =============================================================================
//  mtl/sensing/abeam.hpp
//
//  Can a single-axis gimbal ever see each centre from this ground track?
//
//  The camera sits on a 1-DOF gimbal, so the boresight only swings LEFT/RIGHT.
//  It therefore sweeps ONE cross-track line, and a centre is observable only at
//  an instant where
//    (a) the ALONG-TRACK offset of the centre crosses alongOffset - the
//        along-track stand-off of that line, and
//    (b) the CROSS-TRACK offset at that instant is inside the sensor reach.
//  Along-track pointing beyond that needs pitch, and pitch is the flight-path
//  angle, which is why (a) is a hard geometric gate rather than something the
//  gimbal can trade against.  `alongTol` admits the couple of metres of slide a
//  degree or two of pitch buys, so a centre a hair short of the line is not
//  mistaken for a miss.
//
//  ALONGOFFSET IS THE MOUNT.  At nadir it is zero and (a) is the classic abeam
//  condition: the centre passes beside the aircraft.  With the camera on a
//  bracket tilted by tau the swept line stands off h*tan(tau), so pass
//  alongOffset = h*tan(tau) and (a) becomes "the centre passes h*tan(tau) metres
//  AHEAD" - the same test, shifted.
// =============================================================================
#ifndef MTL_SENSING_ABEAM_HPP
#define MTL_SENSING_ABEAM_HPP

#include <vector>

#include "mtl/types.hpp"

namespace mtl::sensing {

struct AbeamResult {
    std::vector<char> observable;  ///< M flags: the centre is observable somewhere
    VecX              dmin;        ///< M: smallest |along offset - alongOffset| seen while
                                   ///  within cross-track reach (0 on an exact crossing,
                                   ///  Inf if the track never came within reach at all)
    Index nObservable = 0;
};

/// @param track       K-by-2 ground track samples
/// @param centers     M-by-2 points to test (valid-cell centres)
/// @param reach       [m] usable cross-track ground offset of the boresight
/// @param alongTol    [m] along-track slack allowed (0 = exact crossing only)
/// @param alongOffset [m] along-track stand-off of the swept line (0 = nadir)
AbeamResult abeamObservable(const Path2& track, const Path2& centers, double reach,
                            double alongTol = 0.0, double alongOffset = 0.0);

}  // namespace mtl::sensing

#endif  // MTL_SENSING_ABEAM_HPP
