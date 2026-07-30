"""
Quick test: does the constellation load and produce sensible results?
Run with: ISAACSIM_PYTHON test_constellation.py
         or: python3 test_constellation.py   (if sgp4/pymap3d installed system-wide)
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gps_degradation_staging.constellation import ConstellationModel

# Load the TLE file (path relative to simulation/isaac-sim/)
model = ConstellationModel("data/gps_ops.tle")

# Pittsburgh coordinates (AirStack's test location)
LAT = 40.4422
LON = -79.9430
ALT = 100.0    # 100 metres altitude

# Get visible satellites (sim_time_s=0 → Jan 1 2025 00:00 UTC)
sats = model.get_visible_satellites(LAT, LON, ALT, sim_time_s=0.0)

print(f"\nVisible satellites from Pittsburgh at {ALT}m altitude:")
print(f"Total: {len(sats)} satellites above {model.ELEVATION_MASK_DEG}° elevation\n")
print(f"{'Name':<32} {'Elevation':>10} {'Azimuth':>10}")
print("-" * 55)
for s in sorted(sats, key=lambda x: -x.elevation_deg):
    print(f"{s.name:<32} {s.elevation_deg:>9.1f}°  {s.azimuth_deg:>9.1f}°")

if sats:
    print(f"\nExample ENU unit vector for first (highest) satellite:")
    s0 = sorted(sats, key=lambda x: -x.elevation_deg)[0]
    print(f"  {s0.name}: {s0.enu_unit}")
    print(f"  (East={s0.enu_unit[0]:.4f}, North={s0.enu_unit[1]:.4f}, Up={s0.enu_unit[2]:.4f})")
    print(f"\nExpect visible GNSS satellites. If 0, check sgp4/pymap3d and TLE files.")
else:
    print("\nWARNING: 0 satellites visible — check packages and TLE file path.")
    sys.exit(1)
