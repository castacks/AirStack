"""
constellation.py

Loads real GNSS satellite TLE data from files and computes satellite positions
(azimuth, elevation, ENU unit vector) as seen from the drone.

Uses sgp4 + pymap3d with TLE data files in data/ (offline at runtime).

Download the TLE file once:
    wget "https://celestrak.org/NORAD/elements/gp.php?GROUP=gps-ops&FORMAT=tle" \
         -O simulation/isaac-sim/data/gps_ops.tle
"""

import math
from dataclasses import dataclass
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import List, Optional

import numpy as np

# ── optional deps ──────────────────────────────────────────────────────────
_SGP4_OK = False
_PM_OK   = False
try:
    from sgp4.api import Satrec, jday
    _SGP4_OK = True
except ImportError:
    pass

try:
    import pymap3d as pm
    _PM_OK = True
except ImportError:
    pass

# Jan 1 2025 00:00 UTC — fixed reference so sim_time_s is reproducible
_REFERENCE_UTC = datetime(2025, 1, 1, 0, 0, 0, tzinfo=timezone.utc)

_DEFAULT_TLE = Path(__file__).parent.parent / "data" / "gps_ops.tle"


# ── data class ─────────────────────────────────────────────────────────────

@dataclass
class SatelliteView:
    sat_id: int           # index in TLE file (0-based)
    name: str             # satellite name, e.g. "GPS BIIR-2  (PRN 13)"
    elevation_deg: float  # angle above horizon; 0 = horizon, 90 = zenith
    azimuth_deg: float    # compass direction; 0 = North, 90 = East
    enu_unit: np.ndarray  # unit vector [East, North, Up] toward satellite
                          # used as PhysX raycast direction and DOP row


# ── TLE constellation ──────────────────────────────────────────────────────

class _TLEConstellation:
    """Propagates real GNSS orbits from a TLE file using sgp4."""

    def __init__(self, tle_path: Path):
        self._satellites = self._parse(tle_path)

    def _parse(self, path: Path) -> list:
        lines = [l.strip() for l in path.read_text().splitlines() if l.strip()]
        sats = []
        # TLE file is in groups of 3: name, line1 (starts '1'), line2 (starts '2')
        for i in range(0, len(lines) - 2, 3):
            name, line1, line2 = lines[i], lines[i + 1], lines[i + 2]
            if line1.startswith('1') and line2.startswith('2'):
                sats.append((name, Satrec.twoline2rv(line1, line2)))
        return sats

    def __len__(self):
        return len(self._satellites)

    def get_visible(
        self,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        elevation_mask_deg: float,
        now: datetime,
    ) -> List[SatelliteView]:
        # Julian date (two-part for precision)
        jd, fr = jday(
            now.year, now.month, now.day,
            now.hour, now.minute, now.second + now.microsecond / 1e6,
        )

        # GMST (Greenwich Mean Sidereal Time) for TEME → ECEF rotation.
        # Formula from the IAU — accurate to ~0.1 arcsec over several centuries.
        # t_days: days since J2000.0 epoch (noon Jan 1 2000 UTC)
        t_days = (jd + fr) - 2451545.0
        gmst   = (6.697374558 + 2400.0130654 * (t_days / 36525.0)) * math.pi / 12.0
        gmst   = gmst % (2 * math.pi)
        cos_g, sin_g = math.cos(gmst), math.sin(gmst)

        visible = []
        for sat_id, (name, sat) in enumerate(self._satellites):
            e, r, _ = sat.sgp4(jd, fr)
            if e != 0:
                continue  # propagation error (decayed sat, bad TLE, etc.)

            # r is in km → metres
            rx, ry, rz = r[0] * 1e3, r[1] * 1e3, r[2] * 1e3

            # TEME → ECEF: rotate by GMST around Z axis
            # [ cos g   sin g   0 ]
            # [-sin g   cos g   0 ]
            # [  0       0      1 ]
            ex =  cos_g * rx + sin_g * ry
            ey = -sin_g * rx + cos_g * ry
            ez =  rz

            # ECEF → azimuth/elevation from drone position
            try:
                az, el, _ = pm.ecef2aer(ex, ey, ez, lat_deg, lon_deg, alt_m)
            except Exception:
                continue

            if float(el) < elevation_mask_deg:
                continue

            visible.append(_make_view(sat_id, name, float(az), float(el)))
        return visible


# ── helpers ────────────────────────────────────────────────────────────────

def _make_view(sat_id: int, name: str, az: float, el: float) -> SatelliteView:
    el_r = math.radians(el)
    az_r = math.radians(az)
    c = math.cos(el_r)
    return SatelliteView(
        sat_id=sat_id,
        name=name,
        elevation_deg=el,
        azimuth_deg=az,
        enu_unit=np.array([c * math.sin(az_r), c * math.cos(az_r), math.sin(el_r)]),
    )


# ── public class ───────────────────────────────────────────────────────────

class ConstellationModel:
    """
    Computes GNSS satellite visibility from drone position.

    Uses real TLE data (sgp4 + pymap3d). Missing dependencies, invalid TLE
    data, and empty propagation results are errors rather than silent
    synthetic-constellation fallbacks.

    Usage:
        model = ConstellationModel("data/gps_ops.tle")
        sats  = model.get_visible_satellites(lat=40.44, lon=-79.94, alt=100.0)
        for s in sats:
            print(s.elevation_deg, s.azimuth_deg)
    """

    # Satellites below this elevation are too close to the horizon — too
    # much atmospheric delay. Standard GPS mask is 5–15°.
    ELEVATION_MASK_DEG = 5.0

    def __init__(self, tle_file_path: str = ""):
        self._tle: Optional[_TLEConstellation] = None
        self._using_tle = False

        path = Path(tle_file_path) if tle_file_path else _DEFAULT_TLE

        if not (_SGP4_OK and _PM_OK):
            missing = [n for n, ok in [("sgp4", _SGP4_OK), ("pymap3d", _PM_OK)] if not ok]
            raise RuntimeError(f"{', '.join(missing)} not installed")
        paths = [
            ("GPS", path),
            ("GLONASS", path.with_name("glonass_ops.tle")),
            ("Galileo", path.with_name("galileo_ops.tle")),
            ("BeiDou", path.with_name("beidou_ops.tle")),
        ]
        for name, tle_path in paths:
            if not tle_path.exists():
                print(f"[ConstellationModel] Warning: TLE file not found: {tle_path}")
                continue
            constellation = _TLEConstellation(tle_path)
            if self._tle is None:
                self._tle = constellation
            else:
                self._tle._satellites.extend(constellation._satellites)
            print(f"[ConstellationModel] Loaded {len(constellation)} {name} satellites from {tle_path}")
        if self._tle is None or len(self._tle) == 0:
            raise ValueError("No valid GNSS satellites found in TLE files")
        self._using_tle = True
        print(f"[ConstellationModel] Loaded {len(self._tle)} total GNSS satellites")

    @property
    def using_tle(self) -> bool:
        return self._using_tle

    def get_visible_satellites(
        self,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        sim_time_s: Optional[float] = None,
    ) -> List[SatelliteView]:
        """
        Return all satellites above the elevation mask.

        Args:
            lat_deg:    drone latitude in degrees
            lon_deg:    drone longitude in degrees
            alt_m:      drone altitude above sea level (metres)
            sim_time_s: seconds since Jan 1 2025 00:00 UTC.
                        Pass Isaac Sim's world.current_time here.
                        None → uses wall-clock UTC (non-reproducible).
        """
        if sim_time_s is None:
            now = datetime.now(timezone.utc)
        else:
            now = _REFERENCE_UTC + timedelta(seconds=float(sim_time_s))

        sats = self._tle.get_visible(lat_deg, lon_deg, alt_m, self.ELEVATION_MASK_DEG, now)
        if not sats:
            raise RuntimeError("TLE propagation produced zero visible GNSS satellites")
        return sats
