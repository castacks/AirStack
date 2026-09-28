"""WGS84 georeference of the scene: scene metres <-> latitude / longitude / ellipsoidal height.

    ~/.venvs/recon/bin/python georef.py X Y [Z]                 # scene -> lat lon h
    ~/.venvs/recon/bin/python georef.py --inverse LAT LON [H]   # lat lon h -> scene
    ~/.venvs/recon/bin/python georef.py --write                 # data/recon/georeference.json + geofence.geojson, and stamp
                                                                # the stages' customLayerData["georeference"] (assemble_scene.py
                                                                # and autumn.py stamp them on every build too)

The scene frame is a local east-north-up (ENU) tangent plane on the WGS84 ellipsoid (EPSG:4326 / 4979), at the
Blosm import origin stored in the blend (scene["lat"], scene["lon"]): x = east, y = north, z = up, metres, origin on
the ellipsoid surface (h = 0). So z IS the WGS84 ellipsoidal height, give or take the tangent plane's drop
(d^2 / 2R: 3 cm at 600 m, handled exactly here). NAVD88 height = h + 26.75 m here (NOAA GEOID18).

How that was established (2026-09-27), because the blend offers two candidates:
  * the blend's OSM layers are Blosm's spherical transverse Mercator (R = 6378137) at the same origin: live OSM
    building nodes projected that way land on them to 0.000 m. The tiles are NOT in that frame -- it stretches
    north-south by 0.41% against the ellipsoid (2.3 m at the fence's south edge);
  * the Google tiles are ENU: 25 tile-measured building centres (lod1_buildings.py) against live OSM outlines fit
    a y scale of -0.07% under ENU vs -0.48% under the sphere, leaving a constant ~1 m offset (OSM tracing) and
    0.3-0.6 m scatter;
  * heights: the tile bare earth (ground.py dtm) against USGS 3DEP lidar (EPQS, NAVD88) + GEOID18 at 8 points:
    z - h = -0.77 m mean, 0.6 m sd. Blosm's scene["height_offset"] (54.6 m) is NOT applied to the tiles.
Absolute accuracy is Google's photogrammetry, ~1 m horizontally and vertically.
"""
import json, sys
import numpy as np

LAT0, LON0, H0 = 30.57891082763672, -96.35235214233398, 0.0      # blend scene["lat"], scene["lon"]; ENU origin on the ellipsoid
GEOID_N = -26.75                                                  # GEOID18 here: NAVD88 = h - GEOID_N
FENCE = (-205.0, 355.0, -615.0, -20.0)                            # x0, x1, y0, y1: the built window (ortho_site.json)
A, F = 6378137.0, 1 / 298.257223563; E2 = F * (2 - F)

def _ecef(lat, lon, h):
    la, lo = np.radians(lat), np.radians(lon); n = A / np.sqrt(1 - E2 * np.sin(la) ** 2)
    return np.stack([(n + h) * np.cos(la) * np.cos(lo), (n + h) * np.cos(la) * np.sin(lo), (n * (1 - E2) + h) * np.sin(la)], -1)

_la, _lo = np.radians(LAT0), np.radians(LON0)
_ROT = np.array([[-np.sin(_lo), np.cos(_lo), 0.0],                                   # rows: east, north, up in ECEF
                 [-np.sin(_la) * np.cos(_lo), -np.sin(_la) * np.sin(_lo), np.cos(_la)],
                 [np.cos(_la) * np.cos(_lo), np.cos(_la) * np.sin(_lo), np.sin(_la)]])
_O = _ecef(LAT0, LON0, H0)

def scene_to_wgs84(x, y, z=0.0):
    """scene metres -> (lat deg, lon deg, WGS84 ellipsoidal height m); arrays broadcast"""
    X = _O + np.stack(np.broadcast_arrays(x, y, z), -1).astype(float) @ _ROT
    p = np.hypot(X[..., 0], X[..., 1]); lon = np.arctan2(X[..., 1], X[..., 0]); lat = np.arctan2(X[..., 2], p * (1 - E2))
    for _ in range(5):
        n = A / np.sqrt(1 - E2 * np.sin(lat) ** 2); h = p / np.cos(lat) - n
        lat = np.arctan2(X[..., 2], p * (1 - E2 * n / (n + h)))
    n = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.degrees(lat), np.degrees(lon), p / np.cos(lat) - n

def wgs84_to_scene(lat, lon, h=0.0):
    """(lat deg, lon deg, ellipsoidal height m) -> scene x, y, z"""
    v = (_ecef(*np.broadcast_arrays(lat, lon, h)) - _O) @ _ROT.T
    return v[..., 0], v[..., 1], v[..., 2]

def metadata():
    """the georeference as a dict: USD customLayerData and georeference.json"""
    x0, x1, y0, y1 = FENCE
    corners = [[float(v) for v in scene_to_wgs84(x, y)[:2]] for x, y in ((x0, y1), (x1, y1), (x1, y0), (x0, y0))]
    return {"crs": "EPSG:4979 (WGS84 geographic 3D)", "frame": "ENU",
            "frame_note": "local east-north-up tangent plane: x east, y north, z up, metres; origin on the WGS84 ellipsoid, "
                          "so z is ellipsoidal height (NAVD88 = z + 26.75 m here)",
            "origin_lat_deg": LAT0, "origin_lon_deg": LON0, "origin_height_m": H0, "geoid_undulation_m": GEOID_N,
            "origin_altitude_msl_m": H0 - GEOID_N,                 # e.g. Pegasus set_global_coordinates(lat, lon, this) for PX4 GPS
            "accuracy_m": 1.0, "code": "scene_gen/disaster_city/georef.py",
            "geofence_scene_m": {"x": [x0, x1], "y": [y0, y1]},
            "geofence_latlon_deg": corners}                         # NW, NE, SE, SW

def stamp(stage_or_layer):
    """put the georeference in a stage's root layer metadata (customLayerData["georeference"]; lists as JSON strings)"""
    layer = stage_or_layer.GetRootLayer() if hasattr(stage_or_layer, "GetRootLayer") else stage_or_layer
    d = dict(layer.customLayerData)
    d["georeference"] = {k: (json.dumps(v) if isinstance(v, (list, dict)) else v) for k, v in metadata().items()}
    layer.customLayerData = d

if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "--write":
        from _paths import R
        m = metadata(); json.dump(m, open(R / "georeference.json", "w"), indent=1)
        ring = [[lon, lat] for lat, lon in m["geofence_latlon_deg"]]
        json.dump({"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {"name": "disaster_city_geofence",
                   "scene_m": m["geofence_scene_m"]}, "geometry": {"type": "Polygon", "coordinates": [ring + ring[:1]]}}]},
                  open(R / "geofence.geojson", "w"), indent=1)
        from pxr import Sdf
        for f in ("disaster_city.usda", "disaster_city_autumn.usda", "disaster_city_raw_tiles.usda"):
            if (R / f).exists(): layer = Sdf.Layer.FindOrOpen(str(R / f)); stamp(layer); layer.Save()
        print(f"wrote {R / 'georeference.json'} and {R / 'geofence.geojson'}; stamped the stages")
    elif a and a[0] == "--inverse":
        print(*(f"{float(v):.3f}" for v in wgs84_to_scene(*map(float, a[1:]))))
    elif a:
        lat, lon, h = scene_to_wgs84(*map(float, a))
        print(f"{float(lat):.8f} {float(lon):.8f} {float(h):.3f}")
    else:
        print(__doc__)
