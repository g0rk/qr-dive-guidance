# utils/geo_utils.py — Pure math functions for geographic computation.
# No side effects, no project imports, no MAVSDK dependency.

import math

EARTH_RADIUS_M = 6_378_137.0

def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Forward azimuth from point 1 to point 2.
    Returns bearing in degrees [0, 360).
    """
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlon_r = math.radians(lon2 - lon1)

    x = math.sin(dlon_r) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon_r))

    return math.degrees(math.atan2(x, y)) % 360.0

def ground_track_deg(vel_north: float, vel_east: float) -> float:
    """
    Track angle from NED velocity components.
    Returns track in degrees [0, 360).
    """
    return math.degrees(math.atan2(vel_east, vel_north)) % 360.0

def ground_speed_m_s(vel_north: float, vel_east: float) -> float:
    """Horizontal speed magnitude from NED velocity components."""
    return math.hypot(vel_north, vel_east)

def angle_error_deg(target_deg: float, current_deg: float) -> float:
    """
    Shortest angular difference from current to target.
    Returns value in [-180, +180]. Positive means target is clockwise from current.
    """
    diff = (target_deg - current_deg) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff

def offset_lat_lon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """
    Offset a coordinate by meters in N/E directions.
    Uses flat-Earth approximation (valid for small offsets < ~10 km).
    """
    d_lat = north_m / EARTH_RADIUS_M
    d_lon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(d_lat), lon_deg + math.degrees(d_lon)


def point_from_bearing(lat_deg: float, lon_deg: float, bearing_deg_val: float, distance_m_val: float) -> tuple[float, float]:
    """
    Project a point from origin along bearing for distance_m.
    Uses the Vincenty direct formula (spherical approximation).
    """
    lat_r = math.radians(lat_deg)
    lon_r = math.radians(lon_deg)
    brg_r = math.radians(bearing_deg_val)
    d_over_r = distance_m_val / EARTH_RADIUS_M

    lat2 = math.asin(
        math.sin(lat_r) * math.cos(d_over_r)
        + math.cos(lat_r) * math.sin(d_over_r) * math.cos(brg_r)
    )
    lon2 = lon_r + math.atan2(
        math.sin(brg_r) * math.sin(d_over_r) * math.cos(lat_r),
        math.cos(d_over_r) - math.sin(lat_r) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)

def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine distance between two geographic points in meters."""
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c