"""Pure sample-pairing and coordinate math shared by simulation consumers."""

from collections import deque
from dataclasses import dataclass
import math


# Allow independent /clock and sensor callbacks to arrive a few sim steps apart.
SIM_CLOCK_TOLERANCE_S = 0.02


@dataclass(frozen=True)
class Sample:
    time_s: float
    value: tuple[float, ...]
    received_s: float
    reference: tuple[float, float, float] | None = None


def slerp(a, b, fraction):
    """Shortest-arc quaternion interpolation, in Hamilton wxyz order."""
    norm_a, norm_b = math.sqrt(sum(x*x for x in a)), math.sqrt(sum(x*x for x in b))
    a = tuple(v / norm_a for v in a)
    b = tuple(v / norm_b for v in b)
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b = tuple(-x for x in b)
        dot = -dot
    if dot > 0.9995:
        q = tuple(x + fraction*(y-x) for x, y in zip(a, b))
        norm = math.sqrt(sum(x*x for x in q))
        return tuple(v / norm for v in q)
    angle = math.acos(min(1.0, dot))
    return tuple((math.sin((1-fraction)*angle)*x + math.sin(fraction*angle)*y)
                 / math.sin(angle) for x, y in zip(a, b))


class SamplePairs:
    """Interpolate truth at each PX4 sample time; never extrapolate or reuse it.

    Monotonic receipt times bound staleness even when simulation time stops.
    Each quantity has its own queue, so attitude/odometry are not sampled at
    unrelated VehicleLocalPosition callback times.
    """

    def __init__(self, max_gap_s=0.1, max_age_s=0.5):
        self.max_gap_s = max_gap_s
        self.max_age_s = max_age_s
        self.pending = {}
        self.truth = {}
        self.last_px4 = {}
        self.dropped = 0

    def clear(self):
        self.pending.clear()
        self.truth.clear()
        self.last_px4.clear()

    def add(self, source, key, sample):
        if not math.isfinite(sample.time_s) or sample.time_s < 0:
            return False
        if not all(math.isfinite(v) for v in sample.value):
            return False
        if key == 'rpy' and sum(v*v for v in sample.value) < 1e-12:
            return False
        queues = self.pending if source == 'px4' else self.truth
        queue = queues.setdefault(key, deque(maxlen=256))
        last = self.last_px4.get(key) if source == 'px4' else (queue[-1].time_s if queue else None)
        if last is not None and sample.time_s <= last:
            self.dropped += 1
            return False
        if source == 'px4':
            self.last_px4[key] = sample.time_s
        queue.append(sample)
        return True

    def ready(self, now_s):
        for key, queue in self.pending.items():
            truth = self.truth.get(key, ())
            while queue:
                sample = queue[0]
                if now_s - sample.received_s > self.max_age_s:
                    queue.popleft()
                    self.dropped += 1
                    continue
                if not truth or truth[-1].time_s < sample.time_s:
                    break
                queue.popleft()
                if sample.time_s < truth[0].time_s:
                    self.dropped += 1
                    continue
                right = next(i for i, x in enumerate(truth) if x.time_s >= sample.time_s)
                b = truth[right]
                a = b if b.time_s == sample.time_s else truth[right-1]
                span = b.time_s - a.time_s
                if span > self.max_gap_s or now_s - min(a.received_s, b.received_s) > self.max_age_s:
                    self.dropped += 1
                    continue
                fraction = (sample.time_s-a.time_s)/span if span else 0.0
                value = (slerp(a.value, b.value, fraction) if key == 'rpy' else
                         tuple(x + fraction*(y-x) for x,y in zip(a.value,b.value)))
                yield key, sample, value, a.time_s, b.time_s


def world_to_geodetic(point_enu, origin):
    """Gazebo EARTH_WGS84 ENU tangent plane -> latitude/longitude/altitude.

    This simulation's GZBridge uses NavSat altitude for both MSL and ellipsoid
    altitude; no geoid correction is applied here (not a real-flight model).
    """
    lat, lon = map(math.radians, origin[:2])
    a = 6378137.0
    e2 = 6.6943799901413165e-3
    n = a / math.sqrt(1-e2*math.sin(lat)**2)
    east, north, up = point_enu
    x = (n+origin[2])*math.cos(lat)*math.cos(lon) - math.sin(lon)*east - math.sin(lat)*math.cos(lon)*north + math.cos(lat)*math.cos(lon)*up
    y = (n+origin[2])*math.cos(lat)*math.sin(lon) + math.cos(lon)*east - math.sin(lat)*math.sin(lon)*north + math.cos(lat)*math.sin(lon)*up
    z = (n*(1-e2)+origin[2])*math.sin(lat) + math.cos(lat)*north + math.sin(lat)*up
    p = math.hypot(x,y)
    latitude = math.atan2(z,p*(1-e2))
    for _ in range(8):
        n = a / math.sqrt(1-e2*math.sin(latitude)**2)
        latitude = math.atan2(z+e2*n*math.sin(latitude),p)
    n = a / math.sqrt(1-e2*math.sin(latitude)**2)
    altitude = p*math.cos(latitude)+z*math.sin(latitude)-n*(1-e2*math.sin(latitude)**2)
    return math.degrees(latitude), math.degrees(math.atan2(y,x)), altitude


def geodetic_to_px4_ned(point, reference):
    """PX4 MapProjection azimuthal equidistant XY, altitude relative to ref_alt."""
    lat, lon = map(math.radians, point[:2])
    lat0, lon0 = map(math.radians, reference[:2])
    cosine = math.sin(lat0)*math.sin(lat)+math.cos(lat0)*math.cos(lat)*math.cos(lon-lon0)
    c = math.acos(max(-1.0,min(1.0,cosine)))
    k = c/math.sin(c) if c > 1e-12 else 1.0
    north = k*(math.cos(lat0)*math.sin(lat)-math.sin(lat0)*math.cos(lat)*math.cos(lon-lon0))*6371000.0
    east = k*math.cos(lat)*math.sin(lon-lon0)*6371000.0
    return north, east, reference[2]-point[2]
