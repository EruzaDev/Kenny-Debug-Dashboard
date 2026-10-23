"""Local measured-clearance decisions; unknown sectors are never treated as free."""
import math
from collections import deque
import numpy as np


class MeasuredScanHistory:
    """Minimum of recent actual returns; never interpolate unknown angular bins."""
    def __init__(self, max_age=.3):
        self.max_age=max_age
        self.samples=deque(maxlen=8)

    def add(self, ranges, valid, observed, pose):
        if pose is None or len(pose)!=3 or not np.isfinite(pose).all():
            return
        ranges,valid=np.asarray(ranges,dtype=float),np.asarray(valid,dtype=bool)
        if ranges.shape!=(72,) or valid.shape!=(72,) or not math.isfinite(observed):
            return
        if self.samples and observed<=self.samples[-1][0]:
            return
        valid=valid & np.isfinite(ranges) & (ranges>0)
        self.samples.append((observed,np.array(pose),ranges.copy(),valid.copy()))

    def packet(self, now, pose):
        if pose is None:
            return None,{'samples':0,'retained_bins':0}
        pose=np.asarray(pose)
        recent=[]
        for observed,previous,ranges,valid in self.samples:
            turn=abs(math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2])))
            # Only reuse same-frame evidence while virtually stationary. Old
            # clearance is discarded when motion changes ray geometry.
            if 0<=now-observed<=self.max_age and np.linalg.norm(pose[:2]-previous[:2])<=.005 and turn<=.005:
                recent.append((observed,ranges,valid))
        if not recent:
            return None,{'samples':0,'retained_bins':0}
        ranges=np.full(72,6.)
        valid=np.zeros(72,dtype=bool)
        for _,measurements,mask in recent:
            ranges[mask]=np.minimum(ranges[mask],measurements[mask])
            valid|=mask
        latest=recent[-1]
        return (ranges,valid,latest[0]),{'samples':len(recent),
            'retained_bins':int(np.count_nonzero(valid & ~latest[2])),
            'oldest_age_s':round(now-recent[0][0],3),
            'valid_fraction':float(valid.mean())}


class DirectionStability:
    def __init__(self, dwell=.25):
        self.dwell=dwell
        self.candidate=None
        self.since=0.

    def update(self, direction, now):
        if direction is None:
            self.candidate=None
            return None
        if direction!=self.candidate:
            self.candidate=direction
            self.since=now
        return direction if now-self.since>=self.dwell else None


def roaming_direction(sensors, now, velocity, radius=.163, preferred=None, diagnostics=None):
    diagnostics=diagnostics if diagnostics is not None else {}
    for name in ('lidar','depth','floor'):
        if name not in sensors or now-sensors[name][2]>.5:
            return None,name+'_stale'
    ranges,valid,_=sensors['lidar']
    ranges,valid=np.asarray(ranges),np.asarray(valid,dtype=bool)
    depth,dvalid,_=sensors['depth']
    floor,fvalid,_=sensors['floor']
    dvalid,fvalid=np.asarray(dvalid,dtype=bool),np.asarray(fvalid,dtype=bool)
    if valid.sum()<65:
        return None,'Measured LiDAR coverage insufficient for footprint monitoring'
    if dvalid.sum()<24 or fvalid.sum()<8:
        return None,'Depth/floor coverage insufficient for footprint monitoring'
    footprint=radius+.2
    if np.any(ranges[valid]<footprint) or np.any(np.asarray(depth)<footprint):
        return None,'Measured obstacle too close to turn'
    angles=np.linspace(-math.pi,math.pi,72,endpoint=False)
    speed=max(.3,abs(velocity[0]))
    corridor_length=max(.9,radius+.25+speed*.3+speed*speed/.7)
    half_width=radius+.08
    front=np.cos(angles)>.1
    exit_distance=np.minimum(corridor_length/np.maximum(np.cos(angles),.001),
        half_width/np.maximum(np.abs(np.sin(angles)),.001))
    # A side return matters only if it is inside the swept corridor, rather
    # than treating every return in a fixed cone as an obstacle ahead.
    lidar_clear=valid[front].all() and np.all(ranges[front]>exit_distance[front])
    depth_angles=np.linspace(-math.pi/6,math.pi/6,36)
    depth_front=np.abs(depth_angles)<=math.atan2(half_width,corridor_length)
    floor_angles=np.linspace(-math.pi/6,math.pi/6,12)
    floor_front=np.abs(floor_angles)<=math.atan2(half_width,corridor_length)
    diagnostics.update(corridor_width_m=round(2*half_width,3),corridor_length_m=round(corridor_length,3),
        missing_forward_lidar_bins=np.flatnonzero(front & ~valid).tolist(),
        depth_corridor_covered=bool(dvalid[depth_front].all()),
        floor_corridor_covered=bool(fvalid[floor_front].all()))
    if np.any(np.asarray(floor)<.6):
        return None,'Measured floor drop; roaming stopped'
    camera_clear=dvalid[depth_front].all() and fvalid[floor_front].all() and np.all(np.asarray(depth)[depth_front]>corridor_length)
    if lidar_clear and camera_clear:
        return 0.,'Measured forward corridor clear'
    choices=[]
    for sign in (1,-1):
        # An in-place turn sweeps the circumscribed footprint. Select an
        # observed side corridor, not an unnecessarily broad 25..90-degree cone.
        side=(angles*sign>=math.radians(45))&(angles*sign<=math.radians(75))
        if valid[side].all() and ranges[side].min()>.65:
            choices.append((float(ranges[side].min()),sign))
    if not choices:
        return None,'Forward blocked/unknown; neither turning side has measured clearance'
    old_sign=1 if preferred is not None and preferred>0 else -1 if preferred is not None and preferred<0 else 0
    _,sign=next((choice for choice in choices if choice[1]==old_sign),max(choices))
    return sign*.5,'Turn left toward measured clearance' if sign>0 else 'Turn right toward measured clearance'
