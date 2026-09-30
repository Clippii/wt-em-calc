"""Independent arithmetic for pinned game proximity-candidate point geometry."""
import math
from kernels import f32,add,sub,mul,div


EPS=f32(4e-19)


def norm2(v):
    return add(add(mul(v[2],v[2]),mul(v[1],v[1])),mul(v[0],v[0]))


def closest_point(point,start,end):
    point,start,end=[list(map(f32,v)) for v in (point,start,end)]
    delta=[sub(b,a) for a,b in zip(start,end)]
    length2=norm2(delta);length=f32(math.sqrt(length2))
    inv=div(1.,length) if length>EPS else 0.
    direction=[mul(d,inv) for d in delta]
    offset=[sub(p,s) for p,s in zip(point,start)]
    projection=add(mul(offset[2],direction[2]),add(mul(offset[1],direction[1]),mul(offset[0],direction[0])))
    if projection<0:closest=start
    elif mul(projection,projection)>length2:closest=end
    else:closest=[add(mul(d,projection),s) for d,s in zip(direction,start)]
    return dict(point=closest,projection=projection,distance_squared=norm2([sub(p,c) for p,c in zip(point,closest)]),
                segment_length=length,segment_length_squared=length2)


def point_candidate(*,point,start,end,origin,elapsed,timeout,arm_distance,radius,scale,height_field=-1.,
                    properties=True,owner=False,same_owner=False,same_team=False,rule_flags=0,friendly_enabled=False):
    if not properties or f32(elapsed)<f32(timeout):return -1.
    if norm2([sub(s,o) for s,o in zip(start,origin)])<mul(arm_distance,arm_distance):return -1.
    c=closest_point(point,start,end)
    if f32(height_field)>0 and add(c['point'][1],height_field)>f32(point[1]):
        detected=(c['distance_squared']<=mul(height_field,height_field) and c['projection']>=0
                  and mul(c['projection'],c['projection'])<=c['segment_length_squared'])
    else:detected=c['distance_squared']<=mul(radius,radius)
    if not detected:return -1.
    if owner and (same_owner or (not rule_flags&0x1080000 and same_team and not friendly_enabled)):return -1.
    distance=f32(math.sqrt(norm2([sub(s,c) for s,c in zip(start,c['point'])])))
    if c['segment_length']>EPS:distance=div(mul(distance,scale),c['segment_length'])
    return max(distance,EPS)
