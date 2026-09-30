"""Recovered type-3 trace limiting and consumption without detailed contacts.

Scene queries and contact initiation remain external. A stored mode-2 delay is
not decremented here: the audited entity/frame/consumer path leaves it intact.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,div
from proximity_geometry import norm2


def selected_velocity(velocity,alternate=(0.,0.,0.)):
    vectors=[list(map(f32,v)) for v in (velocity,alternate)]
    if any(len(v)!=3 or not all(math.isfinite(x) for x in v) for v in vectors):
        raise ValueError('Three finite velocity components required')
    return vectors[1] if any(vectors[1]) else vectors[0]


def limit_pending(descriptor,*,mode,delay,velocity,alternate_velocity=(0.,0.,0.)):
    """Apply the native builder's mode-2 limit to an ordinary built request."""
    if mode not in (1,2):raise NotImplementedError('Only active modes 1 and 2 audited here')
    delay=f32(delay)
    if not math.isfinite(delay):raise ValueError('Finite stored delay required')
    d=deepcopy(descriptor)
    if not d['queued']:return d
    d['forced']=False
    if mode==2:
        vx,vy,vz=selected_velocity(velocity,alternate_velocity)
        speed=f32(math.sqrt(add(add(mul(vy,vy),mul(vx,vx)),mul(vz,vz))))
        distance=max(0.,mul(speed,delay))
        if d['length']>distance:
            d['forced']=True;d['length']=distance;d['original_length']=distance
    return d


def consume_no_contacts(descriptor,*,mode,delay,velocity,base_time,offsets,
                        alternate_velocity=(0.,0.,0.),auxiliary_enabled=False,initial_event_time=-1.):
    """Consume supplied trace results; no physical-contact records are allowed.

Returns an optional event and the native returned/marked/mode results. It does
not change the body pose or apply damage/removal. Timestamp offsets are explicit
caller inputs relative to base_time. Positions use the supplied trace direction.
"""
    if not descriptor['queued']:raise ValueError('A queued descriptor is required')
    if mode not in (0,1,2,3,4):raise NotImplementedError('Unsupported projectile mode')
    d=descriptor;length,original,delay=map(f32,(d['length'],d['original_length'],delay))
    auxiliary,proximity=map(f32,d['candidate_results'])
    prior,body,base,initial=map(f32,(d['prior_body_time'],d['body_time'],base_time,initial_event_time))
    low,high=map(f32,offsets)
    start,direction=[list(map(f32,v)) for v in (d['start'],d['direction'])]
    values=[length,original,delay,auxiliary,proximity,prior,body,base,initial,low,high,*start,*direction]
    if len(start)!=3 or len(direction)!=3 or not all(math.isfinite(x) for x in values):
        raise ValueError('Finite trace coordinates and scalars required')
    limit=length;delayed=-1.
    if mode==2:
        vx,vy,vz=selected_velocity(velocity,alternate_velocity)
        speed=f32(math.sqrt(add(add(mul(vz,vz),mul(vx,vx)),mul(vy,vy))))
        delayed=max(mul(delay,speed),0.);limit=min(limit,delayed)
    distance=limit if d.get('forced',False) else -1.
    if delayed>0. and delayed<limit:distance=delayed
    if proximity>=0. and (distance<0. or proximity<distance):distance=proximity
    valid=mode not in (3,4) and distance>=0.
    returned=valid or (auxiliary>0. and auxiliary_enabled)
    event=None
    if valid:
        position=[add(s,mul(v,min(limit,distance))) for s,v in zip(start,direction)]
        stamp=initial
        if stamp<=0.:
            if prior>=0. and original>f32(4e-19):
                delta=[sub(s,p) for s,p in zip(start,position)]
                fraction=min(div(f32(math.sqrt(norm2(delta))),original),1.)
                stamp=add(prior,mul(sub(body,prior),fraction))
                stamp=min(max(add(base,low),stamp),add(base,high))
            else:stamp=base
        event=dict(position=position,direction=direction,time=stamp)
    return dict(returned=bool(returned),marked=bool(returned),mode=4 if valid else mode,
                delay=delay,event=event)
