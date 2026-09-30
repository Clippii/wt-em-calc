"""Recovered type-3 state-1 proximity request and preparation arithmetic.

Runtime enablement, mode, surface service and manager fields are explicit inputs.
This constructs trace candidates; it does not select units or detonate missiles.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,div,safe_div
from proximity_geometry import norm2


def properties(rocket):
    nested=rocket.get('proximityFuse',{});radius=f32(nested.get('radius',5.))
    p={k:f32(nested.get(k,v)) for k,v in (('timeOut',.3),('radius',5.),('minimalAltitude',-1.),
        ('armDistance',radius),('armDistanceToTarget',-1.))}
    for key in ('detectAirUnits','detectGroundUnits','detectGround','detectShells','hasUnitsOnlyMode','detectDeadUnits'):
        p[key]=bool(nested.get(key,key not in ('hasUnitsOnlyMode','detectDeadUnits')))
    for alias,key in (('proximityFuseRange','radius'),('proximityFuseArmDistance','armDistance'),
                      ('proximityFuseArmDistanceToTarget','armDistanceToTarget')):
        if alias in rocket:p[key]=f32(rocket[alias])
    for alias,key in (('proximityFuseDetectAir','detectAirUnits'),('proximityFuseDetectGround','detectGroundUnits'),
                      ('proximityFuseDetectDeadUnits','detectDeadUnits')):
        if alias in rocket:p[key]=bool(rocket[alias])
    low,high=map(f32,nested.get('shellCaliberRange',[0.,.0001]))
    p.update(shellCaliberMin=low,shellCaliberInverseSpan=safe_div(1.,sub(high,low)))
    return p


def advance_clock(clock,frame_dt,has_fuse,expired):
    """expired means the frame time/distance check returned false.

    A guidance callback can mark destruction pending while that check still
    returns true; the native proximity clock advances on that visit.
    """
    return add(clock,frame_dt) if has_fuse and not expired else f32(clock)


def build(*,start,end,origin,elapsed,enabled,has_fuse,detect_shells,mode,
          prior_body_time,body_time,guidance_distance=-1.,guidance_flag=False):
    """Represented request fields; native queue index/allocation are external."""
    start,end,origin=[list(map(f32,v)) for v in (start,end,origin)]
    for v in (start,end,origin):
        if len(v)!=3 or not all(math.isfinite(x) for x in v):raise ValueError('Three finite coordinates required')
    # The native builder has other paths for large/invalid source positions.
    if add(add(mul(start[0],start[0]),mul(start[1],start[1])),mul(start[2],start[2]))>f32(1e11):
        raise NotImplementedError('Large-position trace-builder branch not recovered')
    delta=[sub(e,s) for s,e in zip(start,end)];length=f32(math.sqrt(norm2(delta)))
    if length<f32(1e-5):return dict(queued=False,length=length)
    direction=[mul(d,div(1.,length)) for d in delta]
    return dict(queued=True,collision_flags=3,shell_request=bool(not mode and has_fuse and detect_shells),
        elapsed=f32(elapsed),origin=origin,properties_bound=bool(enabled and has_fuse),start=start,direction=direction,
        length=min(length,1000.),original_length=min(length,1000.),guidance_scalar=f32(guidance_distance),
        guidance_flag=int(bool(guidance_flag)),prior_body_time=f32(prior_body_time),body_time=f32(body_time),candidate_results=[-1.,-1.])


def prepare(descriptor,p,surface):
    """Retain native endpoint gate/query order; surface receives endpoint xyz."""
    if not descriptor['queued']:raise ValueError('A queued trace is required')
    d=deepcopy(descriptor);end=[add(s,mul(v,d['length'])) for s,v in zip(d['start'],d['direction'])]
    queried=d['properties_bound'] and d['elapsed']>=p['timeOut'] and norm2([sub(e,o) for e,o in zip(end,d['origin'])])>=mul(p['armDistance'],p['armDistance'])
    queries=[];allowed=False
    if queried:
        queries.append(end[:]);height=f32(surface(end[:]))
        allowed=not (p['minimalAltitude']>0 and sub(end[1],p['minimalAltitude'])<=height)
        allowed=allowed and not (p['armDistanceToTarget']>0 and d['guidance_scalar']>0 and d['guidance_scalar']>=p['armDistanceToTarget'])
    mask=((8 if p['detectAirUnits'] else 0)|(4 if p['detectGroundUnits'] else 0)) if allowed else 0
    d['collision_flags']=(d['collision_flags']&~12)|mask
    return dict(descriptor=d,flags=d['collision_flags'],air_enabled=bool(mask&8),ground_enabled=bool(mask&4),endpoint=end,surface_queries=queries)
