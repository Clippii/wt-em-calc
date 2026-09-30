"""Recovered rocket distance setup and post-update expiry comparisons."""
from kernels import f32,add,sub,mul


def distance_setting(rocket,owner_scale=1.,requested=None):
    cap=mul(rocket.get('maxDistance',2000.),owner_scale)
    if requested is not None and rocket.get('distanceFuse',True) and f32(requested)<cap:
        return f32(requested)
    return cap


def evaluate(launch_time,lifetime,distance_limit,origin,position,now,initial_flag=False):
    d=[sub(o,p) for o,p in zip(origin,position)]
    distance_squared=add(add(mul(d[2],d[2]),mul(d[1],d[1])),mul(d[0],d[0]))
    deadline=add(launch_time,lifetime)
    time_expired=deadline<f32(now)
    distance_expired=f32(distance_limit)>0. and distance_squared>=mul(distance_limit,distance_limit)
    expired=time_expired or distance_expired
    return dict(returned=not expired,expiry_flag=bool(initial_flag or expired),time_expired=time_expired,
        distance_expired=distance_expired,deadline=deadline,distance_squared=distance_squared)
