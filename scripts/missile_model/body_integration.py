"""Readable single-update arithmetic from game build 2.59.0.34.

Research component, not a trajectory simulator. Inputs are already evaluated
world acceleration (double), body angular acceleration (float), and body state.
Only the pure-air immersion result is included; no surrounding scheduler.
"""
import math
from kernels import f32, add, sub, mul, div


def half_angle_trig(increment):
    """Native packed polynomial sin/cos of NEGATIVE half the angular increment."""
    h=mul(-f32(increment),.5)
    quadrant_input=sub(math.copysign(.5,h),mul(increment,.31830987334251404))
    quadrant=math.trunc(quadrant_input)
    if not -(2**31)<=quadrant<2**31:
        raise ValueError('Outside audited finite quadrant-conversion domain')
    t=add(mul(f32(quadrant),-1.5707963705062866),h)
    t2=mul(t,t)
    c=add(mul(add(mul(add(mul(-.0013602249091491103,t2),.04165669530630112),t2),
                      -.4999990165233612),t2),1.)
    s=add(mul(add(mul(add(mul(-.0001950727018993348,t2),.00833207555115223),t2),
                      -.16666652262210846),mul(t2,t)),t)
    if quadrant&1:s,c=c,s
    if quadrant&2:s=-s
    if (quadrant+1)&2:c=-c
    return dict(sine=s,cosine=c,quadrant=quadrant,reduced=t)


def orientation(quaternion,increment):
    trig=[half_angle_trig(v) for v in increment]
    sx,sy,sz=[v['sine'] for v in trig];cx,cy,cz=[v['cosine'] for v in trig]
    # Exact product/sum order of the packed Euler increment construction.
    a=[mul(mul(sy,cx),sz),mul(mul(sy,cx),cz),mul(mul(cy,cx),sz),mul(mul(cy,cx),cz)]
    b=[mul(mul(cy,sx),cz),mul(mul(cy,sx),sz),mul(mul(sy,sx),cz),mul(mul(sy,sx),sz)]
    dx,dy,dz,dw=add(b[0],a[0]),add(b[1],a[1]),sub(a[2],b[2]),sub(a[3],b[3])
    x,y,z,w=map(f32,quaternion)
    raw=[sub(add(mul(dz,y),add(mul(dx,w),mul(dw,x))),mul(dy,z)),
         sub(add(mul(dx,z),add(mul(dw,y),mul(dy,w))),mul(dz,x)),
         sub(add(mul(dy,x),add(mul(dw,z),mul(dz,w))),mul(dx,y)),
         sub(mul(dw,w),add(mul(dx,x),add(mul(dz,z),mul(dy,y))))]
    norm2=add(add(mul(raw[3],raw[3]),mul(raw[2],raw[2])),
              add(mul(raw[1],raw[1]),mul(raw[0],raw[0])))
    q=[mul(v,div(1.,f32(math.sqrt(norm2)))) for v in raw] if norm2!=0. else [0.]*4
    return dict(trig=trig,delta=[dx,dy,dz,dw],raw=raw,quaternion=q)


def integrate(state,acceleration,angular_acceleration,dt,absolute_time,clock_rates=(0.,)*4):
    dt=f32(dt);half_dt2=mul(mul(dt,dt),.5)
    # Conversion to float occurs AFTER multiplication of double acceleration.
    displacement=[add(f32(a*half_dt2),mul(v,dt)) for a,v in zip(acceleration,state['velocity'])]
    position=[add(p,d) for p,d in zip(state['position'],displacement)]
    velocity=[add(f32(a*dt),v) for a,v in zip(acceleration,state['velocity'])]
    increment=[add(mul(a,half_dt2),mul(w,dt)) for a,w in zip(angular_acceleration,state['omega'])]
    omega=[add(mul(a,dt),w) for a,w in zip(angular_acceleration,state['omega'])]
    rotation=orientation(state['quaternion'],increment)
    vx,vy,vz=velocity
    distance_step=mul(f32(math.sqrt(add(add(mul(vz,vz),mul(vy,vy)),mul(vx,vx)))),dt)
    elapsed=sub(absolute_time,state['time'])
    clocks=[add(mul(rate,elapsed),old) for rate,old in zip(clock_rates,state['clocks'])]
    updated=dict(position=position,velocity=velocity,omega=omega,quaternion=rotation['quaternion'],
        time=f32(absolute_time),clocks=clocks,distance=add(state['distance'],distance_step),
        water_distance=add(distance_step,state['water_distance']) if state['water'] else state['water_distance'],
        immersion=0.)
    return dict(state=updated,displacement=displacement,increment=increment,rotation=rotation)
