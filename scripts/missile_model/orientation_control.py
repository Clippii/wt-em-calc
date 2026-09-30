"""Independent orientation autopilot, pinned native routine 145298d10.

Native float32 arithmetic and recovered scalar-SSE tanf. Host atan2 precision
remains an explicit validation boundary. No launch latch initialization is inferred.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,div,safe_div
from optical_geometry import boresight
from native_tanf import tanf


def properties(c):
    degrees=f32(.01745329238474369)
    elevation=c.get('elevationAdd',0.)
    a,b,d,e=c.get('rangeToElevationAdd',[0.,elevation,0.,elevation])
    h=c.get('elevationToHeadingMult',[0.,0.,0.,0.])
    return dict(start=f32(c.get('timeOut',0.)),duration=f32(c.get('controlTime',0.)),
        final_speed=mul(c.get('finalSpeed',0.),f32(.2777777910232544)),final_distance=f32(c.get('finalDist',0.)),
        depth_min=f32(c.get('depthMin',-2147440000.)),elevation=[mul(a,a),mul(b,degrees),mul(d,d),mul(e,degrees)],
        heading=[mul(h[0],degrees),f32(h[2]),mul(h[1],degrees),f32(h[3])],
        angle_to_rate=f32(c.get('angleToAngularRate',0.)),coefficients=[f32(c.get(key,default)) for key,default in
            (('accelControlProp',.1),('accelControlIntg',.01),('accelControlIntgLim',.5),('accelControlDiff',.01),('accelControlDTau',1/48))],
        use_fins=bool(c.get('useFinsToOrientation',False)))


def interpolate(points,x):
    a,b,c,d=map(f32,points);x=f32(x)
    if a>c:a,b,c,d=c,d,a,b
    if x<=a:return b
    if x>=c:return d
    return add(b,safe_div(mul(sub(x,a),sub(d,b)),sub(c,a)))


def elevated_angles(q,angles,elevation):
    vx,vy,vz=boresight(q,angles)
    horizontal=f32(math.sqrt(add(mul(vx,vx),mul(vz,vz))))
    inverse=div(1.,horizontal) if horizontal>f32(4e-19) else 0.
    ux,uz=mul(inverse,vx),mul(inverse,-vz)
    along=sub(mul(ux,vx),mul(uz,vz));tangent=tanf(elevation)
    term=mul(vy,tangent)
    vx=sub(vx,mul(ux,term));vy=add(mul(along,tangent),vy);vz=add(mul(term,uz),vz)
    x,y,z,w=map(f32,q)
    zz=add(z,z);a=mul(x,zz);b=mul(y,mul(-2.,w));c=mul(zz,y);d=mul(x,mul(-2.,w))
    e=mul(mul(-2.,w),z);base=add(add(mul(w,w),mul(w,w)),-1.)
    bz=add(add(mul(add(add(mul(z,z),mul(z,z)),base),vz),mul(sub(a,b),vx)),mul(add(d,c),vy))
    xy=mul(add(x,x),y)
    by=add(add(mul(sub(c,d),vz),mul(add(xy,e),vx)),mul(add(add(mul(y,y),mul(y,y)),base),vy))
    bx=add(add(mul(add(b,a),vz),mul(add(add(mul(x,x),mul(x,x)),base),vx)),mul(sub(xy,e),vy))
    return [f32(math.atan2(-bz,bx)),f32(math.atan2(by,f32(math.sqrt(add(mul(bz,bz),mul(bx,bx))))))]


def update(p,state,time,range_squared,q,omega,angles,depth,dt):
    s=deepcopy(state);time=f32(time);dt=f32(dt);depth=f32(depth)
    if s['enabled'] and (time>add(p['start'],p['duration']) or depth<p['depth_min']):s['enabled']=False
    result=dict(state=s,override=bool(s['enabled']),fins=[0.,0.],auxiliary=[0.,0.],active=False)
    if not s['enabled'] or time<p['start']:return result
    elevation=interpolate(p['elevation'],range_squared)
    angles=list(map(f32,angles)) if abs(elevation)<f32(2**-23) else elevated_angles(q,angles,elevation)
    heading=interpolate(p['heading'],abs(angles[1]))
    errors=[sub(mul(mul(heading,p['angle_to_rate']),-angles[0]),omega[1]),sub(mul(-angles[1],p['angle_to_rate']),omega[2])]
    kp,ki,limit,kd,tau=p['coefficients'];inverse=safe_div(1.,tau);values=[];states=[]
    for previous,error in zip(s['pid'],errors):
        integral=min(limit,max(-limit,add(mul(mul(error,dt),ki),previous[3])))
        derivative=add(mul(sub(error,add(mul(dt,previous[7]),previous[6])),inverse),previous[7])
        value=-add(add(integral,mul(kp,error)),mul(derivative,kd))
        values.append(min(1.,max(-1.,value)))
        states.append([kp,ki,limit,integral,kd,inverse,error,derivative])
    s['pid']=states;result['fins' if p['use_fins'] else 'auxiliary']=values
    result.update(active=True,angles=angles,elevation=elevation,errors=errors)
    return result
