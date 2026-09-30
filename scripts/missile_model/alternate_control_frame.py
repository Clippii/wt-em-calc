"""Recovered non-matrix velocity frame from .38 acceleration control.

Finite arithmetic with round-to-nearest and gradual underflow. This is the
game's two-vector rotation and ordered quaternion product, not a substituted
generic quaternion library.
"""
import math
from kernels import f32,add,sub,mul,div


def norm(values,order):
    a,b,c=(mul(values[i],values[i]) for i in order)
    length=f32(math.sqrt(add(add(a,b),c)))
    inverse=div(1.,length) if length>f32(4e-19) else 0.
    return [mul(v,inverse) for v in values]


def frame(quaternion,velocity,velocity_reference=True):
    q=list(map(f32,quaternion))
    if not velocity_reference:return q
    x,y,z,w=q
    forward=[add(mul(2.,add(mul(x,x),mul(w,w))),-1.),
        mul(2.,add(mul(w,z),mul(y,x))),mul(2.,sub(mul(z,x),mul(w,y)))]
    forward=norm(forward,(2,1,0))
    vx,vy,vz=map(f32,velocity)
    speed=f32(math.sqrt(add(mul(vz,vz),add(mul(vy,vy),mul(vx,vx)))))
    desired=[mul(v,div(1.,speed)) for v in (vx,vy,vz)] if speed>1e-9 else forward[:]
    # Both vectors are normalized again by this branch. They use different
    # accumulation orders; retaining these rounds matters even at tiny speed.
    fx,fy,fz=norm(forward,(2,1,0));tx,ty,tz=norm(desired,(0,1,2))
    cosine=add(mul(tz,fz),add(mul(ty,fy),mul(tx,fx)))
    if cosine<f32(-.9999):
        if abs(fz)>0.7071067811865476:
            length=f32(math.sqrt(add(mul(fy,fy),mul(fz,fz))))
            inverse=div(1.,length) if length>f32(4e-19) else 0.
            rx,ry,rz,rw=0.,mul(-fz,inverse),mul(fy,inverse),0.
        else:
            length=f32(math.sqrt(add(mul(fx,fx),mul(fy,fy))))
            inverse=div(1.,length) if length>f32(4e-19) else 0.
            rx,ry,rz,rw=mul(-fy,inverse),mul(fx,inverse),0.,0.
    else:
        rx=sub(mul(tz,fy),mul(ty,fz));ry=sub(mul(fz,tx),mul(tz,fx));rz=sub(mul(ty,fx),mul(tx,fy))
        length=f32(math.sqrt(add(add(cosine,cosine),2.)))
        inverse=div(1.,length) if length>f32(4e-19) else 0.
        rx,ry,rz,rw=mul(rx,inverse),mul(ry,inverse),mul(rz,inverse),mul(length,.5)
    return [add(sub(add(mul(rz,y),mul(rw,x)),mul(z,ry)),mul(rx,w)),
        add(add(sub(mul(rw,y),mul(x,rz)),mul(ry,w)),mul(rx,z)),
        sub(add(mul(ry,x),add(mul(rw,z),mul(rz,w))),mul(y,rx)),
        sub(mul(rw,w),add(mul(rx,x),add(mul(ry,y),mul(rz,z))))]
