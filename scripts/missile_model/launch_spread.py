"""Independent arithmetic for the pinned game's attitude/velocity release spread.

Research components only. The piecewise random lookup is supplied as a pinned
table; scalar sin/cos use Python's math library rounded to float32, with measured
native discrepancies reported by the probe rather than an exactness guarantee.
"""
import math
import struct
from kernels import f32,add,sub,mul,div
from body_integration import half_angle_trig

MASK=0xffffffff
STEP=0x9e3779b9


def mix(seed):
    product=(seed^(seed>>16))*0x21f0aaad
    return ((product>>32)^product)&MASK


def sample(seed,global_seed,table):
    seed&=MASK;global_seed&=MASK
    if seed==0:
        global_seed=(global_seed+STEP)&MASK
        seed=mix(global_seed)>>1
    word=(seed*0x41c64e6d+0x3039)&MASK
    index=(word>>23)&255;fraction=(word>>16)&127
    slope,intercept=table[index]
    magnitude=add(mul(float(fraction),slope),intercept)
    bits=(mix((word+STEP)&MASK)>>9)|0x3f800000
    uniform=struct.unpack('<f',struct.pack('<I',bits))[0]
    azimuth=add(mul(uniform,6.2831854820251465),-6.2831854820251465)
    trig=half_angle_trig(mul(-2.,azimuth))
    return dict(magnitude=magnitude,azimuth=azimuth,sine=trig['sine'],cosine=trig['cosine'],
                index=index,fraction=fraction,effective_seed=seed,global_seed=global_seed)


def attitude(q,max_angle,scale,seed,global_seed,table):
    q=list(map(f32,q));max_angle=f32(max_angle)
    if max_angle<0:return dict(quaternion=q,global_seed=global_seed,sample=None)
    draw=sample(seed,global_seed,table)
    norm=f32(math.sqrt(add(mul(draw['sine'],draw['sine']),mul(draw['cosine'],draw['cosine']))))
    inverse=div(1.,norm) if norm>f32(4e-19) else 0.
    half=mul(mul(mul(scale,.008726646192371845),max_angle),draw['magnitude'])
    s=mul(f32(math.sin(half)),inverse)
    dy=mul(s,draw['sine']);dz=mul(s,draw['cosine']);dw=f32(math.cos(half))
    x,y,z,w=q
    out=[sub(add(mul(dz,y),mul(dw,x)),mul(dy,z)),
         sub(add(mul(dw,y),mul(dy,w)),mul(dz,x)),
         add(mul(dy,x),add(mul(dw,z),mul(dz,w))),
         sub(mul(dw,w),add(mul(dz,z),mul(dy,y)))]
    return dict(quaternion=out,global_seed=draw['global_seed'],sample=draw,half_angle=half)


def velocity(v,q,*,first_delay,propulsion_count,start_speed,use_start_speed,max_angle,scale,seed,global_seed,table):
    vx,vy,vz=map(f32,v);x,y,z,w=map(f32,q)
    # Preserve native raw quaternion matrix rounding and each summation order.
    twice_z=add(z,z);xz=mul(x,twice_z);minus_twice_w=mul(-2.,w)
    nyw=mul(y,minus_twice_w);nxw=mul(x,minus_twice_w);nzw=mul(minus_twice_w,z)
    ww=sub(add(mul(w,w),mul(w,w)),1.)
    xx=add(add(mul(x,x),mul(x,x)),ww)
    yy=add(add(mul(y,y),mul(y,y)),ww)
    zz=add(add(mul(z,z),mul(z,z)),ww)
    twice_y=add(y,y);xy=mul(twice_y,x);yz=mul(twice_z,y)
    local_x=add(mul(vx,xx),add(mul(sub(xy,nzw),vy),mul(add(nyw,xz),vz)))
    local_y=add(add(mul(add(xy,nzw),vx),mul(sub(yz,nxw),vz)),mul(vy,yy))
    local_z=add(add(mul(sub(xz,nyw),vx),mul(add(nxw,yz),vy)),mul(vz,zz))
    factor=1. if propulsion_count and f32(first_delay)>0 else f32(.333)
    local_x=add(local_x,start_speed if use_start_speed else 200.)
    local_y=mul(local_y,factor);local_z=mul(local_z,factor)
    draw=None;angle=0.
    if f32(max_angle)>0:
        draw=sample(seed,global_seed,table);global_seed=draw['global_seed']
        angle=mul(mul(mul(scale,.01745329238474369),max_angle),draw['magnitude'])
        spread=mul(f32(math.sin(angle)),local_x)
        local_y=add(local_y,mul(draw['sine'],spread))
        local_z=add(local_z,mul(spread,draw['cosine']))
    yw=mul(twice_y,w);twice_x=add(x,x);xw=mul(twice_x,w)
    xy2=mul(twice_x,y);zw=mul(twice_z,w)
    out=[add(mul(add(yw,xz),local_z),add(mul(sub(xy2,zw),local_y),mul(xx,local_x))),
         add(mul(sub(yz,xw),local_z),add(mul(yy,local_y),mul(add(xy2,zw),local_x))),
         add(mul(zz,local_z),add(mul(add(xw,yz),local_y),mul(sub(xz,yw),local_x)))]
    return dict(velocity=out,global_seed=global_seed,sample=draw,angle=angle,local=[local_x,local_y,local_z])
