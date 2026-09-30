"""Shared War Thunder sensor kernel 14331fca0, zero-size target specialization.

Build 2.59.0.34. Includes authored radar lobes and range-response modifier.
The raw 48-float packet is an evidence interface, not a simulator public API.
Target centroid offsets and finite bounds remain explicit unsupported inputs.
"""
import math
import struct
from kernels import f32,add,sub,mul,div,safe_div
from optical_geometry import local_point


def native_asin(value):
    """Finite [-1,1] numerical path of the pinned CRT routine 146b63c88."""
    x=f32(value);a=abs(x)
    if a>1. or not math.isfinite(a):raise ValueError('asin domain is finite [-1,1]')
    if a<2.**-14:return x
    if a==1.:return math.copysign(f32(math.pi/2.),x)
    large=a>=.5
    t=mul(sub(1.,a),.5) if large else mul(a,a)
    root=f32(math.sqrt(t)) if large else a
    numerator=sub(-.013381929136812687,mul(t,.003961374517530203))
    numerator=sub(mul(numerator,t),.05652986839413643)
    numerator=add(mul(numerator,t),.18416160345077515)
    numerator=mul(numerator,t)
    ratio=div(numerator,sub(1.1049696207046509,mul(t,.8364112973213196)))
    if large:
        bits=struct.unpack('<I',struct.pack('<f',root))[0]&0xffff0000
        truncated=struct.unpack('<f',struct.pack('<I',bits))[0]
        correction=div(sub(t,mul(truncated,truncated)),add(truncated,root))
        tail=sub(mul(add(root,root),ratio),sub(7.549789415861596e-08,add(correction,correction)))
        tail=sub(tail,sub(.7853981256484985,mul(truncated,2.)))
        result=sub(.7853981256484985,tail)
    else:result=add(mul(ratio,a),a)
    return math.copysign(result,x)


def lobe(squared,values,side_lobes):
    cosine,exponent,side,half,cutoff,falloff=values
    if sub(1.,squared)>mul(abs(cosine),cosine):return 1.
    value=f32(math.exp(mul(-squared,exponent)))
    if side_lobes and value<side:
        angle=native_asin(min(f32(math.sqrt(squared)),1.))
        fraction=safe_div(max(sub(angle,cutoff),0.),sub(f32(math.pi/2.),cutoff))
        tail=mul(f32(math.exp(mul(mul(falloff,-2.),fraction))),side)
        value=max(value,tail)
    return value


def point_response(packet,*,side_lobes=False,range_scaling=False):
    p=list(map(f32,packet))
    if len(p)!=48:raise ValueError('Expected the 48-float sensor packet')
    if any(p[35:41]):raise NotImplementedError('Target centroid offset/finite extents require the full geometry kernel')
    frame=p[:12];target=p[27:30];local=local_point(frame,target)
    if local[0]<0.:return dict(accepted=False,reason='behind')
    delta=[sub(t,o) for t,o in zip(target,frame[9:12])]
    squared=add(add(mul(delta[2],delta[2]),mul(delta[1],delta[1])),mul(delta[0],delta[0]))
    if squared<100.:return dict(accepted=False,reason='minimum_distance')
    earth=mul(p[26],6371000.);earth2=mul(earth,earth)
    h1=add(max(frame[10],1.),earth);h2=add(max(p[34],1.),earth)
    a=sub(mul(h1,h1),earth2);b=sub(mul(h2,h2),earth2)
    root=f32(math.sqrt(mul(b,a)));horizon_squared=add(add(b,a),add(root,root))
    if squared>horizon_squared:return dict(accepted=False,reason='horizon')
    distance=f32(math.sqrt(squared));inverse=safe_div(1.,distance)
    base=mul(inverse,inverse)
    if range_scaling and distance<=p[45]:
        if distance>p[44]:base=p[47]
        elif distance<p[44]:base=mul(base,p[46])
        else:base=p[46]
    if mul(p[41],base)<p[42]:return dict(accepted=False,reason='range_response_threshold')
    unit_local=[mul(v,inverse) for v in local]
    yy=mul(unit_local[1],unit_local[1]);zz=mul(unit_local[2],unit_local[2])
    radial=add(mul(mul(p[24],p[24]),yy),zz)
    primary=lobe(radial,p[12:18],side_lobes)
    secondary=lobe(yy,p[18:24],side_lobes)
    gain=min(primary,secondary);response=mul(base,gain)
    if mul(p[41],response)<p[42]:return dict(accepted=False,reason='lobe_threshold')
    output=[*[mul(v,inverse) for v in delta],distance,inverse,*unit_local,0.,0.,0.,response,gain]
    return dict(accepted=True,reason=None,output=output)
