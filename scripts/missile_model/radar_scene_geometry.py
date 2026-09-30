"""Recovered ordinary point-unit scene-query geometry, client 2.59.0.34.

Numerical admission only; scene indexing, ownership/type filters and associated
unit lookup remain explicit services. Preserves the native 50-unit padding.
"""
from kernels import f32,add,sub,mul


def admitted(position,origin,axis,cosine,minimum,maximum,radius=0.):
    p=list(map(f32,position));o=list(map(f32,origin));a=list(map(f32,axis))
    d=[sub(x,y) for x,y in zip(p,o)]
    norm=lambda v:add(add(mul(v[2],v[2]),mul(v[0],v[0])),mul(v[1],v[1]))
    distance=norm(d);lower=mul(max(sub(minimum,50.),0.),max(sub(minimum,50.),0.))
    upper=mul(abs(add(maximum,50.)),add(maximum,50.))
    dot=add(add(mul(d[2],a[2]),mul(d[0],a[0])),mul(d[1],a[1]))
    threshold=mul(min(mul(mul(abs(f32(cosine)),cosine),distance),
        sub(distance,mul(add(radius,50.),add(radius,50.)))),norm(a))
    return lower<=distance<=upper and mul(abs(dot),dot)>=threshold
