"""Isolated reconstruction of 0x14031fce0; history services are inputs.

This is a collision-segment experiment, not a missile simulator or a recovery
of the game's position/attitude history interpolation.
"""
import math
from kernels import f32, add, sub, mul, div
from aero_vectors import columns


def sample_segment(start, end, times, lower, upper, position, attitude):
    start = list(map(f32, start)); end = list(map(f32, end))
    t0, t1 = map(f32, times); lower, upper = f32(lower), f32(upper)
    def clamp(t): return min(max(t, lower), upper)
    first, last = clamp(t0), clamp(t1)
    a, b = position(first), position(last)  # supplied double-precision history
    delta = [f32(y-x) for x,y in zip(a,b)]
    relative = [sub(e,add(s,d)) for s,e,d in zip(start,end,delta)]
    to_target = [sub(f32(x),s) for x,s in zip(a,start)]
    def dot(x,y): return add(add(mul(x[2],y[2]),mul(x[1],y[1])),mul(x[0],y[0]))
    length = f32(math.sqrt(dot(relative,relative)))
    reciprocal = div(1.,max(length,f32(.01)))
    offset = mul(mul(reciprocal,reciprocal),mul(sub(t1,t0),dot(to_target,relative)))
    attitude_time = clamp(add(max(offset,0.),t0))
    q = list(map(f32,attitude(attitude_time)))
    rotation = columns(q) if any(q) else [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    return dict(position_times=[first,last],attitude_time=attitude_time,
                first_time=first,end=[sub(e,d) for e,d in zip(end,delta)],
                matrix=[x for col in rotation for x in col]+list(map(f32,a)))
