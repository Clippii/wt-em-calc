"""Recovered acceleration-controller frames, selected by explicit game setting."""
import math
from kernels import f32, add, sub, mul, div


def length(values, order):
    a, b, c = (mul(values[i], values[i]) for i in order)
    return f32(math.sqrt(add(add(a, b), c)))


def normalized(values, order):
    n = length(values, order)
    inv = div(1., n) if n > f32(4e-19) else 0.
    return [mul(v, inv) for v in values]


def matrix_quaternion(forward, up, right):
    fx, fy, fz = forward; ux, uy, uz = up; rx, ry, rz = right
    trace_x = add(fx, 1.); trace_yz = add(rz, uy)
    squared = [sub(trace_x, trace_yz), sub(add(uy, 1.), add(fx, rz)),
        add(sub(1., add(fx, uy)), rz), add(trace_yz, trace_x)]
    magnitudes = [mul(f32(math.sqrt(v)), .5) if v > 0. else 0. for v in squared]
    best = f32(3.4028234663852886e38); chosen = [0.]*4
    for sx in (1., -1.):
        for sy in (1., -1.):
            for sz, sw in ((1., 1.), (1., -1.), (-1., 1.), (-1., -1.)):
                x, y, z, w = [mul(a, b) for a, b in zip(magnitudes, (sx, sy, sz, sw))]
                xy, wz, wy, xz, yz, wx = [mul(a, b) for a, b in ((x,y),(w,z),(w,y),(x,z),(y,z),(w,x))]
                differences = [sub(mul(2., sub(xy, wz)), ux), sub(mul(2., add(wy, xz)), rx),
                    sub(mul(2., add(xy, wz)), fy), sub(mul(2., sub(yz, wx)), ry),
                    sub(mul(2., sub(xz, wy)), fz), sub(mul(2., add(yz, wx)), uz)]
                a, b, c, d, e, f = [mul(v, v) for v in differences]
                if sz == sw:
                    error = add(add(add(d, b), add(c, a)), add(f, e))
                else:
                    error = add(add(f, e), add(add(d, c), add(b, a)))
                if error < best:
                    chosen = [x, y, z, w]; best = error
    return chosen


def frame(quaternion, velocity, velocity_reference=True,*,matrix=True):
    if not matrix:
        from alternate_control_frame import frame as alternate_frame
        return alternate_frame(quaternion,velocity,velocity_reference)
    q = list(map(f32, quaternion))
    if not velocity_reference:
        return q
    x, y, z, w = q
    forward = [add(mul(2., add(mul(x,x), mul(w,w))), -1.),
        mul(2., add(mul(w,z), mul(y,x))), mul(2., sub(mul(z,x), mul(w,y)))]
    forward = normalized(forward, (2, 1, 0))
    velocity = list(map(f32, velocity)); speed = length(velocity, (0, 1, 2))
    if speed > 1.e-9:
        inv = div(1., speed); forward = [mul(v, inv) for v in velocity]
    up = [mul(2., sub(mul(y,x), mul(w,z))), add(mul(2., add(mul(w,w), mul(y,y))), -1.),
        mul(2., add(mul(w,x), mul(z,y)))]
    up = normalized(up, (2, 0, 1))
    fx, fy, fz = forward; ux, uy, uz = up
    right = [sub(mul(uz,fy), mul(uy,fz)), sub(mul(ux,fz), mul(fx,uz)), sub(mul(fx,uy), mul(fy,ux))]
    right = normalized(right, (1, 0, 2))
    rx, ry, rz = right
    new_up = [sub(mul(ry,fz), mul(rz,fy)), sub(mul(rz,fx), mul(rx,fz)), sub(mul(rx,fy), mul(ry,fx))]
    ux, uy, uz = new_up
    positive = add(add(mul(mul(rx,fy),uz), mul(mul(rz,fx),uy)), mul(mul(ry,fz),ux))
    negative = add(mul(mul(ry,fx),uz), add(mul(mul(rz,fy),ux), mul(mul(rx,fz),uy)))
    if positive < negative:
        right = [-v for v in right]
    return matrix_quaternion(forward, new_up, right)
