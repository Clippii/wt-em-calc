"""Isolated finite-input mode rules from game routine 0x14529b130.

Loaded properties and caller tracking inputs are required. This is neither a
trajectory simulator nor a real-world missile guidance model.
"""
import math
from kernels import f32, add, sub, mul, safe_div


def transition(mode, p, los, rate, q, time_to_hit, gain):
    if f32(gain) < f32(.001):
        return mode
    x, y, z = map(f32, los)
    rx, ry, rz = map(f32, rate)
    qx, qy, qz, qw = map(f32, q)
    elevation = mul(abs(y), y)
    horizontal = add(mul(z, z), mul(x, x))
    if mode == 2:
        # Projection as evaluated by this routine; preserve float32 ordering.
        a = mul(add(mul(qy, qw), mul(qx, qz)), rx)
        b = mul(sub(mul(qy, qz), mul(qx, qw)), ry)
        c = mul(add(mul(2., add(mul(qw, qw), mul(qz, qz))), -1.), rz)
        projected = abs(add(mul(2., add(b, a)), c))
        if (elevation >= mul(horizontal, p['hold_elevation'])
                and projected <= p['hold_rate']
                and f32(time_to_hit) >= p['hold_time']):
            return 2
        mode = 1 if p['loft_enabled'] else 0
    if mode == 1:
        norm = add(mul(rz, rz), add(mul(ry, ry), mul(rx, rx)))
        if elevation < mul(horizontal, p['loft_elevation']) or norm > p['loft_rate_squared']:
            return 0
    return mode


def loft_acceleration(p, q, range_squared, ordinary_y):
    x0, y0, x1, y1 = p['range_elevation']
    if x0 > x1:
        x0, y0, x1, y1 = x1, y1, x0, y0
    distance = f32(range_squared)
    if distance <= x0:
        angle = y0
    elif distance >= x1:
        angle = y1
    else:
        angle = add(y0, safe_div(mul(sub(distance, x0), sub(y1, y0)), sub(x1, x0)))
    x, y, z, w = map(f32, q)
    a = mul(2., sub(mul(x, z), mul(w, y)))
    b = mul(2., add(mul(z, w), mul(y, x)))
    c = add(mul(2., add(mul(w, w), mul(x, x))), -1.)
    length = f32(math.sqrt(add(add(mul(a, a), mul(b, b)), mul(c, c))))
    inv = safe_div(1., length) if length > f32(4e-19) else 0.
    a, b, c = (mul(v, inv) for v in (a, b, c))
    pitch = f32(math.atan2(b, f32(math.sqrt(add(mul(a, a), mul(c, c))))))
    return max(mul(sub(angle, pitch), p['loft_accel']), f32(ordinary_y))
