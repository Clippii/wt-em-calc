"""Independent shared seeker arithmetic for War Thunder client 2.59.0.34.

Ports 143322ea0 (measurement) and 143323450 (coast). This component neither
generates observations nor decides lock survival. Frames: body measurement,
world filter direction/rate, body yaw/pitch, xyzw body-to-world quaternion.
Finite native CRT atan2f arithmetic is supplied by native_atan2f.
"""
import math
import struct
from kernels import f32, add, sub, mul, div
from native_atan2f import atan2f


def properties(config):
    """Fields consumed by these two routines, from native loader 143321740."""
    radians = f32(math.pi/180.)
    default_gate = struct.unpack('<f', struct.pack('<I', 0x4efffeab))[0]
    return dict(angle_max=mul(config.get('angleMax', 20.), radians),
                lock_angle_max=mul(config.get('lockAngleMax', 5.), radians),
                rate_max=mul(config.get('rateMax', 20.), radians),
                alpha=f32(config.get('filterAlpha', .85)),
                beta=f32(config.get('filterBetta', .2)),
                gate_rate=mul(config.get('angleGateRate', default_gate), radians))


def cross(a, b):
    return [sub(mul(a[1], b[2]), mul(a[2], b[1])),
            sub(mul(a[2], b[0]), mul(a[0], b[2])),
            sub(mul(a[0], b[1]), mul(a[1], b[0]))]


def normalize(v, phase):
    x, y, z = [mul(a, a) for a in v]
    if phase == 'predict':
        squared = add(add(z, y), x)
    elif phase == 'correct':
        squared = add(z, add(y, x))
    else:
        squared = add(add(z, x), y)
    length = f32(math.sqrt(squared))
    if phase == 'coast' and length <= 1e-9:
        return [1., 0., 0.]
    reciprocal = div(1., length) if length > f32(4e-19) else 0.
    return [mul(a, reciprocal) for a in v]


def world_residual(q, measurement, predicted):
    # The native code subtracts predicted before adding the diagonal term.
    # Rotating fully and then subtracting is algebraically, but not f32, equal.
    x, y, z, w = q
    mx, my, mz = measurement
    a = mul(add(z, z), x)
    b = mul(y, add(z, z))
    c = mul(add(z, z), w)
    d = mul(add(w, w), y)
    e = mul(w, add(x, x))
    f = mul(add(x, x), y)
    ww = sub(add(mul(w, w), mul(w, w)), 1.)
    xx = add(add(mul(x, x), mul(x, x)), ww)
    yy = add(add(mul(y, y), mul(y, y)), ww)
    zz = add(add(mul(z, z), mul(z, z)), ww)
    return [add(sub(add(mul(add(d, a), mz), mul(sub(f, c), my)), predicted[0]), mul(xx, mx)),
            add(sub(add(mul(sub(b, e), mz), mul(add(c, f), mx)), predicted[1]), mul(yy, my)),
            add(sub(add(mul(add(e, b), my), mul(sub(a, d), mx)), predicted[2]), mul(zz, mz))]


def coast_body(q, direction):
    x, y, z, w = q
    ux, uy, uz = direction
    a = mul(x, add(z, z))
    b = mul(add(z, z), y)
    nw = mul(-2., w)
    d = mul(y, nw)
    e = mul(x, nw)
    c = mul(nw, z)
    f = mul(add(x, x), y)
    ww = sub(add(mul(w, w), mul(w, w)), 1.)
    xx = add(add(mul(x, x), mul(x, x)), ww)
    yy = add(add(mul(y, y), mul(y, y)), ww)
    zz = add(add(mul(z, z), mul(z, z)), ww)
    return [add(mul(add(d, a), uz), add(mul(sub(f, c), uy), mul(xx, ux))),
            add(mul(sub(b, e), uz), add(mul(yy, uy), mul(add(f, c), ux))),
            add(mul(zz, uz), add(mul(add(e, b), uy), mul(sub(a, d), ux)))]


def slew(p, desired, angles, dt, lock_limit, authored_rate):
    x, y, z = desired
    targets = [atan2f(-z, x),
               atan2f(y, f32(math.sqrt(add(mul(z, z), mul(x, x)))))]
    step = mul(p['rate_max'] if authored_rate else 10., dt)
    limit = p['lock_angle_max'] if lock_limit else p['angle_max']
    out = []
    for previous, target in zip(angles, targets):
        change = min(step, max(-step, sub(target, previous)))
        change = min(sub(limit, previous), max(sub(-limit, previous), change))
        out.append(add(change, previous))
    return out


def update(p, quaternion, measurement, state, dt, *, lock_limit=False,
           authored_rate=True, coast=False):
    """Return a fresh state and acceptance flag for finite inputs and positive dt.

    `accepted` is the measurement filter's gate result, not a tracking flag.
    Both angles still slew on a rejected measurement. Coast leaves rate intact.
    """
    dt = f32(dt)
    if not math.isfinite(dt) or dt <= 0.:
        raise ValueError('Shared seeker reconstruction requires finite positive dt')
    q = list(map(f32, quaternion))
    angles = list(map(f32, state['angles']))
    u = list(map(f32, state['direction']))
    omega = list(map(f32, state['angular_rate']))
    predicted = normalize([add(mul(v, dt), old) for old, v in zip(u, cross(u, omega))],
                          'coast' if coast else 'predict')
    desired = coast_body(q, predicted) if coast else list(map(f32, measurement))
    new_angles = slew(p, desired, angles, dt, lock_limit, authored_rate)
    if coast:
        return dict(angles=new_angles, direction=predicted, angular_rate=omega, accepted=None)
    error = cross(world_residual(q, desired, predicted), predicted)
    gate = mul(p['gate_rate'], dt)
    accepted = all(abs(a) <= gate for a in error)
    direction = predicted
    if accepted:
        step = mul(p['rate_max'] if authored_rate else 10., dt)
        error = [min(step, max(-step, a)) for a in error]
        direction = normalize([add(mul(v, p['alpha']), old)
            for old, v in zip(predicted, cross(predicted, error))], 'correct')
        gain = div(p['beta'], dt)
        omega = [min(10., max(-10., add(mul(e, gain), old))) for old, e in zip(omega, error)]
    return dict(angles=new_angles, direction=direction, angular_rate=omega, accepted=accepted)
