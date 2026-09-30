"""Recovered game launcher inertial-state builder 145297d80, client 2.59.0.34.

This is the shared native optical/radar state helper. Caller eligibility,
designation production and flight extrapolation are separate operations.
"""
from copy import deepcopy
from kernels import f32, add, sub, mul, safe_div


def initial():
    return dict(valid=False, point=[0., 0., 0.], velocity=[0., 0., 0.],
                offset=[0., 0., 0.], source_tag=0, padding=[0]*6)


def update(state, origin, velocity, direction, angular_rate, *, use_seeker,
           range_valid, distance, radial_velocity, designation_valid,
           designation_point, designation_velocity, source_tag):
    """Retain untouched values on invalidation; clear offset on every call.

    Vectors are in world coordinates. Without seeker range, the native helper
    intersects its direction with the supplied designation point's y plane.
    The near-zero vertical-direction division guard is preserved exactly.
    """
    out = deepcopy(state)
    o, v, d, w, point, designated_v = [list(map(f32, x)) for x in
        (origin, velocity, direction, angular_rate, designation_point, designation_velocity)]
    distance, radial_velocity = f32(distance), f32(radial_velocity)
    if use_seeker:
        out['valid'] = True
        if range_valid:
            out['point'] = [add(mul(a, distance), b) for a,b in zip(d, o)]
            cross = [sub(mul(w[2], d[1]), mul(w[1], d[2])),
                     sub(mul(w[0], d[2]), mul(w[2], d[0])),
                     sub(mul(w[1], d[0]), mul(w[0], d[1]))]
            cross = [mul(a, distance) for a in cross]
            out['velocity'] = [add(add(mul(d[0], radial_velocity), cross[0]), v[0]),
                               add(add(mul(d[1], radial_velocity), v[1]), cross[1]),
                               add(add(mul(d[2], radial_velocity), cross[2]), v[2])]
        else:
            ratio = safe_div(sub(point[1], o[1]), d[1])
            out['point'] = [add(mul(d[0], ratio), o[0]), point[1], add(mul(d[2], ratio), o[2])]
            dx, dy, dz = [sub(a,b) for a,b in zip(out['point'], o)]
            projected = add(mul(d[2], v[2]), add(mul(d[1], v[1]), mul(d[0], v[0])))
            out['velocity'] = [
                sub(add(mul(w[2], dy), v[0]), add(mul(projected, d[0]), mul(dz, w[1]))),
                sub(add(mul(w[0], dz), v[1]), add(mul(d[1], projected), mul(dx, w[2]))),
                sub(add(mul(dx, w[1]), v[2]), add(mul(d[2], projected), mul(dy, w[0])))]
    elif designation_valid:
        out.update(valid=True, point=point, velocity=designated_v)
    else:
        out['valid'] = False
    out.update(offset=[0., 0., 0.], source_tag=source_tag & 255)
    return out
