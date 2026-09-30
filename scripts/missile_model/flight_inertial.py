"""Native game's shared flight inertial helpers, recovered from client .38.

Updater 14529fff0 matches saved .34 instruction sequence 145298080 after
relocation. Projection 14529f6c0 is the relocated caller's previously unsaved
145297750 target; no claim of old/new projection byte identity is made.
The provider view, loaded properties, lag limit, and initialized noise tables
are explicit inputs. This module contains no native execution dependencies.
"""
from copy import deepcopy
import math

from kernels import f32, add, sub, mul, div, safe_div
from native_atan2f import atan2f
from noise_tables import initialize as initialize_noise_tables


_DEFAULT_NOISE_TABLES = initialize_noise_tables()


def square_zyx(v):
    return add(add(mul(v[2], v[2]), mul(v[1], v[1])), mul(v[0], v[0]))


def relative_rate(delta, velocity):
    """Velocity cross displacement / squared displacement, native operand order."""
    x, y, z = delta
    vx, vy, vz = velocity
    squared = square_zyx(delta)
    inverse = div(1., squared) if squared > f32(4e-19) else 0.
    return [mul(sub(mul(vy, z), mul(vz, y)), inverse),
            mul(sub(mul(vz, x), mul(vx, z)), inverse),
            mul(sub(mul(vx, y), mul(vy, x)), inverse)]


def body_direction(quaternion, vector):
    """Updater-specific inverse rotation; association differs from activation."""
    x, y, z, w = map(f32, quaternion)
    vx, vy, vz = vector
    z2 = add(z, z)
    xz = mul(x, z2)
    mw2 = mul(-2., w)
    wy, yz, wx, wz = mul(y, mw2), mul(z2, y), mul(x, mw2), mul(mw2, z)
    xy = mul(add(x, x), y)
    ww = add(add(mul(w, w), mul(w, w)), -1.)
    xx, yy, zz = [add(add(mul(a, a), mul(a, a)), ww) for a in (x, y, z)]
    return [add(add(mul(add(wy, xz), vz), mul(sub(xy, wz), vy)), mul(xx, vx)),
            add(mul(sub(yz, wx), vz), add(mul(yy, vy), mul(add(xy, wz), vx))),
            add(add(mul(zz, vz), mul(add(wx, yz), vy)), mul(sub(xz, wy), vx))]


def project(state, origin, velocity, provider_origin, provider_velocity, rate_limit):
    """Project complete inertial state into a possibly bistatic designation.

The returned validity preserves the input valid byte; zero selects fixed
defaults. The provider can coincide with the missile for monostatic geometry.
"""
    valid = int(state['valid']) & 255
    if not valid:
        return dict(valid=valid, direction=[1., 0., 0.], angular_rate=[0., 0., 0.],
                    range=0., closure=0.)
    point = [add(a, b) for a, b in zip(state['offset'], state['point'])]
    delta = [sub(a, b) for a, b in zip(point, origin)]
    squared = square_zyx(delta)
    inverse = div(1., squared) if squared > f32(4e-19) else 0.
    rate = relative_rate(delta, [sub(a, b) for a, b in zip(state['velocity'], velocity)])
    rate_squared = add(mul(rate[2], rate[2]), add(mul(rate[1], rate[1]), mul(rate[0], rate[0])))
    rate_limit = f32(rate_limit)
    if rate_squared > mul(rate_limit, rate_limit):
        scale = div(rate_limit, f32(math.sqrt(rate_squared)))
        rate = [mul(x, scale) for x in rate]
    direction = [mul(x, f32(math.sqrt(inverse))) for x in delta]
    provider_delta = [sub(a, b) for a, b in zip(point, provider_origin)]
    baseline = [sub(a, b) for a, b in zip(origin, provider_origin)]
    provider_range = f32(math.sqrt(square_zyx(provider_delta)))
    baseline_range = f32(math.sqrt(square_zyx(baseline)))
    provider_inverse = div(1., provider_range) if provider_range > f32(4e-19) else 0.
    baseline_inverse = div(1., baseline_range) if baseline_range > f32(4e-19) else 0.
    distance = mul(sub(add(provider_range, f32(math.sqrt(squared))), baseline_range), .5)
    pv = [sub(a, b) for a, b in zip(provider_velocity, state['velocity'])]
    provider_speed = mul(add(add(mul(pv[2], provider_delta[2]), mul(pv[1], provider_delta[1])),
                             mul(pv[0], provider_delta[0])), provider_inverse)
    mv = [sub(a, b) for a, b in zip(velocity, state['velocity'])]
    missile_speed = add(mul(mv[2], direction[2]), add(mul(mv[1], direction[1]), mul(mv[0], direction[0])))
    bv = [sub(a, b) for a, b in zip(velocity, provider_velocity)]
    baseline_speed = add(mul(add(mul(bv[0], baseline[0]), mul(bv[1], baseline[1])), baseline_inverse),
                         mul(mul(baseline[2], baseline_inverse), bv[2]))
    closure = mul(add(baseline_speed, add(missile_speed, provider_speed)), .5)
    return dict(valid=valid, direction=direction, angular_rate=rate, range=distance, closure=closure)


def noise(coordinate, permutation, gradients):
    """Native 1-D gradient interpolation, including mixed f32/f64 smoothing."""
    value = f32(coordinate)
    base = math.floor(value)
    fraction = sub(value, f32(base))
    smooth = f32((3. - 2.*fraction)*mul(fraction, fraction))
    left = mul(fraction, gradients[permutation[base & 255]])
    right = mul(add(fraction, -1.), gradients[permutation[(base+1) & 255]])
    return add(mul(sub(right, left), smooth), left)


def select_record(properties, state, records, channel_first, channel_last, source_mask):
    """Exact byte tests, signed channel limit, register bit-mask selection."""
    p = properties
    channel = channel_last if p['keep_channels_for_last'] else channel_first
    if (int(p['datalink']) & 255) != 1 or channel >= p['channels_max']:
        return None
    for index, record in enumerate(records):
        if not ((int(source_mask) & 0xffffffff) >> (int(record['source']) & 31)) & 1:
            continue
        if (int(p['reconnect']) & 1):
            eligible = (int(record['flag48']) & 255) == 1
        else:
            eligible = (int(record['source_tag']) & 255) == (int(state['source_tag']) & 255)
        if eligible and (int(record['point_valid']) & 255) != 0:
            return index
    return None


def update(properties, state, view, origin, quaternion, velocity, *, use_seeker,
           direction, angular_rate, range_valid, distance, radial_speed,
           designation_valid, designation_point, designation_velocity,
           channel_first, channel_last, source_mask, time, dt, angles, rate,
           lag_limit, noise_tables=None):
    """Advance full 44-byte state and preserve untouched output arguments.

``view`` contains records, fallback ``point`` and provider ``velocity``.
Seeker updates change state but leave angle/rate output storage untouched.
Without seeker tracking, output angles use the LOS from before the update;
output rates use the extrapolated point. Source tag and padding never change.
Drift defaults to independently recovered native fixed-seed startup tables.
Explicit tables allow initialized global state to be provided by a caller.
    """
    out = deepcopy(state)
    result = dict(state=out, angles=list(angles), rate=list(rate))
    selected = select_record(properties, state, view['records'], channel_first, channel_last, source_mask)
    result['selection'] = selected
    record = view['records'][selected] if selected is not None else None
    fallback = (record['origin'] if record is not None and (int(record['origin_override']) & 255) == 1
                else view['point'])
    offsets = list(state['offset'])
    old_point = [add(a, b) for a, b in zip(state['point'], offsets)]
    delta = [sub(a, b) for a, b in zip(old_point, origin)]
    length = f32(math.sqrt(square_zyx(delta)))
    if length > 1e-9:
        inv = div(1., length)
        los = [mul(d, inv) for d in delta]
    else:
        x, y, z, w = map(f32, quaternion)
        los = [add(add(add(mul(x, x), mul(w, w)), add(mul(x, x), mul(w, w))), -1.),
               add(add(mul(w, z), mul(y, x)), add(mul(w, z), mul(y, x))),
               add(sub(mul(x, z), mul(w, y)), sub(mul(x, z), mul(w, y)))]
    if use_seeker:
        out['valid'] = 1
        if range_valid:
            displacement = [mul(d, distance) for d in direction]
            out['point'] = [add(o, d) for o, d in zip(origin, displacement)]
            vx, vy, vz = map(f32, velocity)
            dx, dy, dz = map(f32, direction)
            wx, wy, wz = map(f32, angular_rate)
            rx, ry, rz = displacement
            residual = sub(radial_speed, add(mul(dz, vz), add(mul(vy, dy), mul(vx, dx))))
            out['velocity'] = [add(mul(residual, dx), add(sub(mul(wz, ry), mul(wy, rz)), vx)),
                               add(mul(dy, residual), add(sub(mul(wx, rz), mul(wz, rx)), vy)),
                               add(sub(add(mul(wy, rx), vz), mul(ry, wx)), mul(dz, residual))]
        else:
            height = f32(designation_point[1]) if designation_valid else old_point[1]
            scale = safe_div(sub(height, origin[1]), direction[1])
            out['point'] = [add(mul(direction[0], scale), origin[0]), height,
                            add(mul(direction[2], scale), origin[2])]
        out['offset'] = [0., 0., 0.]
        return result
    if designation_valid:
        out.update(valid=1, point=list(map(f32, designation_point)), velocity=list(map(f32, designation_velocity)))
    if record is not None:
        out['valid'] = 1
        target_velocity = [add(a, b) for a, b in zip(record['relative_velocity'], view['velocity'])]
        if (int(record['point_valid']) & 255) == 1:
            lag = min(max(sub(time, record['time']), -f32(lag_limit)), f32(lag_limit))
            out['point'] = [add(mul(v, lag), a) for v, a in zip(target_velocity, record['point'])]
        else:
            out['point'] = list(map(f32, fallback))
        out['velocity'] = target_velocity
    elif (int(out['valid']) & 255) != 1:
        result.update(angles=[0., 0.], rate=[0., 0., 0.])
        return result
    if not properties['gnss']:
        if noise_tables is None:
            noise_tables = _DEFAULT_NOISE_TABLES
        permutation, gradients = noise_tables['permutation'], noise_tables['gradients']
        drift_scale = mul(mul(f32(20.293798446655273), dt), properties['drift_speed'])
        offsets = [add(offset, mul(noise(mul(add(mul(axis, 60.), time), .3), permutation, gradients), drift_scale))
                   for offset, axis in zip(offsets, los)]
        out['offset'] = offsets
    out['point'] = [add(mul(v, dt), a) for v, a in zip(out['velocity'], out['point'])]
    displacement = [sub(add(a, b), o) for a, b, o in zip(out['point'], offsets, origin)]
    result['rate'] = relative_rate(displacement, [sub(a, b) for a, b in zip(out['velocity'], velocity)])
    body = body_direction(quaternion, los)
    result['angles'] = [atan2f(-body[2], body[0]), atan2f(body[1], f32(math.sqrt(add(mul(body[2], body[2]), mul(body[0], body[0])))))]
    return result
