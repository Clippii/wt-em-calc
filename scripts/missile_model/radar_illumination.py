"""SARH prelaunch source selection and frame assembly, native 1452ad1f0.

The 2.59.0.34 routine and its caller 1452ada80 are saved evidence. The direction
leaf is recovered from 2.59.0.38 at 143484b80: the old 14347dcf0 body was not
saved. Cross-build identity of that leaf is unproven; replay coverage against
old native frame packets must be distinguished from static old-code recovery.
Aircraft provider and actual sensor implementations remain explicit inputs.
"""
from copy import deepcopy
import math
from kernels import f32, add, sub, mul, div, safe_div


def designation_beam(body, record, *, lag_limit=.2):
    """2.59.0.38 leaf 143484b80, with an explicit loaded lag-compensation limit.

    Equivalent old call target 14347dcf0 lacks saved disassembly. This uses the
    direct point-origin plus relative-velocity expression; full designation
    conversion's separate origin/absolute velocity arithmetic is not identical.
    """
    limit = f32(lag_limit)
    lag = min(max(sub(body[13], record['time']), -limit), limit)
    velocity = record['relative_velocity']
    if (int(record['point_valid']) & 255) == 1:
        origin = record['origin'] if (int(record['origin_override']) & 255) != 0 else body[:3]
        delta = [add(sub(point, base), mul(v, lag))
                 for point, base, v in zip(record['point'], origin, velocity)]
    else:
        delta = [add(mul(v, lag), d) for v, d in zip(velocity, record['direction'])]
    square = [mul(d, d) for d in delta]
    inverse = safe_div(1., f32(math.sqrt(add(add(square[2], square[1]), square[0]))))
    return [mul(d, inverse) for d in delta]


def select_source(records, source_tag, *, prediction, inertial, datalink, reconnect_datalink):
    """First eligible 80-byte record, preserving the native byte comparisons.

    The inherited source tag constrains selection only when inertial navigation
    and datalink are both enabled and reconnectDatalink bit zero is clear.
    Prediction here means adapter mode 2, not a launcher-wide prediction flag.
    """
    tag = int(source_tag) & 255
    if ((int(inertial) & int(datalink) & 255) == 0 or
            (int(reconnect_datalink) & 1)):
        tag = 0
    for index, record in enumerate(records):
        flag = int(record['flag48']) & 255
        priority = int(record['priority']) & 255
        source = int(record['source']) & 255
        if prediction:
            eligible = flag != 0 or (source != 255 and (priority == 1 if tag else priority != 0))
        else:
            eligible = flag == 1 if tag else flag != 0
        if eligible and (not tag or (int(record['source_tag']) & 255) == tag):
            return index
    return None


def direction_frame(direction, origin):
    """1452ad42c..52f: binary32 axes with the double 1e-9 horizontal test.

    The returned direction is intentionally not normalized by this routine.
    Providers, including diagnostic providers, control its magnitude.
    """
    x, y, z = map(f32, direction)
    horizontal = f32(math.sqrt(add(mul(x, x), mul(z, z))))
    if horizontal > 1e-9:
        inverse = div(1., horizontal)
        right_x, right_z = mul(inverse, -z), mul(inverse, x)
    else:
        right_x, right_z = 0., 1.
    up = [mul(-y, right_z), sub(mul(x, right_z), mul(z, right_x)), mul(right_x, y)]
    return [x, y, z, *up, right_x, 0., right_z, *map(f32, origin)]


def absent():
    """Defined fields of the native invalid frame; output padding is not modeled."""
    return dict(present=False, frame=[1., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0.],
                velocity=[0., 0., 0.])


def illumination(records, source_tag, time, provider, source_direction,
                 *, prediction=False, inertial=False, datalink=False, reconnect_datalink=False,
                 direction_converter=designation_beam, lag_limit=.2):
    """Compose the original helper with explicit aircraft/provider services.

    provider is None or a mapping of available(), pose(), velocity() callbacks.
    pose returns position (three doubles) and quaternion (four floats).
    direction_converter(body14, record, lag_limit=...) uses the recovered .38
    direction leaf by default; callers may supply another verified conversion.
    source_direction(source_kind, time, vector) supplies 14065d030, which can be
    independently implemented by radar_source.direction with resolved sources.

    Repeated pose/velocity calls preserve native ordering and allow time-varying
    providers. The body uses the first pose position, second pose quaternion,
    first velocity, zero angular velocity, and the current unshifted body time.
    The output uses a third pose position unless the record overrides its origin,
    followed by a second velocity. The caller's projected launch offsets never
    enter this helper.
    """
    if provider is None or not provider['available']():
        return absent()
    index = select_source(records, source_tag, prediction=prediction, inertial=inertial,
                          datalink=datalink, reconnect_datalink=reconnect_datalink)
    if index is None:
        return absent()
    record = records[index]
    position = list(map(f32, provider['pose']()['position']))
    quaternion = list(map(f32, provider['pose']()['quaternion']))
    velocity = list(map(f32, provider['velocity']()))
    time = f32(time)
    body = [*position, *quaternion, *velocity, 0., 0., 0., time]
    beam = list(map(f32, direction_converter(body, deepcopy(record), lag_limit=lag_limit)))
    beam = list(map(f32, source_direction(int(record['source']) & 255, time, beam)))
    # Exact equality to one, as opposed to a truth test, at 1452ad52f.
    origin = (record['origin'] if (int(record['origin_override']) & 255) == 1 else
              provider['pose']()['position'])
    frame = direction_frame(beam, origin)
    return dict(present=True, frame=frame, velocity=list(map(f32, provider['velocity']())))
