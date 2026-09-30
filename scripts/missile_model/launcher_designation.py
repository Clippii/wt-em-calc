"""Recovered launcher designation conversion/selection, client 2.59.0.34.

14347de20 converts a supplied 80-byte record. 14347e1f0 selects from an ordered
list. Aircraft-side production of those records is outside this module.
"""
from copy import deepcopy
import math
from kernels import f32, add, sub, mul, div, safe_div


def initial():
    return dict(source=255, origin=[0., 0., 0.], direction=[0., 0., 0.],
                angular_rate=[0., 0., 0.], range_valid=False, range=0., radial_velocity=0.,
                closure_valid=False, closure=0., point=[0., 0., 0.], velocity=[0., 0., 0.],
                source_tag=0, flag55=False, target_id=0, padding=[0]*11)


def convert(frame, record, seed=None, *, lag_limit=.2):
    """Convert one selected record with canonical boolean input flags.

    point_valid selects the ranged-point branch; otherwise direction and its
    supplied derivative are extrapolated. velocity_valid controls closure.
    Output padding and all other unwritten bytes remain in seed.
    """
    out = deepcopy(initial() if seed is None else seed)
    limit = f32(lag_limit)
    lag = min(max(sub(frame['time'], record['time']), -limit), limit)
    base = record['origin'] if record['origin_override'] else frame['position']
    launcher_v = list(map(f32, frame['velocity']))
    relative_v = list(map(f32, record['relative_velocity']))
    origin = [add(mul(v, lag), o) for o,v in zip(base, launcher_v)]
    absolute_v = [add(a,b) for a,b in zip(relative_v, launcher_v)]
    if record['point_valid']:
        delta = [add(sub(mul(v, lag), o), p) for v,o,p in zip(absolute_v, origin, record['point'])]
        square = [mul(a,a) for a in delta]
        distance = max(f32(math.sqrt(add(add(square[2], square[1]), square[0]))), 1.)
        inverse = div(1., distance)
        direction = [mul(a, inverse) for a in delta]
        radial_velocity = add(mul(relative_v[2], direction[2]),
                              add(mul(relative_v[1], direction[1]), mul(relative_v[0], direction[0])))
    else:
        delta = [add(mul(v, lag), d) for v,d in zip(relative_v, record['direction'])]
        square = [mul(a,a) for a in delta]
        length = f32(math.sqrt(add(add(square[2], square[1]), square[0])))
        inverse = safe_div(1., length)
        direction = [mul(a, inverse) for a in delta]
        distance = radial_velocity = 0.
    rate = [sub(mul(relative_v[1], direction[2]), mul(relative_v[2], direction[1])),
            sub(mul(relative_v[2], direction[0]), mul(relative_v[0], direction[2])),
            sub(mul(relative_v[0], direction[1]), mul(relative_v[1], direction[0]))]
    if record['point_valid']:
        rate = [mul(a, inverse) for a in rate]
    closure = -add(mul(relative_v[0], direction[0]),
                   add(mul(relative_v[2], direction[2]), mul(relative_v[1], direction[1]))) if record['velocity_valid'] else 0.
    out.update(source=record['source'] & 255, origin=origin, direction=direction, angular_rate=rate,
               range_valid=bool(record['point_valid']), range=distance, radial_velocity=radial_velocity,
               closure_valid=bool(record['velocity_valid']), closure=closure,
               point=[add(mul(d, distance), o) for d,o in zip(direction, origin)], velocity=absolute_v,
               source_tag=record['source_tag'] & 255, flag55=bool(record['flag48']), target_id=record['target_id'])
    return out


def select(frame, records, primary_mask, fallback_mask, seed=None, *, lag_limit=.2):
    """First enabled primary match wins; fallback ignores the priority flag.

    BT uses the low five bits of each source tag. A miss preserves source_tag
    and output padding while initializing the defined no-designation fields.
    """
    for primary, mask in ((True, primary_mask), (False, fallback_mask)):
        for i, record in enumerate(records):
            if (not primary or record['priority']) and ((mask & 0xffffffff) >> (record['source'] & 31)) & 1:
                return dict(selected_index=i, primary=primary, output=convert(frame, record, seed, lag_limit=lag_limit))
    out = deepcopy(initial() if seed is None else seed)
    x,y,z,w = map(f32, frame['quaternion'])
    direction = [sub(add(add(mul(w,w), mul(x,x)), add(mul(w,w), mul(x,x))), 1.),
                 add(add(mul(y,x), mul(z,w)), add(mul(y,x), mul(z,w))),
                 add(sub(mul(x,z), mul(w,y)), sub(mul(x,z), mul(w,y)))]
    squares = [mul(a,a) for a in direction]
    inverse = safe_div(1., f32(math.sqrt(add(add(squares[2], squares[1]), squares[0]))))
    out.update(source=255, origin=list(map(f32, frame['position'])), direction=[mul(a,inverse) for a in direction],
               angular_rate=[0., 0., 0.], range_valid=False, range=0., radial_velocity=0.,
               closure_valid=False, closure=0., point=[0., 0., 0.], velocity=[0., 0., 0.],
               flag55=False, target_id=-1)
    return dict(selected_index=None, primary=None, output=out)
