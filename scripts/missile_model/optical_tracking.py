"""Independent active optical seeker transitions from aces.exe 2.59.0.34.

Observation generation is an explicit callback, not yet an independent scene
model. Mode 5 requires no allowed constant designation. Initialization and
acquisition paths are implemented separately in optical_modes.
"""
from copy import deepcopy
import math
from native_atan2f import atan2f
from kernels import f32, add, sub, mul, safe_div
from motor_vector import rotate_thrust
from shared_seeker import properties as shared_properties, slew, update as shared_update


def properties(config, *, reset_time_default=1.):
    half_degree = f32(math.pi/360.)
    return dict(shared=shared_properties(config),
        width_max=f32(math.sin(mul(config.get('fov', config.get('angleHalfSens', 5.)), half_degree))),
        width_min=f32(math.sin(mul(config.get('gateWidth', 90.), half_degree))),
        prolongation=f32(config.get('prolongationTimeMax', 1.)),
        reset_time=f32(config.get('trackResetTimeMax', reset_time_default)))


def pre_slew_body(q, direction):
    # The inline mode-8 inverse transform uses the force-adapter sum order,
    # unlike the shared coast routine's inverse transform.
    x, y, z, w = q
    return rotate_thrust([-x, -y, -z, w], direction)


def reseed(p, q, measurement):
    """143322990 with zero supplied body angular history, as mode 8 calls it."""
    x, y, z = measurement
    limit = p['lock_angle_max']
    angles = [atan2f(-z, x),
              atan2f(y, f32(math.sqrt(add(mul(z,z), mul(x,x)))))]
    return dict(angles=[min(limit, max(-limit, a)) for a in angles],
                direction=rotate_thrust(q, measurement),
                # Native reseed rotates even zero body history; the quaternion
                # arithmetic can retain negative zero in the world rate.
                angular_rate=rotate_thrust(q, [0.,0.,0.]))


def update(p, state, quaternion, old_time, new_time, observe, *,
           authored_rate=True, predict_before_observation=True, suppress=False,
           los_check_timeout=-1., mode=8):
    """Execute mode 8, or non-designated mode 5, above the reset threshold.

    `observe` receives the temporary shared state and returns a dict with
    available, measurement (body LOS), distance, strength, flag, count,
    auxiliary (three native observation output values). The distance write is
    an explicit supplied observation effect. It may be omitted on failure.
    refresh_los_cache reports an observation cache refresh at 143fe5980 or
    143fe61ca (the latter through an alias of +5c). Both set +60 and stamp +5c
    with new_time; this is distinct from generic observation availability.

    `los_check_time`/`los_cache_valid` preserve native +5c/+60 visibility-cache
    fields; validity does not imply clear visibility. State uses shared fields plus widths,
    gap, distance and these two values. No input objects are mutated.
    """
    dt = sub(new_time, old_time)
    if mode not in (5,8):
        raise ValueError('Active optical transition requires mode 5 or 8')
    lock_limit = mode==5
    if not math.isfinite(dt) or dt < f32(1e-5):
        raise NotImplementedError('Mode-8 short-step reset must use its separate recovered path')
    q = list(map(f32, quaternion))
    out = deepcopy(state)
    out['widths'] = list(map(f32, state['widths']))
    for key in ('gap', 'distance', 'los_check_time'):
        out[key] = f32(state[key])
    p_shared = p['shared']
    shared = {key:list(map(f32, state[key])) for key in ('angles','direction','angular_rate')}
    if add(los_check_timeout, state['los_check_time']) < f32(new_time):
        out['los_cache_valid'] = False
    shared['angles'] = slew(p_shared, pre_slew_body(q, shared['direction']), shared['angles'],
                           dt, lock_limit, authored_rate if lock_limit else False)
    temporary = shared
    if predict_before_observation:
        temporary = shared_update(p_shared, q, None, shared, dt,
                                  authored_rate=authored_rate, lock_limit=lock_limit, coast=True)
    observed = dict(available=False, strength=0., flag=False, count=0, auxiliary=[0.,0.,0.])
    if not suppress:
        observed.update(observe(deepcopy(temporary)))
    if 'distance' in observed:
        out['distance'] = f32(observed['distance'])
    if observed.get('refresh_los_cache', False):
        out['los_check_time'] = f32(new_time)
        out['los_cache_valid'] = True
    filter_accepted = None
    reseeded = False
    if observed['available']:
        measurement = list(map(f32, observed['measurement']))
        if f32(state['gap']) > p['reset_time']:
            shared = reseed(p_shared, q, measurement)
            reseeded = True
        out['gap'] = 0.
        shared = shared_update(p_shared, q, measurement, shared, dt,
                               authored_rate=authored_rate, lock_limit=lock_limit)
        filter_accepted = shared['accepted']
        half_inverse_distance = mul(safe_div(1., out['distance']), .5)
        out['widths'] = [min(p['width_max'], max(p['width_min'], mul(v, half_inverse_distance)))
                         for v in (observed['auxiliary'][2], observed['auxiliary'][1])]
        if lock_limit:
            out['widths'] = [min(p['width_max'],max(p['width_min'],mul(mul(v,safe_div(1.,out['distance'])),.5)))
                             for v in (observed['auxiliary'][2],observed['auxiliary'][1])]
        tracking = True
    else:
        out['gap'] = add(state['gap'], dt)
        shared = shared_update(p_shared, q, None, shared, dt,
                               authored_rate=authored_rate, lock_limit=lock_limit, coast=True)
        tracking = out['gap'] < p['prolongation'] or suppress
    for key in ('angles','direction','angular_rate'):
        out[key] = shared[key]
    outputs = dict(tracking=tracking, angles=out['angles'][:], angular_rate=out['angular_rate'][:],
                   strength=f32(observed['strength']), flag=bool(observed['flag']),
                   count=int(observed['count']), auxiliary=list(map(f32, observed['auxiliary'])))
    return dict(state=out, outputs=outputs, filter_accepted=filter_accepted,
                reseeded=reseeded, observation_state=temporary, observed=not suppress,
                observation_available=bool(observed['available']), dt=dt)
