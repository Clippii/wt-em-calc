"""Independent primary optical launcher for pinned client 2.59.0.34.

Cold activation, warm-up, non-designated acquisition/tracking, availability
gate and constructor handoff. Uses the explicit geometric_optical_v1 policy.
Aircraft command scheduling and designated/LOAL/dual-head paths are excluded.
All state is JSON serializable; update never mutates caller-owned histories.
"""
from copy import deepcopy
import math
import struct
from kernels import f32, sub
from optical_modes import update as seeker_update
from geometric_optical_observation import observe


def properties(rocket, guidance):
    """Combine authored launcher fields with the exported seeker properties."""
    if rocket.get('guidanceType') != 'optical':
        raise ValueError('Optical rocket required')
    config = rocket['guidance']
    unsupported = ('lockAfterLaunch', 'permanentlyActivated', 'inertialNavigation',
                   'applyExtraDifficultyParameters', 'opticalSeeker2')
    if any(config.get(key) for key in unsupported):
        raise NotImplementedError('Primary non-designated, single-head launcher required')
    if config['opticalSeeker'].get('designationRequired', False):
        raise NotImplementedError('Designation-required launcher is unsupported')
    work = f32(config.get('workTime', struct.unpack('<f', struct.pack('<I', 0x4efffeab))[0]))
    return dict(warm_up=f32(config.get('warmUpTime', 0.)), work_time=work,
                acquisition_time=f32(config.get('acquisitionTime', work)),
                uncage=bool(config.get('uncageBeforeLaunch', False)),
                seeker=deepcopy(guidance['seeker']), observation=deepcopy(guidance['observation']),
                los_check_timeout=f32(guidance['los_check_timeout']))


def initial():
    """Recovered launcher constructor state, before any reset or activation."""
    raw = [0]*32
    raw[16:20] = raw[28:32] = [255]*4
    seeker = dict(angles=[0., 0.], direction=[1., 0., 0.], angular_rate=[0., 0., 0.],
                  widths=[0., 0.], gap=0., distance=-1., los_check_time=0., los_cache_valid=False)
    return dict(phase=0, activation_time=-float.fromhex('0x1.fffffep127'), output_phase=-1,
                seeker=dict(state=seeker, opaque_state=raw))


def activate(state, time):
    """Invoke the recovered cold activation command at supplied absolute time."""
    if state['phase'] != 0:
        raise ValueError('Activation requires phase 0; command toggling is unsupported')
    if not math.isfinite(time):
        raise ValueError('Finite activation time required')
    out = deepcopy(state)
    out.update(phase=0x20, activation_time=f32(time))
    return out


def update(p, state, old, new, targets, *, available=True):
    """Advance with supplied 14-float launcher body records (position/q/v/w/time).

    Empty designation list, normal launcher context flags, no observation
    suppression. Availability false preserves histories and clears only the
    output phase. Timers use binary32 subtraction and strict greater-than.
    """
    if state['phase'] not in (0, 0x20, 0x30, 0x40):
        raise ValueError('Unsupported launcher phase')
    if len(old) != 14 or len(new) != 14 or not all(math.isfinite(x) for x in [*old, *new]):
        raise ValueError('Finite 14-float body records required')
    old_time, new_time = f32(old[13]), f32(new[13])
    if new_time < old_time:
        raise ValueError('Launcher time must not run backwards')
    out = deepcopy(state)
    result = dict(state=out, mode=None, phase_at_seeker=None, seeker_output=None, observations=[])
    if not available:
        out['output_phase'] = 0
        return result
    phase = out['phase']
    age = sub(new_time, out['activation_time'])
    if phase == 0x20 and age > p['warm_up']:
        phase = 0x30
        out['activation_time'] = new_time
    elif phase == 0x30 and age > p['acquisition_time']:
        phase = 0
    elif phase == 0x40 and age > p['work_time']:
        phase = 0
    mode = 0 if phase in (0, 0x20) else 4 if phase == 0x30 else 4 | int(p['uncage'])
    ss = out['seeker']
    raw = ss['opaque_state']
    q, origin = list(map(f32, new[3:7])), list(map(f32, new[:3]))

    def observation(temporary):
        os = dict(temporary, reject_time=struct.unpack('<f', bytes(raw[24:28]))[0],
                  target_id=struct.unpack('<i', bytes(raw[28:32]))[0])
        measured = observe(p['observation'], os, q, origin, targets,
                           dt=sub(new_time, old_time), new_time=new_time, collect=mode in (5, 8))
        result['observations'].append(measured)
        changed = raw[:]
        changed[24:28] = struct.pack('<f', measured['state']['reject_time'])
        changed[28:32] = struct.pack('<i', measured['state']['target_id'])
        if measured['accepted']:
            changed[0] = 0
        return dict(available=measured['accepted'], measurement=measured['measurement'],
                    distance=measured['state']['distance'], strength=measured['strength'],
                    flag=measured['flag'], count=measured['reported_target_count'],
                    auxiliary=measured['auxiliary'], opaque_state=changed,
                    refresh_los_cache=not temporary['los_cache_valid'] and measured['state']['los_cache_valid'])

    child = seeker_update(p['seeker'], ss['state'], q, old_time, new_time, observation,
                          mode=mode, opaque_state=raw, los_check_timeout=p['los_check_timeout'])
    out['seeker'] = {key: child[key] for key in ('state', 'opaque_state')}
    tracking = child['outputs']['tracking']
    out['phase'] = 0x40 if phase == 0x30 and tracking else 0x30 if phase == 0x40 and not tracking else phase
    out['output_phase'] = out['phase']
    # The launcher consumes these outputs. Failed acquisition may leave the
    # separate angle/rate scratch outputs untouched; the stored seeker history
    # above is the defined angle/rate state, including on failure.
    result.update(mode=mode, phase_at_seeker=phase,
                  seeker_output={key: child['outputs'][key] for key in
                                 ('tracking', 'strength', 'flag', 'count', 'auxiliary')})
    return result


def constructed_guidance(state, template):
    """Transfer launcher history into a supplied cold flight-constructor template.

    143eba687..6b7 copies 80 bytes (+0c..+5b of the seeker), then resets the
    integer at +4c and byte at +50. Visibility cache is newly initialized.
    Controller construction and supplied active status remain in the template.
    This path has no inertial/designation state. Call normal release initialization
    afterwards to apply launch spread, tags and controller initialization.
    """
    out = deepcopy(template)
    ss = deepcopy(state['seeker'])
    ss['state'].update(los_check_time=0., los_cache_valid=False)
    ss['opaque_state'][16:20] = [255]*4
    ss['opaque_state'][20] = 0
    out['seeker'] = ss
    out['manager'].update(mode=4, seeker_direction=ss['state']['direction'][:],
                          seeker_distance=ss['state']['distance'],
                          seeker_point_valid=bool(ss['opaque_state'][0]))
    return out
