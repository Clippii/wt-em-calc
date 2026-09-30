"""Radar launcher activation, native 1452ad610, pinned client 2.59.0.34.

Consumes a supplied complete seeker history and aircraft designation records.
This command is independent of the still separate continuous prelaunch update.
"""
from copy import deepcopy
from kernels import f32, add, sub, mul
from launcher_designation import select
from radar_designation import properties as seeker_properties, reseed
from seeker_designation import source_mask


def properties(rocket):
    config = rocket['guidance']
    seeker = config['radarSeeker']
    signature = seeker.get('targetSignatureType', seeker.get('visibilityType', 'radar'))
    if signature != 'radar':
        raise NotImplementedError('Conventional radar seeker required')
    # 143fe9de3..9e2a: radar signature defaults to source bit 6.
    primary = source_mask(seeker.get('designationSourceType'),
                          seeker.get('designationSourceTypeMask', 0x40)) & 0xffff
    return dict(permanent=bool(config.get('permanentlyActivated', False)),
                lock_after_launch=bool(config.get('lockAfterLaunch', False)),
                primary_mask=primary, fallback_mask=primary & 0xff1f,
                seeker=seeker_properties(seeker))


def world_to_body(quaternion, vector):
    """Activation's inverse quaternion rotation with its float32 operand order."""
    x, y, z, w = map(f32, quaternion)
    vx, vy, vz = map(f32, vector)
    zz2 = add(z,z)
    xz = mul(x,zz2)
    minus_w2 = mul(-2.,w)
    wy = mul(y,minus_w2)
    yz = mul(zz2,y)
    wx = mul(x,minus_w2)
    wz = mul(minus_w2,z)
    xy = mul(add(x,x),y)
    ww = add(add(mul(w,w),mul(w,w)),-1.)
    xx = add(add(mul(x,x),mul(x,x)),ww)
    yy = add(add(mul(y,y),mul(y,y)),ww)
    zz = add(add(mul(z,z),mul(z,z)),ww)
    return [add(add(mul(add(wy,xz),vz),mul(sub(xy,wz),vy)),mul(xx,vx)),
            add(add(mul(sub(yz,wx),vz),mul(add(xy,wz),vx)),mul(yy,vy)),
            add(mul(zz,vz),add(mul(add(wx,yz),vy),mul(sub(xz,wy),vx)))]


def seeker_designation(frame, selected):
    """Activation always supplies a designation and forces associated ID zero."""
    return dict(present=True,
                direction=world_to_body(frame['quaternion'], selected['direction']),
                angular_rate=world_to_body(frame['quaternion'], selected['angular_rate']),
                range_valid=selected['range_valid'], range=selected['range'],
                closure_valid=bool(selected['closure_valid'] or selected['range_valid']),
                closure=selected['closure'] if selected['closure_valid'] else -selected['radial_velocity'],
                target_id=0)


def activate(p, state, frame, records, *, enabled=True, lag_limit=.2):
    """Preserve all unrelated launcher state and return command diagnostics.

    Permanent activation precedes the enabled switch, does not reseed, and
    preserves activation time. Ordinary disable changes only the phase.
    Enabling can reseed an already active launcher; it is not cold-only.
    """
    out = deepcopy(state)
    result = dict(state=out, selection=None, designation=None)
    if p['permanent']:
        out['phase'] = 0x30 + 0x20*int(p['lock_after_launch'])
    elif not enabled:
        out['phase'] = 0
    else:
        selection = select(frame, records, p['primary_mask'], p['fallback_mask'], lag_limit=lag_limit)
        designation = seeker_designation(frame, selection['output'])
        out['seeker'] = reseed(p['seeker'], out['seeker'], frame, designation)
        out['seeker']['los_cache_valid'] = False
        out['phase'] = 0x20
        out['activation_time'] = f32(frame['time'])
        result.update(selection=selection, designation=designation)
    return result
