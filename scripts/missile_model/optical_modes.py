"""Recovered optical reset, designation and acquisition transitions.

Pinned client 2.59.0.34, routine 143fe6c00. Observations remain supplied.
Modes 0..8 are supported with supplied designation direction/rate. Mode 7
requires scanHalfSector <= half-FOV; designated paths require surfaceAsTarget
false. Non-designated modes 5/8 delegate to optical_tracking.
"""
from copy import deepcopy
import math
from kernels import f32, add, sub, mul, safe_div
from optical_geometry import boresight
from optical_tracking import properties as tracking_properties, pre_slew_body, reseed
from optical_tracking import update as tracking_update
from shared_seeker import slew, update as shared_update
from seeker_designation import point as point_designation
from seeker_designation import masks as designation_masks, allowed as designation_is_allowed


def properties(config, *, reset_time_default=1.):
    p = tracking_properties(config, reset_time_default=reset_time_default)
    p.update(designation_masks(config))
    p.update(arret_angles=[mul(a, f32(math.pi/180.)) for a in config.get('arretAngles',[0.,0.])],
             scan_half_sector=mul(config.get('scanHalfSector',0.),f32(math.pi/180.)),
             surface_as_target=bool(config.get('surfaceAsTarget',False)),
             half_fov=mul(config.get('fov',config.get('angleHalfSens',5.)),f32(math.pi/360.)))
    return p


def update(p, state, quaternion, old_time, new_time, observe, *, mode,
           initial_angles=(0.,0.), opaque_state=None, designation_allowed=None,
           designation=None, additional_mask=0,
           authored_rate=True, predict_before_observation=True, suppress=False,
           los_check_timeout=-1.):
    """Preserve untouched output angles and opaque state bytes explicitly.

    opaque_state holds native +3c..+5b (32 bytes). Reset's +3c/+58 writes and
    disabled surface-target clearing at +3c/+40..+48 are interpreted here;
    observation can supply opaque_state as
    an explicit side effect. Mode 3 restores these bytes and distance after
    observation while retaining the separate visibility-cache effects.
    Callbacks receive known seeker state; they do not mutate it in place.
    With designation_allowed=None, authored masks plus additional_mask select
    the supplied designation source. An explicit boolean is a diagnostic override.
    Designation direction/rate must already be in missile body coordinates.
    Inputs are finite; quaternion normalization is not silently imposed.
    """
    dt = sub(new_time, old_time)
    if not math.isfinite(dt):
        raise ValueError('Finite seeker time interval required')
    mode = mode & 0xffffffff
    raw = list(opaque_state if opaque_state is not None else [0]*32)
    if len(raw)!=32 or any(not isinstance(v,int) or not 0<=v<=255 for v in raw):
        raise ValueError('opaque_state requires 32 bytes')
    q = list(map(f32,quaternion))
    out = deepcopy(state)
    for key in ('angles','direction','angular_rate','widths'):
        out[key] = list(map(f32,state[key]))
    for key in ('gap','distance','los_check_time'):
        out[key] = f32(state[key])
    outputs = dict(tracking=False,angles=list(map(f32,initial_angles)),angular_rate=[0.,0.,0.],
                   strength=0.,flag=False,count=0,auxiliary=[0.,0.,0.])
    result = dict(state=out,outputs=outputs,opaque_state=raw,observed=False,
                  observation_state=None,filter_accepted=None,reseeded=False,dt=dt)

    def park():
        out.update(angles=p['arret_angles'][:],direction=boresight(q,p['arret_angles']),
                   angular_rate=[0.,0.,0.],widths=[p['width_max']]*2)

    if mode==0 or dt<f32(1e-5):
        park()
        out.update(gap=0.,distance=-1.)
        raw[0]=0
        raw[28:32]=[255]*4
        return result
    if mode>8:
        return result
    if designation_allowed is None:
        designation_allowed=designation is not None and designation_is_allowed(mode,designation['source'],
            p['designation_mask'],p['constant_mask'],additional_mask)
    designated=designation_allowed and mode in (1,2,4,5,6,7)
    if designated:
        if designation is None:
            raise ValueError('Allowed designation requires body direction and angular rate')
        if mode in (4,5,7) and p['surface_as_target']:
            raise NotImplementedError('Designated surface-target generation is not reconstructed here')
        if mode==5:
            if add(los_check_timeout,out['los_check_time'])<f32(new_time):
                out['los_cache_valid']=False
            out['angles']=slew(p['shared'],pre_slew_body(q,out['direction']),out['angles'],
                               dt,True,authored_rate)
        shared=point_designation(p['shared'],q,designation['direction'],designation['angular_rate'],out['angles'])
        out['widths']=[p['width_max']]*2
        for key in ('angles','direction','angular_rate'):
            out[key]=shared[key]
        result['designation_accepted']=shared['accepted']
        if mode in (1,2,6):
            return result
        raw[0]=0
        if shared['accepted']:
            # 143fe12d0 with surfaceAsTarget false clears the cached point.
            raw[4:16]=[0]*12
    if mode==8 or (mode==5 and not designated):
        if add(los_check_timeout,out['los_check_time'])<f32(new_time):
            out['los_cache_valid']=False
        def active_observe(shared):
            temporary=deepcopy(out)
            temporary.update({key:shared[key] for key in ('angles','direction','angular_rate')})
            result['observation_state']=temporary
            observation=observe(deepcopy(temporary))
            if 'opaque_state' in observation:
                raw[:]=observation['opaque_state']
            return observation
        tracked = tracking_update(p,state,q,old_time,new_time,active_observe,
            authored_rate=authored_rate,predict_before_observation=predict_before_observation,
            suppress=suppress,los_check_timeout=los_check_timeout,mode=mode)
        tracked['opaque_state']=raw
        tracked['observation_state']=result['observation_state']
        return tracked
    if mode in (1,2,4) and not designated:
        park()
    if mode in (1,2):
        return result
    if mode in (3,4,7):
        out['los_cache_valid']=False
    if mode in (6,7) and not designated:
        out['angles']=slew(p['shared'],pre_slew_body(q,out['direction']),out['angles'],dt,False,False)
        coast=shared_update(p['shared'],q,None,out,dt,authored_rate=authored_rate,coast=True)
        for key in ('angles','direction','angular_rate'):
            out[key]=coast[key]
    if mode==6:
        return result
    if mode==7 and p['scan_half_sector']>p['half_fov']:
        raise NotImplementedError('Optical spiral-scan acquisition is not reconstructed here')
    observed=dict(available=False,strength=0.,flag=False,count=0,auxiliary=[0.,0.,0.])
    if not suppress:
        result['observed']=True
        result['observation_state']=deepcopy(out)
        observed.update(observe(deepcopy(out)))
    if observed.get('refresh_los_cache',False):
        out.update(los_check_time=f32(new_time),los_cache_valid=True)
    if mode!=3:
        if 'distance' in observed:
            out['distance']=f32(observed['distance'])
        if 'opaque_state' in observed:
            raw[:]=observed['opaque_state']
    for key in ('strength','flag','count','auxiliary'):
        outputs[key]=deepcopy(observed[key])
    outputs['strength']=f32(outputs['strength'])
    outputs['auxiliary']=list(map(f32,outputs['auxiliary']))
    outputs['tracking']=bool(observed['available'])
    if mode==3:
        return result
    if mode==5:
        # Constant-designation mode observes but does not run measurement/coast
        # filters or copy angle/rate outputs, irrespective of observation result.
        out['gap']=0. if observed['available'] else add(out['gap'],dt)
        outputs['tracking']=bool(observed['available']) or out['gap']<p['prolongation'] or suppress
        return result
    if not observed['available']:
        out['gap']=add(out['gap'],dt)
        return result
    measurement=list(map(f32,observed['measurement']))
    shared=reseed(p['shared'],q,measurement)
    shared=shared_update(p['shared'],q,measurement,shared,dt,
                         lock_limit=mode==4,authored_rate=authored_rate)
    for key in ('angles','direction','angular_rate'):
        out[key]=shared[key]
    inverse_distance=safe_div(1.,out['distance'])
    out['widths']=[min(p['width_max'],max(p['width_min'],mul(mul(v,inverse_distance),.5)))
                   for v in (observed['auxiliary'][2],observed['auxiliary'][1])]
    out['gap']=0.
    outputs.update(angles=out['angles'][:],angular_rate=out['angular_rate'][:])
    result.update(reseeded=True,filter_accepted=shared['accepted'])
    return result
