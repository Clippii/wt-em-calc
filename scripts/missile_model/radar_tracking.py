"""Independent radar mode-8 transition with explicitly supplied observations.

Client 2.59.0.34, 143fed5a0 active branch. Single authored observation mode,
positive non-reset timestep. Aircraft/provider and scene effects are inputs.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,div
from shared_seeker import slew
from optical_tracking import pre_slew_body,reseed
from radar_tracking_filters import properties as filter_properties,update as measurement_update,coast


def properties(config):
    p=filter_properties(config)
    p.update(prolongation=f32(config.get('prolongationTimeMax',1.)),
             reset_time=f32(config.get('trackResetTimeMax',1.)))
    return p


def update(p,state,old_frame,new_frame,observe,limit_age,*,suppress=False,authored_rate=True,
           include_gap_after_reject=True,los_check_timeout=-1.,initial_extra_flags=0):
    """Frames contain position, velocity, quaternion and time, in native units.

    Callback sees the corrected range/closure and pre-slewed head angles.
    It may explicitly refresh the visibility cache; other provider state remains
    outside this component. Only one authored observation mode is supported.
    """
    dt=sub(new_frame['time'],old_frame['time'])
    if not math.isfinite(dt) or dt<f32(1e-5):raise NotImplementedError('Radar short-step reset is separate')
    out=deepcopy(state);q=list(map(f32,new_frame['quaternion']))
    for key in ('angles','direction','angular_rate'):out[key]=list(map(f32,out[key]))
    for key in ('gap','range','range_rate','closure','closure_rate','los_check_time'):out[key]=f32(out[key])
    if add(los_check_timeout,out['los_check_time'])<f32(new_frame['time']):out['los_cache_valid']=False
    d=out['direction']
    dp=[mul(sub(a,b),u) for a,b,u in zip(new_frame['position'],old_frame['position'],d)]
    projected_position=add(dp[2],add(dp[1],dp[0]))
    out['range']=sub(out['range'],projected_position)
    dv=[mul(sub(a,b),u) for a,b,u in zip(new_frame['velocity'],old_frame['velocity'],d)]
    projected_velocity=add(add(dv[2],dv[1]),dv[0])
    out['closure']=add(out['closure'],projected_velocity)
    out['angles']=slew(p['shared'],pre_slew_body(q,d),out['angles'],dt,False,False)
    observation_state=deepcopy(out)
    observed=dict(available=False,measurement=[0.,0.,0.],range=0.,closure=0.,strength=0.,extra_flags=initial_extra_flags)
    if not suppress:
        observed.update(observe(deepcopy(out)))
        out.update(observed.get('state_effects',{}))
        out['channel_index']=0
    if observed.get('refresh_los_cache',False):
        out.update(los_check_time=f32(new_frame['time']),los_cache_valid=True)
    filtered=None;reseeded=False
    if observed['available']:
        if out['gap']>p['reset_time']:
            out.update(reseed(p['shared'],q,observed['measurement']));reseeded=True
        filtered=measurement_update(p,out,q,observed['measurement'],observed['range'],observed['closure'],
            dt,limit_age,authored_rate=authored_rate,include_gap_after_reject=include_gap_after_reject,
            initial_angles=(0.,0.),initial_rate=(0.,0.,0.))
        out=filtered['state']
    accepted=filtered is not None and filtered['accepted']
    if accepted:
        d=out['direction']
        if not p['doppler']['present']:
            old_terms=[mul(v,u) for v,u in zip(old_frame['velocity'],d)]
            previous_speed=add(old_terms[2],add(old_terms[1],old_terms[0]))
            acceleration=0.
            if dt>f32(4e-19):
                acceleration=div(sub(add(out['closure'],out['range_rate']),
                                     add(projected_velocity,previous_speed)),dt)
            out['closure_rate']=acceleration
            terms=[mul(v,u) for v,u in zip(new_frame['velocity'],d)]
            out['closure']=add(sub(terms[0],out['range_rate']),add(terms[2],terms[1]))
        if not p['distance']['present']:
            out['range']=sub(add(projected_position,out['range']),mul(dt,out['closure']))
            terms=[mul(v,u) for v,u in zip(new_frame['velocity'],d)]
            out['range_rate']=add(sub(terms[0],out['closure']),add(terms[2],terms[1]))
        out['gap']=0.;outputs=filtered['outputs'];tracking=True
    else:
        coasted=coast(p,out,q,dt,authored_rate=authored_rate)
        out=coasted['state'];outputs=coasted['outputs']
        out['gap']=add(out['gap'],dt)
        tracking=out['gap']<p['prolongation'] or suppress
    outputs.update(tracking=bool(tracking),strength=f32(observed['strength']),extra_flags=observed['extra_flags'])
    return dict(state=out,outputs=outputs,observed=not suppress,observation_state=observation_state,
                filter_accepted=None if filtered is None else filtered['accepted'],reseeded=reseeded)
