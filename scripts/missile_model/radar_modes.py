"""Independent radar reset, designation, acquisition and coast transitions.

Client 2.59.0.34, 143fed5a0. Modes 0..6 and 8, with supplied designation
and observations. Mode 7's scene search remains separate.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul
from optical_geometry import boresight
from optical_tracking import pre_slew_body,reseed
from shared_seeker import slew
from radar_tracking import properties as tracking_properties,update as tracking_update
from radar_tracking_filters import rounded_state,update as filter_update,coast
from radar_designation import properties as designation_properties,apply as apply_designation


def properties(config):
    p=tracking_properties(config)
    p.update(arret_angles=[mul(a,f32(math.pi/180.)) for a in config.get('arretAngles',[0.,0.])],
             use_target_id=bool(config.get('useTargetId',False)))
    p['designation']=designation_properties(config)
    for name,key in (('distance','distance'),('doppler','dopplerSpeed')):
        c=config.get(key,{})
        p[name]['initial']=add(p[name]['minimum'],mul(.5,
            c.get('signalWidthMin',mul(p[name]['width'],.1)))) if p[name]['present'] else 0.
    return p


def update(p,state,old_frame,new_frame,observe,limit_age,*,mode,designation_present=False,designation=None,
           suppress=False,authored_rate=True,include_gap_after_reject=True,los_check_timeout=-1.,
           initial_angles=(0.,0.),initial_extra_flags=0):
    """Keep output angles/extra flags that the native branch does not overwrite.

    State extends radar_tracking with associated_id (+3c), target_id (+44),
    target_value (+48), visibility_clear (+51), visibility_weight (+54).
    Observation state_effects are explicit provider writes, not inferred from
    observation availability. Unknown bytes remain the caller's responsibility.
    """
    dt=sub(new_frame['time'],old_frame['time']);mode=mode&0xffffffff
    if not math.isfinite(dt):raise ValueError('Finite radar time interval required')
    out=rounded_state(state);q=list(map(f32,new_frame['quaternion']))
    for key in ('los_check_time','target_value','visibility_weight'):out[key]=f32(out[key])
    outputs=dict(tracking=False,angles=list(map(f32,initial_angles)),angular_rate=[0.,0.,0.],
                 strength=0.,extra_flags=initial_extra_flags)
    result=dict(state=out,outputs=outputs,observed=False,observation_state=None,
                filter_accepted=None,reseeded=False)

    def park():
        out.update(angles=p['arret_angles'][:],direction=boresight(q,p['arret_angles']),
            angular_rate=[0.,0.,0.],channel_index=0,range_valid=False,range=p['distance']['initial'],range_rate=0.,
            closure_valid=False,closure=p['doppler']['initial'],closure_rate=0.,associated_id=-1)

    if mode==0 or dt<f32(1e-5):
        park();out.update(gap=0.,target_id=-1,target_value=-1.)
        return result
    if mode not in (1,2,3,4,5,6,8):raise NotImplementedError('Radar scene-search/unknown mode remains separate')
    designated=designation_present and mode!=8
    if designated and designation is None:raise ValueError('Designation-present mode requires a supplied designation')
    if designated:
        if mode==2:out.update(los_cache_valid=True,visibility_clear=True,visibility_weight=1.)
        elif mode==3:out['los_cache_valid']=False
        elif mode==4:
            if add(los_check_timeout,out['los_check_time'])<f32(new_frame['time']):out['los_cache_valid']=False
            out['angles']=slew(p['shared'],pre_slew_body(q,out['direction']),out['angles'],dt,True,authored_rate)
        saved=deepcopy(out)
        designated_result=apply_designation(p['designation'],out,new_frame,designation,dt)
        out.update(designated_result['state']);result['designation_accepted']=designated_result['accepted']
        if mode==1:
            outputs['tracking']=designated_result['accepted']
            return result
        if mode in (5,6):
            out['gap']=add(out['gap'],dt)
            if mode==6 and out['gap']>p['prolongation']:out['associated_id']=-1
            return result
        observed=dict(available=False,strength=0.,extra_flags=initial_extra_flags,target_id=-1)
        result['observation_state']=deepcopy(out)
        if not suppress and (mode==2 or designated_result['accepted']):
            result['observed']=True;observed.update(observe(deepcopy(out)))
            out.update(observed.get('state_effects',{}))
            out['channel_index']=0
        if observed.get('refresh_los_cache',False):
            out.update(los_check_time=f32(new_frame['time']),los_cache_valid=True)
        outputs.update(strength=f32(observed['strength']),extra_flags=observed['extra_flags'])
        if mode==2:
            visibility={k:out[k] for k in ('los_check_time','los_cache_valid','visibility_clear','visibility_weight')}
            out.update(saved);out.update(visibility);outputs['tracking']=designated_result['accepted']
        elif mode==3:
            outputs['tracking']=bool(observed['available'])
            if observed['available']:
                out['gap']=0.
                if p['use_target_id']:out['associated_id']=observed['target_id']
            else:
                out['gap']=add(out['gap'],dt)
                if out['gap']>=p['prolongation'] and out['associated_id']!=-1:park()
        else:
            out['gap']=0. if observed['available'] else add(out['gap'],dt)
            outputs['tracking']=observed['available'] or out['gap']<p['prolongation'] or suppress
        return result
    if mode==1:
        park();outputs['tracking']=True
        return result
    if mode in (4,8):
        if mode==4:
            if add(los_check_timeout,out['los_check_time'])<f32(new_frame['time']):out['los_cache_valid']=False
            out['angles']=slew(p['shared'],pre_slew_body(q,out['direction']),out['angles'],dt,True,authored_rate)
        return tracking_update(p,out,old_frame,new_frame,observe,limit_age,suppress=suppress,
            authored_rate=authored_rate,include_gap_after_reject=include_gap_after_reject,
            los_check_timeout=los_check_timeout,initial_extra_flags=initial_extra_flags)
    if mode in (5,6):
        d=out['direction']
        out['angles']=slew(p['shared'],pre_slew_body(q,d),out['angles'],dt,False,False)
        dp=[mul(sub(a,b),v) for a,b,v in zip(old_frame['position'],new_frame['position'],d)]
        out['range']=add(add(add(dp[2],dp[1]),out['range']),dp[0])
        dv=[mul(sub(a,b),v) for a,b,v in zip(new_frame['velocity'],old_frame['velocity'],d)]
        out['closure']=add(add(add(dv[2],dv[1]),out['closure']),dv[0])
        coasted=coast(p,out,q,dt,authored_rate=authored_rate);out.update(coasted['state'])
        outputs.update(coasted['outputs']);out['gap']=add(out['gap'],dt)
        if mode==6 and out['gap']>p['prolongation']:out['associated_id']=-1
        return result
    # Mode 2 restores +0..+4b after observation; visibility state lies outside.
    saved=deepcopy(out)
    if mode==2:out.update(los_cache_valid=True,visibility_clear=True,visibility_weight=1.)
    else:out['los_cache_valid']=False
    park()
    result['observation_state']=deepcopy(out)
    observed=dict(available=False,strength=0.,extra_flags=initial_extra_flags,target_id=-1)
    if not suppress:
        result['observed']=True;observed.update(observe(deepcopy(out)))
        out.update(observed.get('state_effects',{}))
    if observed.get('refresh_los_cache',False):
        out.update(los_check_time=f32(new_frame['time']),los_cache_valid=True)
    outputs.update(strength=f32(observed['strength']),extra_flags=observed['extra_flags'])
    if mode==2:
        visibility={k:out[k] for k in ('los_check_time','los_cache_valid','visibility_clear','visibility_weight')}
        out.update(saved);out.update(visibility);outputs['tracking']=True
        return result
    if observed['available']:
        out.update(reseed(p['shared'],q,observed['measurement']));result['reseeded']=True
        filtered=filter_update(p,out,q,observed['measurement'],observed['range'],observed['closure'],dt,limit_age,
            authored_rate=authored_rate,include_gap_after_reject=include_gap_after_reject,
            initial_angles=initial_angles,initial_rate=(0.,0.,0.))
        out.update(filtered['state']);outputs.update(filtered['outputs']);outputs['tracking']=True
        result['filter_accepted']=filtered['accepted'];out['gap']=0.
        if p['use_target_id']:out['associated_id']=observed['target_id']
    else:
        out['gap']=add(out['gap'],dt)
        if out['gap']>=p['prolongation'] and out['associated_id']!=-1:park()
    return result
