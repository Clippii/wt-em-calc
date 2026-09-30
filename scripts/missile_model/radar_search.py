"""Independent radar search candidates from supplied enumerated point targets.

Client 2.59.0.34, mode-7 block 143fef4ca..143fefb3d. Includes bistatic geometry,
history gates and separate range/Doppler occupancy maps. Scene enumeration,
target providers and radar observations remain explicit inputs.
"""
import math
from copy import deepcopy
from kernels import f32,add,sub,mul,div,safe_div
from radar_modes import properties as mode_properties
from radar_candidate_order import order_candidates


def antenna_axis(config):
    half=max(mul(config['angleHalfSens'],f32(math.pi/360.)),f32(1e-4))
    sine=f32(math.sin(half));exponent=div(f32(math.log(2.)),mul(sine,sine))
    side=f32(2.**mul(config['sideLobesSensitivity'],.3321928083896637))
    complement=1. if side>f32(.9999) else sub(1.,min(div(mul(f32(math.log(max(side,mul(side,side)))),-.5),exponent),1.))
    cutoff=f32(math.asin(f32(math.sqrt(max(0.,sub(1.,complement))))))
    return dict(half=half,cutoff=cutoff,exponent=exponent,side=side)


def properties(config):
    p=mode_properties(config)
    signature=config.get('targetSignatureType','radar')
    if signature not in ('radar','radarIntercept'):raise NotImplementedError('Search source-frame defaults outside the radar catalog remain separate')
    half=f32(math.pi/360.)
    antenna=config['receiver']['antenna']
    az=antenna_axis(antenna.get('azimuth',antenna));el=antenna_axis(antenna.get('elevation',antenna))
    search_cos=f32(math.cos(mul(config.get('searchFov',360.),half)))
    p.update(use_source_frame=bool(config.get('active',signature=='radar')),
        receiver_axes=[az,el],beam_cosine=max(f32(math.cos(max(az['cutoff'],el['cutoff']))),mul(abs(search_cos),search_cos)),
        scan_half_sector=mul(config.get('scanHalfSector',0.),f32(math.pi/180.)),
        range_search_half=mul(config.get('distGate',{}).get('distGateSearchRange',1000.),.5),
        closure_search_half=mul(config.get('dopplerSpeedGate',{}).get('dopplerSpeedGateSearchRange',50.),.5))
    if p['scan_half_sector']>min(az['half'],el['half']):raise NotImplementedError('Angular scan candidate generation remains separate')
    return p


def norm_squared(v):
    return add(add(mul(v[2],v[2]),mul(v[1],v[1])),mul(v[0],v[0]))


def candidates(p,state,frame,beam,illuminator,targets):
    """Return unsorted [range, closure, score] rows in enumeration order.

    frame/illuminator supply position and velocity; illuminator also supplies
    its world forward axis. Targets supply position and velocity. Actual scene
    bounds/filtering and target-ID association selection must happen upstream.
    """
    position=list(map(f32,frame['position']));velocity=list(map(f32,frame['velocity']))
    ipos=list(map(f32,illuminator['position']));ivel=list(map(f32,illuminator['velocity']))
    iaxis=list(map(f32,illuminator['direction']));beam=list(map(f32,beam))
    baseline=[sub(a,b) for a,b in zip(position,ipos)]
    baseline_length=f32(math.sqrt(norm_squared(baseline)))
    baseline_terms=[mul(sub(a,b),d) for a,b,d in zip(velocity,ivel,baseline)]
    baseline_rate=mul(add(add(baseline_terms[1],baseline_terms[2]),baseline_terms[0]),safe_div(1.,baseline_length))
    occupied=[set(),set()];result=[]
    for target in targets:
        delta=[sub(a,b) for a,b in zip(target['position'],position)]
        squared=norm_squared(delta)
        dot=add(mul(delta[2],beam[2]),add(mul(beam[1],delta[1]),mul(delta[0],beam[0])))
        if mul(abs(dot),dot)<mul(mul(p['beam_cosine'],p['beam_cosine']),squared):continue
        other=[sub(a,b) for a,b in zip(target['position'],ipos)]
        other_squared=norm_squared(other)
        distance=0.;closure=0.;score=0.
        if p['distance']['present']:
            distance=mul(add(sub(f32(math.sqrt(other_squared)),baseline_length),f32(math.sqrt(squared))),.5)
            if state['range_valid']:
                residual=abs(sub(distance,state['range']))
                if residual>p['range_search_half']:continue
                ratio=div(residual,p['range_search_half']);score=mul(ratio,ratio)
        if p['doppler']['present']:
            other_length=f32(math.sqrt(other_squared));length=f32(math.sqrt(squared))
            u=[mul(a,div(1.,other_length)) for a in other] if other_length>1e-9 else iaxis
            v=[mul(a,div(1.,length)) for a in delta] if length>1e-9 else beam
            tv=list(map(f32,target['velocity']))
            tx=[mul(sub(a,b),d) for a,b,d in zip(ivel,tv,u)]
            rx=[mul(sub(a,b),d) for a,b,d in zip(velocity,tv,v)]
            first=add(add(tx[0],tx[1]),add(tx[2],baseline_rate))
            second=add(rx[0],add(rx[1],rx[2]))
            closure=mul(add(second,first),.5)
            if state['closure_valid']:
                residual=abs(sub(closure,state['closure']))
                if residual>p['closure_search_half']:continue
                ratio=div(residual,p['closure_search_half']);score=add(score,mul(ratio,ratio))
        seen=[]
        for i,(axis,value) in enumerate(((p['distance'],distance),(p['doppler'],closure))):
            if not axis['present']:seen.append(True);continue
            inverse=div(2.,axis['width']) if abs(mul(axis['width'],.5))>f32(4e-19) else 0.
            slot=math.floor(add(mul(sub(value,axis['minimum']),inverse),.5))
            if 0<=slot<=1023:
                seen.append(slot in occupied[i]);occupied[i].add(slot)
            else:seen.append(False)
        if all(seen):continue
        result.append([distance,closure,score])
    return result


def update(p,state,old_frame,new_frame,targets,source,observe,limit_age,*,suppress=False,
           authored_rate=True,include_gap_after_reject=True,initial_extra_flags=0):
    """Mode 7 with supplied scene enumeration and observation provider.

    `source` is the frame supplied by the caller (already resolved upstream for
    the selected seeker/illumination mode). Associated-ID scene lookup and
    angular scanning remain outside this component. Failed providers can have
    explicit state effects, just as in radar_tracking.
    """
    from shared_seeker import slew
    from optical_tracking import pre_slew_body,reseed
    from optical_geometry import boresight
    from motor_vector import rotate_thrust
    from radar_tracking_filters import rounded_state,update as measurement_update,coast
    dt=sub(new_frame['time'],old_frame['time'])
    if not math.isfinite(dt) or dt<f32(1e-5):raise NotImplementedError('Radar short-step reset is separate')
    if not suppress and state['associated_id']!=-1:raise NotImplementedError('Associated-ID scene resolution must be supplied separately')
    out=rounded_state(state);q=list(map(f32,new_frame['quaternion']))
    observed=dict(available=False,strength=0.,extra_flags=initial_extra_flags)
    attempts=[];ordered=[]
    if not suppress:
        out['angles']=slew(p['shared'],pre_slew_body(q,out['direction']),out['angles'],dt,False,False)
        saved_angles=list(out['angles']);saved_range=out['range'];saved_closure=out['closure']
        beam=boresight(q,out['angles'])
        illumination=source if p['use_source_frame'] else dict(position=new_frame['position'],velocity=new_frame['velocity'],direction=beam)
        ordered=order_candidates(candidates(p,out,new_frame,beam,illumination,targets))
        for distance,closure,score in ordered:
            out['los_cache_valid']=False
            if p['distance']['present']:out['range']=distance
            if p['doppler']['present']:out['closure']=closure
            attempts.append(deepcopy(out))
            observed=dict(available=False,measurement=[0.,0.,0.],range=0.,closure=0.,strength=0.,extra_flags=0)
            observed.update(observe(deepcopy(out)))
            out.update(observed.get('state_effects',{}))
            if observed.get('refresh_los_cache',False):out.update(los_check_time=f32(new_frame['time']),los_cache_valid=True)
            if observed['available']:
                if p['distance']['present']:out['range_valid']=True
                if p['doppler']['present']:out['closure_valid']=True
                if p['use_target_id']:out['associated_id']=observed.get('target_id',-1)
                out['direction']=beam
                break
        else:out.update(angles=saved_angles,range=saved_range,closure=saved_closure)
    d=out['direction']
    dp=[mul(sub(a,b),u) for a,b,u in zip(new_frame['position'],old_frame['position'],d)]
    projected_position=add(dp[2],add(dp[1],dp[0]));out['range']=sub(out['range'],projected_position)
    dv=[mul(sub(a,b),u) for a,b,u in zip(new_frame['velocity'],old_frame['velocity'],d)]
    projected_velocity=add(add(dv[2],dv[1]),dv[0]);out['closure']=add(out['closure'],projected_velocity)
    filtered=None
    if observed['available']:
        rate=rotate_thrust(q,out['angular_rate'])
        out.update(reseed(p['shared'],q,observed['measurement']))
        out['angular_rate']=[min(10.,max(-10.,a)) for a in rate]
        out['channel_index']=0
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
            out['closure_rate']=div(sub(add(out['closure'],out['range_rate']),add(projected_velocity,previous_speed)),dt)
            terms=[mul(v,u) for v,u in zip(new_frame['velocity'],d)]
            out['closure']=add(sub(terms[0],out['range_rate']),add(terms[2],terms[1]))
        if not p['distance']['present']:
            out['range']=sub(add(projected_position,out['range']),mul(dt,out['closure']))
            terms=[mul(v,u) for v,u in zip(new_frame['velocity'],d)]
            out['range_rate']=add(sub(terms[0],out['closure']),add(terms[2],terms[1]))
        out['gap']=0.;outputs=filtered['outputs']
    else:
        coasted=coast(p,out,q,dt,authored_rate=authored_rate);out=coasted['state'];outputs=coasted['outputs']
        out['gap']=add(out['gap'],dt)
        if out['gap']>p['prolongation']:out['associated_id']=-1
    outputs.update(tracking=accepted,strength=f32(observed['strength']),extra_flags=observed['extra_flags'])
    return dict(state=out,outputs=outputs,attempts=attempts,ordered=ordered,
        filter_accepted=None if filtered is None else filtered['accepted'])
