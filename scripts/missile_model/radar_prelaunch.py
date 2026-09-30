"""Continuous radar launcher orchestration, native 1452ae110 (2.59.0.34).

Loaded properties and aircraft/service inputs are explicit. The seeker adapter
is a callback boundary: this component owns launch phases, inertial history,
prediction arguments, call ordering and published outputs, not observations.
"""
from copy import deepcopy
import math
from kernels import f32, add, sub, mul, safe_div
from launcher_designation import select
from launcher_inertial import update as inertial_update
from radar_activation import world_to_body
from radar_modes import update as modes_update
from radar_point_tracking import observation_provider
from optical_geometry import observation_frame
from radar_illumination import illumination as source_illumination


def body_frame(body):
    return dict(position=list(map(f32,body[:3])),quaternion=list(map(f32,body[3:7])),
                velocity=list(map(f32,body[7:10])),time=f32(body[13]))


def adapter_inputs(selected, old, new, position_offset, velocity_offset,
                   range_offset, speed_offset):
    """1452ada80 input arithmetic; illumination generation is a separate service."""
    def shifted(body):
        return dict(position=[add(a,b) for a,b in zip(position_offset,body[:3])],
                    quaternion=list(map(f32,body[3:7])),
                    velocity=[add(a,b) for a,b in zip(velocity_offset,body[7:10])],time=f32(body[13]))
    d=dict(present=selected['source']!=255,
           direction=world_to_body(new[3:7],selected['direction']),
           angular_rate=world_to_body(new[3:7],selected['angular_rate']),
           range_valid=selected['range_valid'],range=add(selected['range'],range_offset),
           closure_valid=bool(selected['closure_valid'] or selected['range_valid']),
           closure=add(selected['closure'] if selected['closure_valid'] else -selected['radial_velocity'],speed_offset),
           target_id=selected['target_id'])
    return dict(old_frame=shifted(old),new_frame=shifted(new),designation=d)


def seeker_update(p, request, state, observe, *, initial_angles=(0.,0.), initial_extra_flags=0):
    """Connect a prelaunch adapter request to the recovered full seeker modes.

    observe(temporary_state) supplies ordinary radar observations and their
    explicit cache/history effects. Its closure can use request's transformed
    frames and time_parameter, plus externally generated illumination/scene.
    The LOS expiry timeout is a loaded global, not that time_parameter.
    """
    result=modes_update(p['modes'],state,request['old_frame'],request['new_frame'],observe,request['limit_age'],
        mode=request['mode'],designation_present=request['designation']['present'],designation=request['designation'],
        suppress=request['suppress'],los_check_timeout=p['los_check_timeout'],
        authored_rate=p['authored_rate'],include_gap_after_reject=p['include_gap_after_reject'],
        initial_angles=initial_angles,initial_extra_flags=initial_extra_flags)
    return {key:result[key] for key in ('state','outputs')}


def active_illumination(request,state):
    """1452addb4..adec3: current head frame before prediction shifts or reseeding."""
    new=request['new']
    return dict(present=True,frame=observation_frame(new[3:7],state['angles'],new[:3]),
                velocity=list(map(f32,new[7:10])))


def point_seeker_update(p, request, state, illumination, targets, signature_provider,
                        *, initial_angles=(0.,0.)):
    """Compose ordinary point-scene observations with prelaunch seeker modes.

    Semi-active illumination, eligible target histories/signatures and visibility
    remain external inputs. Active head illumination is built independently from
    the unshifted current body frame. Ground/clutter contribution is excluded by the
    ordinary point observation component; no ideal tracking overrides are used.
    """
    if state['associated_id']!=-1 or (p['modes']['use_target_id'] and request['designation']['target_id']!=-1):
        raise NotImplementedError('Associated-unit observation lookup remains separate')
    if not p['semi_active']:illumination=active_illumination(request,state)
    elif illumination is None:raise ValueError('Explicit semi-active illumination state required')
    measure,measurements,selected,queries=observation_provider(p,request['old_frame'],request['new_frame'],
        illumination,targets,signature_provider,collect=False)
    result=seeker_update(p,request,state,measure,initial_angles=initial_angles)
    result.update(measurements=measurements,selected_ids=selected,queries=queries)
    return result


def provider_point_seeker_update(p, request, state, inertial_state, records, provider,
                                source_direction, targets, signature_provider, *,
                                datalink, reconnect_datalink, lag_limit=.2, initial_angles=(0.,0.)):
    """Build aircraft SARH illumination before ordinary point observation modes.

    Aircraft pose/velocity and resolved source sensors remain explicit services.
    The launcher inertial source tag is the value after the prelaunch inertial
    update. Mode 2 selects the prediction source policy; the body timestamp and
    aircraft frame are unshifted. Data-link properties must be provided rather
    than silently inferred from the presence of a designation record.

    The direction leaf uses recovered .38 arithmetic, separately validated with
    original code and saved .34 packets; universal cross-build identity is not
    established. The surrounding source/adapter ordering is saved .34 evidence.
    """
    if p['semi_active']:
        light=source_illumination(records,inertial_state['source_tag'],request['new'][13],
            provider,source_direction,prediction=request['mode']==2,inertial=p['inertial'],
            datalink=datalink,reconnect_datalink=reconnect_datalink,lag_limit=lag_limit)
    else:
        light=active_illumination(request,state)
    result=point_seeker_update(p,request,state,light,targets,signature_provider,initial_angles=initial_angles)
    result['illumination']=light
    return result


def prediction_offsets(p, old, designation, inertial_valid):
    """Native projected position/velocity, including inertial readiness cutoff."""
    x,y,z,w=map(f32,old[3:7])
    fx=add(add(add(mul(w,w),mul(x,x)),add(mul(w,w),mul(x,x))),-1.)
    fy=add(mul(x,y),mul(z,w));fy=add(fy,fy)
    fz=sub(mul(x,z),mul(w,y));fz=add(fz,fz)
    length=f32(math.sqrt(add(add(mul(fz,fz),mul(fy,fy)),mul(fx,fx))))
    factor=mul(safe_div(1.,length),p['speed_up'])
    velocity=[mul(a,factor) for a in (fx,fy,fz)]
    shift=[mul(add(mul(a,.5),v),p['lock_timeout']) for a,v in zip(velocity,old[7:10])]
    result=dict(position_offset=shift,velocity_offset=velocity,early_return=False)
    distance=designation['range'];direction=designation['direction']
    if p['inertial'] and inertial_valid:
        closure=add(-designation['radial_velocity'] if designation['range_valid'] else
                    designation['closure'] if designation['closure_valid'] else 0.,p['speed_up'])
        drift=0. if p['inertial_no_drift'] else mul(p['inertial_drift_speed'],f32(.18260720372200012))
        coefficient=p['prediction_coefficient']
        denominator=add(mul(closure,coefficient),drift)
        travel=mul(safe_div(mul(coefficient,distance),denominator),closure)
        cutoff=sub(distance,p['lock_distance'])
        result.update(travel=travel,cutoff=cutoff)
        if travel<cutoff:
            result['early_return']=True
            return result
        candidate_length=min(add(mul(closure,-3.),distance),travel)
        candidate=[mul(d,candidate_length) for d in direction]
        def dot(a,b):
            terms=[mul(x,y) for x,y in zip(a,b)]
            return add(add(terms[2],terms[1]),terms[0])
        if dot(candidate,direction)>dot(shift,direction):shift=candidate
        result['position_offset']=shift
    delta=[sub(mul(d,distance),v) for d,v in zip(direction,shift)]
    result['range_offset']=sub(f32(math.sqrt(add(add(mul(delta[2],delta[2]),mul(delta[1],delta[1])),mul(delta[0],delta[0])))),distance)
    result['speed_offset']=f32(p['speed_up'])
    return result


def update(p,state,old,new,records,seeker_step,*,available=True,prediction=False,
           suppress=False,associated_flag=None,lag_limit=.2,observation_range=True):
    """Advance with supplied 14-float body records and an explicit seeker service.

    seeker_step(request, seeker_state, inertial_state) returns state/outputs like
    radar_modes.update. Requests include native adapter and seeker arguments.
    associated_flag(id) resolves the optional native unit-signature eligibility
    flag; its absence is rejected if a designation carries a real unit ID.
    Notifications are emitted as data, not delivered to any external provider.
    """
    if len(old)!=14 or len(new)!=14 or not all(math.isfinite(x) for x in [*old,*new]):
        raise ValueError('Finite 14-float launcher records required')
    if state['seeker']['channel_index']!=0:
        raise NotImplementedError('Only the recovered single authored channel is supported')
    out=deepcopy(state);published=out['output']
    published['half_angle']=mul(add(p['half_angles'][1],p['half_angles'][0]),.5)
    result=dict(state=out,selection=None,calls=[],notifications=[],prediction=None)
    def unavailable():
        out['phase']=published['phase']=0
        return result
    if not available:return unavailable()
    selection=select(body_frame(new),records,p['primary_mask'],p['fallback_mask'],lag_limit=lag_limit)
    d=selection['output'];result['selection']=selection
    out['association_flag']=False
    if d['target_id']!=-1:
        if associated_flag is None:raise NotImplementedError('Associated-unit eligibility service required')
        out['association_flag']=bool(associated_flag(d['target_id']))
    if p['designation_required']:
        if d['source']==255:return unavailable()
        if p['permanent'] and out['phase']==0:
            out['phase']=0x30+0x20*int(p['lock_after_launch'])
    phase=out['phase']
    if phase>0 and not p['lock_after_launch'] and not d['flag55']:
        result['notifications'].append(dict(source_tag=d['source_tag'],semi_active=p['semi_active']))
    if p['inertial']:
        s=out['seeker']
        out['inertial']=inertial_update(out['inertial'],new[:3],new[7:10],s['direction'],s['angular_rate'],
            use_seeker=phase==0x40 and p['uncage'],range_valid=s['range_valid'],distance=s['range'],
            radial_velocity=s['range_rate'],designation_valid=d['source']!=255 and d['range_valid'],
            designation_point=d['point'],designation_velocity=d['velocity'],source_tag=d['source_tag'])
    age=sub(new[13],out['activation_time'])
    if phase==0x20 and age>p['warm_up']:
        phase=0x30+0x20*int(p['lock_after_launch']);out['activation_time']=f32(new[13])
    elif phase>=0x50 and age>p['work_time']:
        phase=0x30+0x20*int(p['lock_after_launch']) if p['permanent'] else 0
    out['phase']=phase
    def call(mode,position=(0.,0.,0.),velocity=(0.,0.,0.),range_offset=0.,speed_offset=0.):
        request=dict(mode=mode,old=list(old),new=list(new),selected=deepcopy(d),
            position_offset=list(position),velocity_offset=list(velocity),range_offset=range_offset,
            speed_offset=speed_offset,limit_age=out['activation_time'],time_parameter=out['activation_time'],
            suppress=bool(suppress),**adapter_inputs(d,old,new,position,velocity,range_offset,speed_offset))
        answer=seeker_step(deepcopy(request),deepcopy(out['seeker']),deepcopy(out['inertial']))
        out['seeker']=deepcopy(answer['state'])
        result['calls'].append(dict(request=request,answer=deepcopy(answer)))
        return answer['outputs']
    mode=3 if phase==0x30 else 3+int(p['uncage']) if phase==0x40 else 1 if phase==0x50 else 0
    if phase==0x50 and prediction:
        predicted=prediction_offsets(p,old,d,out['inertial']['valid']);result['prediction']=predicted
        if predicted['early_return']:
            out['strength']=0.;published['phase']=phase
            return result
        output=call(2,predicted['position_offset'],predicted['velocity_offset'],predicted['range_offset'],predicted['speed_offset'])
        out['strength']=f32(output['strength'])
    output=call(mode)
    if phase!=0x50:out['strength']=f32(output['strength'])
    if phase>=0x30 and phase!=0x50:
        phase=0x30+0x10*int(output['tracking']);out['phase']=phase
    s=out['seeker']
    published.update(direction=deepcopy(s['direction']),
        range=s['range'] if observation_range or phase!=0x40 else s['target_value'],
        strength=out['strength'],phase=phase,use_target_id=p['use_target_id'],associated_id=s['associated_id'])
    return result
