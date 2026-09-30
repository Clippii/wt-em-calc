"""Explicit single-point radar ideal-signal policy, geometric_radar_v1.

Retains head dynamics, search/observation cones, native point geometry and
monostatic/bistatic measurement arithmetic. Requires supplied illumination for
semi-active flight. Original signal and association rejection are bypassed.
"""
from copy import deepcopy
import math
from kernels import f32,sub
from optical_geometry import observation_frame
from radar_scene_geometry import admitted
from radar_return_frame import response
from motor_vector import rotate_thrust
from radar_point_modes import update as modes_update


BOUND=f32(2147440000.)


def check_state(state):
    for key in ('range','range_rate','closure','closure_rate','gap'):
        if not math.isfinite(state[key]) or abs(state[key])>=BOUND/8:
            raise ValueError('Geometric radar estimate outside supported finite envelope: '+key)
    squared=sum(x*x for x in state['direction'])
    if not .99<squared<1.01:raise ValueError('Geometric radar requires a normalized direction history')


def configure(p):
    """Copy full flight properties; preserve supplied launch/filter history.

    Finite envelope: flight age below 1e6 s, 1e-5 <= dt <= 1 s, near-unit
    attitude/direction, coordinates and velocities below 1e7 in magnitude,
    scalar history below BOUND/8. Only flight seeker
    modes 5..8 are supported; general reset/prelaunch initialization is separate.
    """
    p=deepcopy(p);p['policy']='geometric_radar_v1';s=p['guidance']['seeker'];m=s['modes']
    m['shared']['gate_rate']=BOUND
    for name in ('distance','doppler'):
        m[name].update(minimum=0. if name=='distance' else -BOUND,maximum=BOUND,width=0.,limit_timeout=BOUND)
    m.update(range_search_half=BOUND,closure_search_half=BOUND)
    s['query']['maximum']=BOUND
    return p


def return_inputs(p,new,frame,illumination,body):
    # Native geometric point kernel with unit, flat angular response and zero
    # detection threshold. Hard angular admission is the original query cone.
    flat=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.]
    props=dict(p['observation']['frame'],tx_lobes=flat,rx_lobes=flat,tx_strength=1.,tx_scale=1.,rx_gain=1.,range_scaling=False)
    transmitter=illumination['frame'] if props['active'] else frame
    transmitter_velocity=illumination['velocity'] if props['active'] else new['velocity']
    signal=dict(target=body['position'],quaternion=body['quaternion'],velocity=body['velocity'],
        transmitter=transmitter,receiver=frame,transmitter_velocity=transmitter_velocity,receiver_velocity=new['velocity'],
        offset=[0.,0.,0.],extent=[0.,0.,0.],multipath=False,side_lobes=False,availability=1.,minimum_signal=0.,surface_height=0.)
    return props,signal


def measure(p,state,old,new,illumination,targets):
    if len(targets)>1:raise ValueError('Select one radar point target explicitly')
    op=p['observation'];frame=observation_frame(new['quaternion'],state['angles'],new['position'])
    if op['frame']['active'] and not illumination['present']:
        raise ValueError('Geometric radar policy requires an explicit illumination frame')
    effects=dict(target_id=-1,visibility_clear=True,visibility_weight=1.)
    out=dict(available=False,measurement=[1.,0.,0.],range=0.,closure=0.,strength=0.,extra_flags=0,
        target_id=-1,state_effects=effects,loss_reason='no_target',policy='geometric_radar_v1')
    if not targets:return out
    target=targets[0];body=target['scene']['new_target']
    if not admitted(body['position'],new['position'],frame[:3],p['query']['cosine'],0.,BOUND,0.):
        out['loss_reason']='observation_cone';return out
    result=response(*return_inputs(p,new,frame,illumination,body))
    if not result['accepted']:
        out['loss_reason']='sensor_geometry';return out
    values=result['output'];q=list(new['quaternion']);q[3]=-q[3]
    effects.update(target_value=values[11],los_cache_valid=True,
        los_check_time=state['los_check_time'] if state['los_cache_valid'] else f32(new['time']))
    out.update(available=True,measurement=rotate_thrust(q,values[3:6]),range=values[11],closure=values[19],
        strength=2.,extra_flags=1,target_id=target.get('id',-1),loss_reason=None)
    return out


def observation_provider(p,old,new,illumination,targets,signature_provider):
    measurements=[]
    def observation(state):
        result=measure(p,state,old,new,illumination,targets);measurements.append(result)
        return result
    return observation,measurements,[],[]


def update(p,state,old,new,illumination,targets,signature_provider,limit_age,*,mode,
           illumination_provider=None,**flags):
    if len(targets)>1:raise ValueError('Select one radar point target explicitly')
    if mode not in (5,6,7,8) or sub(new['time'],old['time'])<f32(1e-5):
        raise NotImplementedError('Geometric radar policy supports positive-step flight modes 5..8')
    if not sub(new['time'],old['time'])<=1.:raise ValueError('Geometric radar step exceeds supported interval')
    if not 0.<=limit_age<1e6:raise ValueError('Geometric radar flight age outside supported envelope')
    check_state(state)
    for frame in (old,new):
        if not .99<sum(x*x for x in frame['quaternion'])<1.01:raise ValueError('Near-unit body quaternion required')
        if any(not math.isfinite(x) or abs(x)>=1e7 for key in ('position','velocity') for x in frame[key]):
            raise ValueError('Body state outside geometric radar envelope')
    for target in targets:
        if any(not math.isfinite(x) or abs(x)>=1e7 for key in ('position','velocity') for x in target['scene']['new_target'][key]):
            raise ValueError('Target outside geometric radar envelope')
    if p['modes']['shared']['gate_rate']!=BOUND:raise ValueError('Configure the geometric radar policy first')
    if not illumination['present'] and p['observation']['frame']['active']:
        if illumination_provider is None:raise ValueError('Semi-active geometric flight requires supplied illumination')
        illumination=illumination_provider(old,new)
        if not illumination['present']:raise ValueError('Ideal illumination provider must be present')
    if any(not math.isfinite(x) or abs(x)>=1e7 for vector in (illumination['frame'],illumination['velocity']) for x in vector):
        raise ValueError('Illumination outside geometric radar envelope')
    result=modes_update(p,state,old,new,illumination,targets,signature_provider,limit_age,mode=mode,
        observation_factory=observation_provider,**flags)
    check_state(result['state'])
    return result
