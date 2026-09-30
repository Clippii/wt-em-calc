"""Independent radar designation helper 143fea360, client 2.59.0.34.

Designation vectors are already in missile body coordinates. Range/closure
validity, values and target ID are supplied; aircraft designation generation
and source selection are outside this helper.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,safe_div
from motor_vector import rotate_thrust
from seeker_designation import point
from radar_tracking_filters import properties as filter_properties,clamp_channel,rounded_state


def properties(config):
    p=filter_properties(config)
    p['use_target_id']=bool(config.get('useTargetId',False))
    p['doppler_to_distance']=f32(config.get('dopplerSpeedToDistMult',0.))
    for key,name in (('distance','distance'),('doppler','dopplerSpeed')):
        c=config.get(name,{});axis=p[key]
        axis['unambiguous']=f32(c.get('maxUnambiguousValue',add(axis['maximum'],axis['maximum']))) if axis['present'] else 1.
        axis['signal_width_min']=f32(c.get('signalWidthMin',mul(axis['width'],.1))) if axis['present'] else 0.
    return p


def apply(p,state,frame,designation,dt):
    out=rounded_state(state);q=list(map(f32,frame['quaternion']))
    supplied_direction=list(map(f32,designation['direction']))
    shared=point(p['shared'],q,supplied_direction,designation['angular_rate'],out['angles'],gate_previous=False)
    for key in ('angles','direction','angular_rate'):out[key]=shared[key]
    out['channel_index']=0;accepted=shared['accepted'];inverse=safe_div(1.,dt)
    def scalar(axis,measurement,key,rate_key,valid_key):
        nonlocal accepted
        previous=out[key];out[valid_key]=True
        if axis['present']:
            if axis['unambiguous']==0.:raise NotImplementedError('Zero radar ambiguity period is outside the finite supported domain')
            wrapped=f32(math.fmod(measurement,axis['unambiguous']))
            value=clamp_channel(wrapped,axis)
            accepted=accepted and abs(sub(wrapped,value))<mul(.5,axis['signal_width_min'])
        else:value=measurement
        out[key]=value;out[rate_key]=mul(sub(value,previous),inverse)
    if designation['range_valid']:
        value=f32(designation['range'])
        if p['distance']['present']:value=add(mul(p['doppler_to_distance'],designation['closure']),value)
        scalar(p['distance'],value,'range','range_rate','range_valid')
        direction=rotate_thrust(q,supplied_direction)
        terms=[mul(u,v) for u,v in zip(direction,frame['velocity'])]
        out['range_rate']=add(add(terms[2],out['range_rate']),add(terms[1],terms[0]))
    if designation['closure_valid']:
        scalar(p['doppler'],f32(designation['closure']),'closure','closure_rate','closure_valid')
    if p['use_target_id'] and designation['target_id']!=-1:out['associated_id']=designation['target_id']
    out['target_id']=-1
    return dict(state=out,accepted=bool(accepted))


def reseed(p, state, frame, designation):
    """143fea230: activation pointing and scalar initialization.

    Unlike the ongoing designation update, absent channels and invalid supplied
    scalars retain their entire previous state. Written scalar rates become
    zero; gap, rejection, target history and visibility cache are preserved.
    The caller clears the cache separately after activation. The designation's
    presence byte is not consulted by this native helper.
    """
    out = rounded_state(state)
    shared = point(p['shared'], frame['quaternion'], designation['direction'],
                   designation['angular_rate'], out['angles'], gate_previous=True)
    for key in ('angles', 'direction', 'angular_rate'):
        out[key] = shared[key]
    out['channel_index'] = 0
    for name, key, rate_key, valid_key in (
        ('distance', 'range', 'range_rate', 'range_valid'),
        ('doppler', 'closure', 'closure_rate', 'closure_valid')):
        axis = p[name]
        if not designation[valid_key] or not axis['present']:
            continue
        value = f32(designation[key])
        if name == 'distance':
            value = add(mul(p['doppler_to_distance'], designation['closure']), value)
        if axis['unambiguous'] == 0.:
            raise NotImplementedError('Zero radar ambiguity period is outside the finite supported domain')
        out[valid_key] = True
        out[key] = clamp_channel(f32(math.fmod(value, axis['unambiguous'])), axis)
        out[rate_key] = 0.
    if p['use_target_id'] and designation['target_id'] != -1:
        out['associated_id'] = designation['target_id']
    return out
