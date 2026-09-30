"""Primary no-aircraft-provider radar manager, native 1452b0480.

Client 2.59.0.34. Seeker/controllers are services; no provider/inertial-estimator
or designation-list branch is silently synthesized. Primary observation global
147983b9e=1. Semi-active illumination is absent without a provider.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,safe_div
from optical_geometry import observation_frame


SEEKER_FIELDS=('seeker_angles','seeker_direction','seeker_range','seeker_closure',
               'range_valid','closure_valid','channel_index')


def navigation(direction,rate,velocity,distance,closure,use_target_velocity):
    if use_target_velocity:
        relative=[mul(x,distance) for x in direction]
        tangent=[sub(mul(rate[1],relative[2]),mul(rate[2],relative[1])),
                 sub(mul(rate[2],relative[0]),mul(rate[0],relative[2])),
                 sub(mul(rate[0],relative[1]),mul(rate[1],relative[0]))]
        velocity=[add(mul(x,closure),y) for x,y in zip(direction,tangent)]
    return list(velocity),safe_div(distance,closure),mul(distance,distance)


def update(p,state,old,new,elapsed,services,*,suppress=False,context_value=0.):
    """14-float body packets; absent source/provider and empty designation list."""
    s=deepcopy(state);events=[];old=list(map(f32,old));new=list(map(f32,new))
    elapsed=f32(elapsed);dt=sub(new[13],old[13]);tracking=False
    s.update(orientation_auxiliary=[0.,0.],count=0)
    def call(name,args):
        events.append(dict(service=name,arguments=deepcopy(args)))
        return services[name](deepcopy(args))
    if s['active']:
        mode=s['mode']
        if (mode&0xffffffff)<2:
            begin=elapsed>p['lock_timeout']
            if p['inertial_navigation'] and s['inertial_valid']:
                d=[sub(add(a,b),c) for a,b,c in zip(s['inertial_point'],s['inertial_offset'],new[:3])]
                distance=add(add(mul(d[2],d[2]),mul(d[1],d[1])),mul(d[0],d[0]))
                terms=[mul(a,b) for a,b in zip(d,s['seeker_direction'])]
                projection=add(terms[2],add(terms[1],terms[0]))
                begin=begin and mul(abs(projection),projection)>mul(p['lock_cone_threshold'],distance) and distance<mul(p['lock_distance'],p['lock_distance'])
            if begin:s.update(mode=2,transition_time=new[13])
        elif mode==4 and sub(new[13],math.trunc(new[13]))<sub(old[13],math.trunc(old[13])):
            s.update(mode=2,transition_time=new[13])
        mode=s['mode'];seeker_mode=(5,6,7,8,6)[mode] if 0<=mode<=4 else 0
        illumination=dict(present=False) if p['semi_active'] else dict(present=True,
            frame=observation_frame(new[3:7],s['seeker_angles'],new[:3]),velocity=new[7:10])
        observed=call('seeker',dict(mode=seeker_mode,old=old[:10]+[old[13]],new=new[:10]+[new[13]],
            illumination=illumination,elapsed=elapsed,tag=s['seeker_tag'],suppress=bool(suppress),
            transition_time=s['transition_time']))
        s.update({key:deepcopy(observed['state'][key]) for key in SEEKER_FIELDS})
        s['count']=observed['count'];tracking=observed['tracking'];rate=observed['angular_rate']
        nav=[0.,0.,0.];tgo=0.;range_squared=0.;use_range=False
        if tracking:s['mode']=3;use_range=True
        elif s['mode']==0:use_range=True
        elif not p['reacquire']:s['mode']=3
        elif s['mode']==3:s.update(mode=2,transition_time=new[13])
        elif s['mode']==2:
            if min(p['search_limits'][:2])>=p['search_limits'][2]:s['mode']=4;use_range=True
        elif s['mode']==4:use_range=True
        if use_range:
            nav,tgo,range_squared=navigation(s['seeker_direction'],rate,new[7:10],s['seeker_range'],s['seeker_closure'],
                p['use_target_velocity'] and s['range_valid'] and s['closure_valid'])
        orientation=call('orientation',dict(time=elapsed,range_squared=range_squared,quaternion=new[3:7],omega=new[10:13],
            angles=observed['angles'],context_value=f32(context_value),dt=dt,fins=s['fins'],auxiliary=s['orientation_auxiliary']))
        s.update(fins=orientation['fins'],orientation_auxiliary=orientation['auxiliary'])
        if not orientation['override']:
            if tracking or s['mode'] in (0,4):
                feedback=[mul(sub(a,b),safe_div(1.,dt)) for a,b in zip(new[7:10],old[7:10])]
                guidance=call('guidance',dict(time=elapsed,time_to_hit=tgo,range_squared=range_squared,position=new[:3],
                    quaternion=new[3:7],velocity=new[7:10],feedback=feedback,direction=s['seeker_direction'],
                    angular_rate=rate,navigation=nav,dt=dt,fins=s['fins']))
                s['fins']=guidance['fins']
                terms=[mul(x,y) for x,y in zip(s['seeker_direction'],nav)]
                prop=call('propulsion',dict(time=elapsed,position=new[:3],velocity=new[7:10],time_to_hit=tgo,
                    closure=add(terms[2],add(terms[1],terms[0])),dt=dt,factors=s['factors']))
                s['factors']=prop['factors']
            else:s['fins']=[0.,0.]
        s['strength']=observed['strength']
    else:s.update(strength=0.,fins=[0.,0.])
    s.update(engaged=s['mode']>=2,tracking=tracking,direction=s['seeker_direction'][:],distance=s['seeker_range'])
    channel=s['channel_index']
    s['search_value']=mul(add(p['search_values'][channel][1],p['search_values'][channel][0]),.5)
    if p['designation_notification']:s['designation_indicator']=0
    return dict(state=s,events=events)
