"""Radar flight manager including aircraft-provider and inertial branches.

Recovered caller: .34 1452b0480; relocated .38 1452b83f0. The full seeker is
separate authoritative storage; only the seven existing manager aliases mirror
it. Aircraft callbacks and child helpers are explicit services. Supplying these
services establishes their inputs, not the correctness of their implementation.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,safe_div
from radar_manager import navigation
from radar_activation import world_to_body
from radar_handoff import manager_inertial
from optical_geometry import observation_frame
from motor_vector import rotate_thrust

ALIASES={'seeker_angles':'angles','seeker_direction':'direction','seeker_range':'range',
         'seeker_closure':'closure','range_valid':'range_valid','closure_valid':'closure_valid',
         'channel_index':'channel_index'}


def empty_designation():
    # This really is a zeroed 48-byte native block, including target ID zero.
    return dict(present=0,direction=[0.,0.,0.],angular_rate=[0.,0.,0.],
        range_valid=0,range=0.,closure_valid=0,closure=0.,target_id=0)


def designation_indicator(p,source_tag,records,channel_counts):
    """Native final output scan: independent of active, tracking and provider."""
    if channel_counts[p['datalink_counter']]>=p['datalink_limit']:return 0
    for record in records:
        source=int(record['source']) & 31
        eligible=(int(record['flag48']) & 255)==1 if p['datalink_reconnect'] & 1 else (
            (int(record['source_tag']) & 255)==source_tag)
        if p['designation_mask'] & (1<<source) and eligible and (int(record['point_valid']) & 255)!=0:
            return 8
    return 0


def update(p,state,seeker,old,new,elapsed,services,*,provider=None,records=(),
           channel_counts=(0,0),suppress=False,context_value=0.,observation_range=True):
    """Complete caller ordering with explicit native child-service contracts.

    provider is None or the same available/pose/velocity callback mapping used
    by radar_illumination. Services receive one argument dictionary and return
    supplied child outputs: illumination, source_selector, project, seeker,
    inertial_update, orientation, guidance and propulsion. Only actually invoked
    services are required. project/inertial_update dictionaries match the named
    flight_inertial arguments, apart from loaded properties held by the service.
    The manager's designation notification flag also enables datalink source
    lookup, just as its native +0x475 property does.
    """
    s=deepcopy(state);ss=deepcopy(seeker);events=[]
    old=list(map(f32,old));new=list(map(f32,new));elapsed=f32(elapsed);dt=sub(new[13],old[13])
    s.update(orientation_auxiliary=[0.,0.],count=0);tracking=False
    def call(name,args):
        events.append(dict(service=name,arguments=deepcopy(args)))
        return services[name](deepcopy(args))
    def aircraft(name):
        events.append(dict(service='provider_'+name,arguments={}))
        return provider[name]()
    def sync():
        for target,source in ALIASES.items():s[target]=deepcopy(ss[source])
    if s['active']:
        mode=s['mode']
        if (mode & 0xffffffff)<2:
            begin=elapsed>p['lock_timeout']
            if p['inertial_navigation']==1 and s['inertial_valid']:
                d=[sub(add(a,b),c) for a,b,c in zip(s['inertial_point'],s['inertial_offset'],new[:3])]
                distance=add(add(mul(d[2],d[2]),mul(d[1],d[1])),mul(d[0],d[0]))
                terms=[mul(a,b) for a,b in zip(d,ss['direction'])]
                projection=add(terms[2],add(terms[1],terms[0]))
                begin=begin and mul(abs(projection),projection)>mul(p['lock_cone_threshold'],distance) and distance<mul(p['lock_distance'],p['lock_distance'])
            if begin:s.update(mode=2,transition_time=new[13])
        elif mode==4 and sub(new[13],math.trunc(new[13]))<sub(old[13],math.trunc(old[13])):
            s.update(mode=2,transition_time=new[13])
        available=provider is not None and bool(aircraft('available'))
        mode=s['mode'];seeker_mode=(5,6,7,8,6)[mode] if 0<=mode<=4 else 0
        if p['semi_active']:
            illumination=call('illumination',dict(time=new[13],source_tag=s['inertial_source_tag'],prediction=False))
        else:
            illumination=dict(present=True,frame=observation_frame(new[3:7],ss['angles'],new[:3]),velocity=new[7:10])
        designation=empty_designation();initial_angles=None
        inertial_branch=available and p['inertial_navigation']==1 and s['inertial_valid']==1
        if inertial_branch and s['mode']!=3:
            source=None
            if p['designation_notification']==1:
                selected=call('source_selector',dict(source_tag=s['inertial_source_tag'],
                    channel_first=channel_counts[0],channel_last=channel_counts[1],
                    source_mask=p['designation_mask'],records=list(records)))
                if selected>=0 and records and (int(records[selected]['origin_override']) & 255)==1:
                    source=list(map(f32,records[selected]['origin']))
            if source is None:source=list(map(f32,aircraft('pose')['position']))
            velocity=list(map(f32,aircraft('velocity'))) if p['semi_active'] else new[7:10]
            projected=call('project',dict(state=manager_inertial(s),origin=new[:3],velocity=new[7:10],
                provider_origin=source if p['semi_active'] else new[:3],provider_velocity=velocity,
                rate_limit=f32(2147440000.)))
            # Projection's world angular-rate output aliases the later seeker's
            # output-angle storage. The body-frame designation has another slot.
            initial_angles=projected['angular_rate'][:2]
            designation=dict(present=projected['valid'],direction=world_to_body(new[3:7],projected['direction']),
                angular_rate=world_to_body(new[3:7],projected['angular_rate']),range_valid=1,
                range=projected['range'],closure_valid=1,closure=projected['closure'],target_id=ss['associated_id'])
        observed=call('seeker',dict(mode=seeker_mode,state=ss,designation=designation,
            old=old[:10]+[old[13]],new=new[:10]+[new[13]],illumination=illumination,
            elapsed=elapsed,tag=s['seeker_tag'],suppress=bool(suppress),transition_time=s['transition_time'],
            initial_angles=initial_angles,initial_extra_flags=0))
        ss=deepcopy(observed['state']);sync()
        s['count']=observed['count'];tracking=bool(observed['tracking'])
        rate=observed['angular_rate'][:];angles=observed['angles'][:]
        if inertial_branch:
            channel=p['distance_limits'][ss['channel_index']]
            skip=tracking and (ss['gap']>f32(.001) or
                (channel['present']==1 and ss['range_valid'] and mul(channel['minimum'],f32(1.1))>ss['range']))
            if not skip:
                position=list(map(f32,aircraft('pose')['position']))
                quaternion=list(map(f32,aircraft('pose')['quaternion']))
                velocity=list(map(f32,aircraft('velocity')))
                estimated=call('inertial_update',dict(state=manager_inertial(s),
                    view=dict(records=list(records),point=position,quaternion=quaternion,velocity=velocity),
                    origin=new[:3],quaternion=new[3:7],velocity=new[7:10],use_seeker=tracking,
                    direction=ss['direction'],angular_rate=ss['angular_rate'],range_valid=ss['range_valid'],
                    distance=ss['range'],radial_speed=ss['range_rate'],designation_valid=False,
                    designation_point=[0.,0.,0.],designation_velocity=[0.,0.,0.],
                    channel_first=channel_counts[0],channel_last=channel_counts[1],source_mask=p['designation_mask'],
                    time=new[13],dt=dt,angles=angles,rate=rate))
                for key,value in estimated['state'].items():s['inertial_'+key]=deepcopy(value)
                angles=estimated['angles'][:];rate=estimated['rate'][:]
        nav=[0.,0.,0.];tgo=0.;range_squared=0.;use_range=False
        if tracking:s['mode']=3;use_range=True
        elif s['mode']==0:use_range=True
        elif p['reacquire']!=1:s['mode']=3
        elif s['mode']==3:s.update(mode=2,transition_time=new[13])
        elif s['mode']==2:
            if min(p['search_limits'][:2])>=p['search_limits'][2]:s['mode']=4;use_range=True
        elif s['mode']==4:use_range=True
        if use_range:
            if not tracking and designation['present'] & 1 and designation['range_valid']:
                direction=rotate_thrust(new[3:7],designation['direction'])
                distance=designation['range'];closure=designation['closure']
                velocity_valid=p['use_target_velocity']==1
            else:
                direction=ss['direction'];distance=ss['range'];closure=ss['closure']
                velocity_valid=p['use_target_velocity']==1 and ss['range_valid'] and ss['closure_valid']
            nav,tgo,range_squared=navigation(direction,rate,new[7:10],distance,closure,velocity_valid)
        orientation=call('orientation',dict(time=elapsed,range_squared=range_squared,quaternion=new[3:7],omega=new[10:13],
            angles=angles,context_value=f32(context_value),dt=dt,fins=s['fins'],auxiliary=s['orientation_auxiliary']))
        s.update(fins=orientation['fins'],orientation_auxiliary=orientation['auxiliary'])
        if not orientation['override']:
            if tracking or s['mode'] in (0,4):
                feedback=[mul(sub(a,b),safe_div(1.,dt)) for a,b in zip(new[7:10],old[7:10])]
                guided=call('guidance',dict(time=elapsed,time_to_hit=tgo,range_squared=range_squared,position=new[:3],
                    quaternion=new[3:7],velocity=new[7:10],feedback=feedback,direction=ss['direction'],
                    angular_rate=rate,navigation=nav,dt=dt,fins=s['fins']))
                s['fins']=guided['fins'];terms=[mul(x,y) for x,y in zip(ss['direction'],nav)]
                prop=call('propulsion',dict(time=elapsed,position=new[:3],velocity=new[7:10],time_to_hit=tgo,
                    closure=add(terms[2],add(terms[1],terms[0])),dt=dt,factors=s['factors']))
                s['factors']=prop['factors']
            else:s['fins']=[0.,0.]
        s['strength']=observed['strength']
    else:s.update(strength=0.,fins=[0.,0.])
    sync()
    s.update(engaged=s['mode']>=2,tracking=tracking,direction=ss['direction'][:],
        distance=ss['target_value'] if tracking and not observation_range else ss['range'])
    channel=ss['channel_index'];s['search_value']=mul(add(p['search_values'][channel][1],p['search_values'][channel][0]),.5)
    if p['designation_notification']==1:
        s['designation_indicator']=designation_indicator(p,s['inertial_source_tag'],records,channel_counts)
    return dict(state=s,seeker=ss,events=events)
