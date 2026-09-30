"""Independent game guidance launch initializers, pinned client 2.59.0.34.

Inputs are the constructed manager/controller/seeker histories and the supplied
release body packet. This is not prelaunch acquisition or body release/spread.
Optical point initialization requires an explicit visibility service when used.
Launcher notification requests are returned, not delivered to aircraft objects.
"""
from copy import deepcopy
import math
import struct
from kernels import f32,add,sub,mul,div
from launch_spread import mix,STEP,MASK
from optical_tracking import pre_slew_body
from seeker_designation import point as designate


def signed_sample(seed):
    value=struct.unpack('<f',struct.pack('<I',(mix(seed&MASK)>>9)|0x3f800000))[0]
    return add(add(value,value),-3.)


def properties(rocket,loaded_guidance):
    family=rocket['guidanceType'];config=rocket['guidance'];autopilot=config.get('guidanceAutopilot',{})
    p=dict(family=family,initial_mode=2 if autopilot.get('altitudeHoldEnabled',False) else
        1 if autopilot.get('loftEnabled',False) else 0,course=int(loaded_guidance['guidance']['course_gain']>0.),
        notify=bool(config.get('lockAfterLaunch',False)),semi_active=False)
    if family=='optical':
        p.update(notify=int(config.get('lockAfterLaunch',0))>0,shared=loaded_guidance['seeker']['shared'],
            point_error=f32(config['opticalSeeker'].get('pointError',0.)),
            designation_error=f32(config.get('designationError',0.)))
    elif family=='radar':p['semi_active']=loaded_guidance['manager']['semi_active']
    else:raise ValueError('Conventional optical or radar guidance required')
    return p


def notification_source(family,manager,source_flag):
    """Resolve the radar manager's inherited +0x170 source byte.

    Legacy reduced radar dictionaries did not represent that byte. Keep their
    supplied source service (and historical omitted-source zero), while rejecting
    incomplete new inertial blocks. Optical source delivery remains explicit.
    """
    if source_flag is not None and (not isinstance(source_flag,int) or not 0<=source_flag<=255):
        raise ValueError('Launch notification source must be a byte')
    if family=='radar':
        if 'inertial_source_tag' in manager:
            inherited=manager['inertial_source_tag']
            if not isinstance(inherited,int) or not 0<=inherited<=255:
                raise ValueError('Radar inertial_source_tag must be a byte')
            if source_flag is not None and source_flag!=inherited:
                raise ValueError('Radar notification source_flag disagrees with inherited inertial_source_tag')
            return inherited
        if 'inertial_velocity' in manager or 'inertial_padding' in manager:
            raise ValueError('Complete radar inertial state requires inertial_source_tag')
    return 0 if source_flag is None else source_flag


def initialize(p,state,body,seed,*,point_visibility=None,source_flag=None,notification_state=(0,0,0)):
    """Return initialized represented state, temporary seed and notification.

    point_visibility receives origin, direction and distance and returns clear,
    direction and distance. These are the native service's in/out values; no
    terrain ray is invented. The projectile's persistent launch seed is separate
    from this mutable temporary seed and is not advanced by its caller. Radar
    notification source is inherited from manager.inertial_source_tag when
    represented; an explicit source must agree. Only legacy reduced snapshots
    retain the historical zero default. LOBL retains notification_state verbatim.
    """
    if type(seed) is not int or not 0<=seed<=MASK:raise ValueError('32-bit unsigned launch seed required')
    source_flag=notification_source(p['family'],state['manager'],source_flag)
    if len(notification_state)!=3:raise ValueError('Three notification bytes required')
    s=deepcopy(state);seed=(seed+STEP)&MASK;manager=s['manager'];ray_requests=[]
    if p['family']=='radar':manager['seeker_tag']=mix(seed)>>24
    elif p['family']=='optical':
        tag=mix(seed)>>25;manager['seeker_flag']=tag
        phase=bytearray(struct.pack('<f',manager['scan_phase']));phase[0]=tag
        manager['scan_phase']=struct.unpack('<f',phase)[0]
        seeker=s['seeker'];raw=bytearray(seeker['opaque_state']);shared=seeker['state']
        if len(raw)!=32:raise ValueError('Optical launch requires all 32 opaque seeker bytes')
        if raw[0]==1:
            draws=[signed_sample(seed+i*STEP) for i in (3,2,1)];seed=(seed+3*STEP)&MASK
            point=[add(a,mul(p['point_error'],b)) for a,b in zip(struct.unpack('<3f',raw[4:16]),draws)]
            raw[4:16]=struct.pack('<3f',*point)
            delta=[sub(a,b) for a,b in zip(point,body['position'])]
            squared=[mul(x,x) for x in delta];distance=f32(math.sqrt(add(squared[2],add(squared[1],squared[0]))))
            if distance>1.:
                inverse=div(1.,distance);direction=[mul(x,inverse) for x in delta]
                query=dict(origin=list(map(f32,body['position'])),direction=direction,distance=distance)
                ray_requests.append(deepcopy(query))
                if point_visibility is None:raise ValueError('Optical point launch requires supplied visibility')
                visible=point_visibility(deepcopy(query));direction=list(map(f32,visible['direction']))
                if not visible['clear']:
                    point=[add(a,mul(visible['distance'],b)) for a,b in zip(body['position'],direction)]
                    raw[4:16]=struct.pack('<3f',*point)
                vx,vy,vz=map(f32,body['velocity']);x,y,z=direction
                # Native retains the inverse distance computed BEFORE the ray.
                rate=[mul(sub(mul(vy,z),mul(vz,y)),inverse),
                    mul(sub(mul(vz,x),mul(vx,z)),inverse),mul(sub(mul(vx,y),mul(vy,x)),inverse)]
                local=pre_slew_body(list(map(f32,body['quaternion'])),direction)
                pointed=designate(p['shared'],body['quaternion'],local,rate,shared['angles'])
                for key in ('angles','direction','angular_rate'):shared[key]=pointed[key]
        seeker['opaque_state']=list(raw)
        # These manager fields alias the same native seeker storage.
        manager.update(seeker_direction=shared['direction'][:],seeker_distance=shared['distance'],seeker_point_valid=bool(raw[0]))
        draws=[signed_sample(seed+i*STEP) for i in (3,2,1)];seed=(seed+3*STEP)&MASK
        if manager['inertial_valid']:
            manager['inertial_point']=[add(a,mul(p['designation_error'],b)) for a,b in zip(manager['inertial_point'],draws)]
    else:raise ValueError('Unsupported guidance family')
    # Each eight-float PID retains gains, decay and frequency; only these three
    # history fields are reset. Propulsion-controller state is untouched.
    for controller in ('orientation','guidance'):
        for pid in s[controller]['pid']:
            for index in (3,6,7):pid[index]=0.
    s['orientation']['enabled']=True
    s['guidance'].update(mode=p['initial_mode'],course=p['course'])
    notification=None;flags=list(notification_state)
    if p['notify']:
        notification=dict(source_flag=source_flag,semi_active=bool(p['semi_active']))
        flags=[1,source_flag,int(p['semi_active'])]
    return dict(state=s,temporary_seed=seed,notification=notification,notification_state=flags,ray_requests=ray_requests)
