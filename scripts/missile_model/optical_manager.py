"""Recovered optical flight-manager orchestration, client 2.59.0.34.

Primary path of 143ebb860 with no aircraft/provider object and no designation
notification. Seeker and three controller services are explicit callbacks;
this module does not yet integrate their independently recovered histories.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,safe_div


def leading_angles(angles,rate,q,gain):
    """Native 143ebc42e..598 expression order, including nonunit q behavior."""
    x,y,z,w=map(f32,q);rx,ry,rz=map(f32,rate)
    xz=mul(x,z);yw=mul(y,w);tworx=add(rx,rx)
    first=mul(add(yw,xz),tworx)
    yz2=mul(add(y,y),z);minus2wx=mul(mul(-2.,w),x)
    first=add(mul(add(minus2wx,yz2),ry),first)
    zz2=add(mul(z,z),mul(z,z));zw=mul(z,w)
    wwminus1=add(add(mul(w,w),mul(w,w)),-1.)
    first=add(mul(add(zz2,wwminus1),rz),first)
    second=mul(sub(zw,mul(x,y)),tworx)
    second=sub(second,mul(add(add(mul(y,y),mul(y,y)),wwminus1),ry))
    second=add(mul(sub(minus2wx,yz2),rz),second)
    return [add(mul(second,gain),angles[0]),sub(angles[1],mul(first,gain))]


def update(p,state,old,new,elapsed,services,*,suppress=False,context_value=0.):
    """Run manager decisions around supplied seeker/controller results.

All frames are native 14-float body states: position, xyzw quaternion, velocity,
angular velocity, timestamp. Callback arguments and results are plain dicts.
State includes mutable manager outputs plus the seeker fields copied to them.
No source/provider object or designation notification is silently synthesized.
"""
    if p.get('designation_notification',False):raise NotImplementedError('Designation notification remains separate')
    s=deepcopy(state);events=[];elapsed=f32(elapsed)
    old=list(map(f32,old));new=list(map(f32,new));dt=sub(new[13],old[13]);tracking=False
    def call(name,args):
        events.append(dict(service=name,arguments=deepcopy(args)))
        return services[name](deepcopy(args))
    if not s['active']:
        s.update(strength=0.,fins=[0.,0.],count=0,auxiliary_flag=False)
    else:
        mode=s['mode']
        # Native unsigned comparison: negative enum values do not enter 0/1.
        if (mode&0xffffffff)<2:
            begin=elapsed>p['lock_timeout']
            if p['inertial_navigation'] and s['inertial_valid']:
                d=[sub(add(a,b),c) for a,b,c in zip(s['inertial_offset'],s['inertial_point'],new[:3])]
                distance=add(add(mul(d[2],d[2]),mul(d[1],d[1])),mul(d[0],d[0]))
                terms=[mul(a,b) for a,b in zip(d,s['seeker_direction'])]
                projection=add(terms[2],add(terms[1],terms[0]))
                begin=begin and mul(abs(projection),projection)>mul(p['lock_cone_threshold'],distance) and distance<mul(p['lock_distance'],p['lock_distance'])
            if begin:s.update(mode=3,transition_time=new[13])
        elif mode==5 and sub(new[13],math.trunc(new[13]))<sub(old[13],math.trunc(old[13])):
            s.update(mode=3,transition_time=new[13])
        mode=s['mode'];seeker_mode=(6,6,6,7,8,6)[mode] if 0<=mode<=5 else 0
        observation=call('seeker',dict(mode=seeker_mode,old=old[:10]+[old[13]],new=new[:10]+[new[13]],
            flag=s['seeker_flag'],suppress=bool(suppress),transition_time=s['transition_time'],scan_phase=s['scan_phase']))
        s.update(scan_phase=observation['scan_phase'],seeker_direction=observation['direction'],
            seeker_distance=observation['distance'],seeker_point_valid=observation['point_valid'])
        tracking=observation['tracking'];rate=observation['angular_rate'] if tracking else [0.,0.,0.]
        angles=list(observation['angles'])
        if p['leading_gain']>0. and elapsed<add(p['leading_times'][0],p['leading_times'][1]):
            angles=leading_angles(angles,rate,new[3:7],p['leading_gain'])
        orientation=call('orientation',dict(time=elapsed,range_squared=0.,quaternion=new[3:7],omega=new[10:13],
            angles=angles,context_value=f32(context_value),dt=dt,fins=s['fins'],auxiliary=s['orientation_auxiliary']))
        s.update(fins=orientation['fins'],orientation_auxiliary=orientation['auxiliary'])
        if not orientation['override']:
            if tracking or p['inertial_navigation']:
                feedback=[mul(sub(a,b),safe_div(1.,dt)) for a,b in zip(new[7:10],old[7:10])]
                guidance=call('guidance',dict(time=elapsed,time_to_hit=0.,range_squared=0.,position=new[:3],
                    quaternion=new[3:7],velocity=new[7:10],feedback=feedback,direction=s['seeker_direction'],
                    angular_rate=rate,navigation=new[7:10],dt=dt,fins=s['fins']))
                s['fins']=guidance['fins']
                propulsion=call('propulsion',dict(time=elapsed,position=new[:3],velocity=new[7:10],
                    time_to_hit=0.,closure=0.,dt=dt,factors=s['factors']))
                s['factors']=propulsion['factors']
            else:s['fins']=[0.,0.]
        s.update(strength=observation['strength'],count=observation['count'],
            auxiliary_flag=p['auxiliary_flag'],auxiliary=observation['auxiliary'])
    if tracking or (s['mode']!=0 and not p['reacquire']):s['mode']=4
    elif s['mode']==4:s.update(mode=3,transition_time=new[13])
    elif s['mode']==3 and not p['scan_half_sector']>p['half_fov']:s['mode']=5
    s.update(engaged=s['mode']>=3,tracking=tracking,direction=s['seeker_direction'][:],
        distance=s['seeker_distance'],point_valid=s['seeker_point_valid'])
    return dict(state=s,events=events)
