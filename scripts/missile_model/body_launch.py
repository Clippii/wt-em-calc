"""Independent dry-world release-record construction and rocket launch body.

Pinned 2.59.0.34: supplied world record, no attachment transform, zero surface
height/offset and immersion. Guidance callbacks consume the preprocessed record
before angular reset and launch attitude/velocity processing. Their histories
are handled separately by guidance_launch, not guessed from the body pose.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import struct
from kernels import f32,add,sub,mul,div,motor_properties
from launch_spread import mix,STEP,MASK,attitude,velocity


def load_table():
    rows=json.loads(Path(__file__).with_name('launch-spread-table.json').read_text())['rows']
    raw=b''.join(struct.pack('<2f',*row) for row in rows)
    if hashlib.sha256(raw).hexdigest()!='b50a7d8183c1166f5fcf8e815dc317280fb96dad0fb519664d0f5317c8f9b580':
        raise ValueError('Launch spread table differs from pinned executable')
    return tuple(tuple(row) for row in rows)


TABLE=load_table()


def position_seed(position):
    words=[]
    for value in position:
        scaled=mul(value,100.)
        words.append((math.trunc(scaled)&MASK) if -2**31<=scaled<2**31 else 0x80000000)
    seed=((words[0]^words[1]^words[2])+STEP)&MASK
    fraction=sub(struct.unpack('<f',struct.pack('<I',(mix(seed)>>9)|0x3f800000))[0],1.)
    return seed,fraction


def construct(record):
    """World-space record after aircraft/attachment transformations, dry air."""
    body={key:list(map(f32,record[key])) for key in ('position','quaternion','velocity','omega')}
    for key,size in (('position',3),('quaternion',4),('velocity',3),('omega',3)):
        if len(body[key])!=size or not all(math.isfinite(x) for x in body[key]):raise ValueError('Invalid release '+key)
    time=f32(record['time'])
    if not math.isfinite(time):raise ValueError('Finite release time required')
    seed,fraction=position_seed(body['position'])
    body.update(time=time,seed=seed,body_random=fraction,clocks=[0.]*4,water=False,
        distance=0.,water_distance=0.,surface_depth=sub(0.,body['position'][1]),immersion=0.)
    return body


def initialize(rocket,body,launch_seed,global_seed,*,launch_time,clock_time,launch_mode=2,
               spread_scale=1.,distance_limit=0.,simple_lifetime=True,lifetime_multiplier=1.):
    """Rocket wrapper's represented body and launch metadata, after construction.

    launch_seed is the persistent projectile launch seed. body.seed is separately
    position-derived and drives later motor deviation. global_seed is consumed
    by spread only when launch_seed is zero. None is silently substituted for
    another. Owner-provided spread scaling is an explicit input.
    """
    if any(type(v) is not int or not 0<=v<=MASK for v in (launch_seed,global_seed)):
        raise ValueError('Unsigned 32-bit launch and global seeds required')
    b=deepcopy(body);manager_input=deepcopy(body)
    b.update(omega=[0.,0.,0.],time=f32(launch_time))
    if launch_mode==2:
        props=motor_properties(rocket)
        processed=attitude(b['quaternion'],f32(rocket['maxDeltaAngle']),
            mul(rocket.get('advancedSpread',0.),spread_scale),launch_seed,global_seed,TABLE)
        launched=velocity(b['velocity'],processed['quaternion'],first_delay=props[0].delay if props else 0.,
            propulsion_count=len(props),start_speed=f32(rocket['startSpeed']),use_start_speed=bool(rocket['useStartSpeed']),
            max_angle=f32(rocket['maxDeltaAngle']),scale=mul(rocket.get('velSpread',1.),spread_scale),
            seed=launch_seed,global_seed=processed['global_seed'],table=TABLE)
        b.update(quaternion=processed['quaternion'],velocity=launched['velocity']);global_seed=launched['global_seed']
    speed=f32(math.sqrt(add(add(mul(b['velocity'][0],b['velocity'][0]),mul(b['velocity'][1],b['velocity'][1])),mul(b['velocity'][2],b['velocity'][2]))))
    if rocket.get('distanceFuse',True) and f32(distance_limit)>0:
        lifetime=div(distance_limit,add(speed,.001))
    else:
        life=f32(rocket.get('timeLife',-1.))
        lifetime=mul(life,1. if simple_lifetime else lifetime_multiplier) if life>0 else 300.
    return dict(body=b,manager_input=manager_input,launch_seed=launch_seed,global_seed=global_seed,
        launch_time=f32(launch_time),clock_time=f32(clock_time),range_origin=body['position'][:],lifetime=lifetime,
        proximity_clock_origin=[0.,*body['position']] if rocket.get('hasProximityFuse',False) else [0.]*4)
