"""Independent launch handoff for supplied world record and guidance histories.

The flight composition retains body timestamps and a separate launch epoch. Aircraft attachment,
prelaunch acquisition and launcher notifications remain external inputs/services.
"""
from copy import deepcopy
import math
from kernels import f32
from body_launch import construct,initialize as body_initialize
from guidance_launch import properties,initialize as guidance_initialize


def initialize(p,record,constructed_guidance,controls,launch_seed,global_seed,*,
               active_flag=False,point_visibility=None,source_flag=None,
               notification_state=(0,0,0),spread_scale=1.,distance_limit=0.,
               simple_lifetime=True,lifetime_multiplier=1.,launch_time=None,clock_time=None):
    """Initialize at an explicit launch epoch; return state and launch metadata.

    Guidance/controller history and the eight existing projectile controls are
    required explicitly. They cannot be inferred from missile pose alone. The
    record is already in world space, after any aircraft/attachment transform.
    Returned lifetime is metadata, not an implemented termination decision.
    launch_time defaults to record.time; clock_time defaults to launch_time.
    Both can be supplied separately, matching the native wrapper's inputs.
    Radar notification source comes from the constructed manager's inherited
    inertial_source_tag; an explicitly supplied source must agree with that byte.
    Legacy reduced snapshots and optical guidance retain their source service.
    """
    controls=list(map(f32,controls))
    if len(controls)!=8 or not all(math.isfinite(x) for x in controls):
        raise ValueError('Eight finite initial projectile controls required')
    launch_time=f32(record['time'] if launch_time is None else launch_time)
    clock_time=f32(launch_time if clock_time is None else clock_time)
    if not math.isfinite(launch_time) or not math.isfinite(clock_time):
        raise ValueError('Finite launch and world clock times required')
    rocket=p['rocket'];body=construct(record)
    guidance=deepcopy(constructed_guidance)
    guidance['manager']['active']=bool(active_flag) if rocket.get('operated',False) else True
    initialized=guidance_initialize(properties(rocket,p['guidance']),guidance,body,launch_seed,
        point_visibility=point_visibility,source_flag=source_flag,notification_state=notification_state)
    launched=body_initialize(rocket,body,launch_seed,global_seed,launch_time=launch_time,clock_time=clock_time,
        spread_scale=spread_scale,distance_limit=distance_limit,simple_lifetime=simple_lifetime,
        lifetime_multiplier=lifetime_multiplier)
    state=dict(body=launched['body'],guidance=initialized['state'],controls=controls)
    # Missing epoch means zero in the v1 snapshot format. Preserve existing
    # zero-epoch exports exactly; nonzero epochs are explicit restart state.
    if launch_time!=0.:state['launch_time']=launch_time
    return dict(state=state,
        body_launch={k:v for k,v in launched.items() if k not in ('body','manager_input')},
        guidance_launch={k:v for k,v in initialized.items() if k!='state'})
