"""Independent optical observation, flight seeker, manager and controllers.

Client 2.59.0.34 primary no-provider path. Body states and motor clocks are
prescribed; point targets are already enumerated, with supplied signatures.
This is a native-behavior research composition, not a geometric-only seeker
policy or a trajectory simulator. Initialization remains supplied explicitly.
"""
from copy import deepcopy
import struct
from kernels import sub
from optical_modes import update as seeker_update
from optical_observation import observe
from optical_manager_controls import update as controls_update


def update(p,state,old,new,elapsed,clocks,targets,*,suppress=False,context_value=0.,height_query=None,
           observation_step=observe,matrix_velocity_frame=True):
    """Propagate all guidance histories without native child-result feedback.

    p adds seeker, observation, los_check_timeout to manager/controller props.
    state adds seeker={state,opaque_state} to manager/controller histories.
    The primary manager selects reset, coast, acquisition or tracking modes.
    Designation/provider and spiral-scan paths remain outside this composition.
    Global authored-rate and pre-observation prediction switches are enabled.
    """
    ss=deepcopy(state['seeker']);observations=[];child=None
    def seeker(args):
        nonlocal ss,child
        raw=ss['opaque_state'];q=args['new'][3:7];origin=args['new'][:3]
        old_time=args['old'][10];new_time=args['new'][10]
        def observation(temporary):
            os=dict(temporary,reject_time=struct.unpack('<f',bytes(raw[24:28]))[0],
                target_id=struct.unpack('<i',bytes(raw[28:32]))[0])
            measured=observation_step(p['observation'],os,q,origin,targets,
                dt=sub(new_time,old_time),new_time=new_time,collect=args['mode'] in (5,8))
            observations.append(measured)
            changed=raw[:]
            changed[24:28]=struct.pack('<f',measured['state']['reject_time'])
            changed[28:32]=struct.pack('<i',measured['state']['target_id'])
            if measured['accepted']:changed[0]=0
            return dict(available=measured['accepted'],measurement=measured['measurement'],
                distance=measured['state']['distance'],strength=measured['strength'],
                flag=measured['flag'],count=measured['reported_target_count'],
                auxiliary=measured['auxiliary'],opaque_state=changed,
                refresh_los_cache=not temporary['los_cache_valid'] and measured['state']['los_cache_valid'])
        result=seeker_update(p['seeker'],ss['state'],q,old_time,new_time,observation,
            mode=args['mode'],opaque_state=raw,authored_rate=True,predict_before_observation=True,
            suppress=args['suppress'],los_check_timeout=p['los_check_timeout'])
        ss={k:result[k] for k in ('state','opaque_state')}
        child=dict(result['outputs'],scan_phase=args['scan_phase'],
            direction=ss['state']['direction'][:],distance=ss['state']['distance'],
            point_valid=bool(ss['opaque_state'][0]))
        return deepcopy(child)
    result=controls_update(p,state,old,new,elapsed,clocks,seeker,
        suppress=suppress,context_value=context_value,height_query=height_query,matrix_velocity_frame=matrix_velocity_frame)
    result['state']['seeker']=ss
    result['seeker_output']=child
    result['observations']=observations
    return result
