"""Ideal-signal, single-point optical policy using recovered game geometry.

This is an explicit simulation policy, not an unmodified game observation.
Retains moving-head nominal FOV, front hemisphere, 10 m blind distance and
native horizon. Ignores signatures, signal range, bands and countermeasures.
Head movement/filtering and continuation remain in the native-model seeker.
"""
from copy import deepcopy
from kernels import f32
from optical_geometry import observation_frame,point_response
from optical_tracking import pre_slew_body
from shared_seeker import normalize


def observe(p,state,quaternion,origin,targets,*,dt,new_time,collect=False,signal_threshold=None):
    if len(targets)>1:
        raise ValueError('Geometric policy requires one explicitly selected point target')
    out=deepcopy(state);out['reject_time']=0.
    frame=observation_frame(quaternion,state['angles'],origin)
    geometry=point_response(frame,targets[0]['position'],p['cosine']) if targets else None
    accepted=geometry is not None and geometry['admitted'] and geometry['core_lobe']
    reason=('no_target' if geometry is None else geometry['reason'] if not geometry['admitted'] else
            'field_of_view' if not geometry['core_lobe'] else None)
    measurement=[1.,0.,0.];auxiliary=[0.,0.,0.];reported=[]
    if accepted:
        measurement=normalize(pre_slew_body(list(map(f32,quaternion)),geometry['world_los']),'predict')
        auxiliary=list(map(f32,targets[0].get('auxiliary',[0.,0.,0.])))
        if not out['los_cache_valid']:out['los_check_time']=f32(new_time)
        out.update(distance=geometry['distance'],target_id=-1,los_cache_valid=True)
        if collect:reported=[dict(id=targets[0].get('id',-1),weight=1.)]
    elif f32(state['gap'])>p['prolongation']:
        out.update(distance=-1.,target_id=-1)
    return dict(accepted=bool(accepted),measurement=measurement,strength=2. if accepted else 0.,
        auxiliary=auxiliary,flag=False,state=out,reported_targets=reported,
        reported_target_count=len(reported),frame=frame,geometry=geometry,loss_reason=reason,
        policy='geometric_optical_v1')
