"""Radar observation finalization after ordinary aggregation, client 2.59.0.34.

143feb78d and 143fec4be paths of 143fea740. Scene collection, special search
selection and early missing-illumination handling are separate.
"""
from kernels import f32,add,mul,safe_div
from motor_vector import rotate_thrust


def finalize(p,state,aggregate,unit_ids):
    """Produce measurement and updated association/cache-time state.

    unit_ids maps opaque aggregate unit handles to supplied unit service IDs.
    clear_association identifies the 143feb78d path; the 143fec4be path preserves
    the association ID. Cache generation remains aggregate's responsibility.
    """
    state=dict(state);v=aggregate['values'];strength=0.
    scale=add(p['angular_width_multiplier'],p['angular_width_multiplier'])
    if aggregate['accepted'] and mul(p['angular_widths'][0],scale)>v[3] and mul(p['angular_widths'][1],scale)>v[4]:
        denominator=add(add(v[11],p['minimum_signal']),v[10])
        strength=safe_div(mul(p['detection_scale'],v[9]),denominator)
    if not state['cache_valid'] and aggregate['cache']['valid']:state['cache_time']=f32(p['time'])
    state['cache_valid']=aggregate['cache']['valid']
    accepted=strength>1.
    reported=[]
    if accepted:
        if not aggregate['ranked']:raise ValueError('Accepted measurement requires a ranked unit')
        q=list(map(f32,p['quaternion']));q[3]=-q[3]
        direction=rotate_thrust(q,v[:3]);distance=v[5];closure=v[7]
        state['last_range']=distance
        if p['clear_association']:state['association_id']=-1
        target_id=unit_ids[aggregate['ranked'][0]['unit']]
        if p['collect']:
            reported=[dict(id=unit_ids[x['unit']],weight=x['weight']) for x in aggregate['ranked'][:8]]
    else:
        direction=[1.,0.,0.];distance=f32(p['default_range']);closure=f32(p['default_closure']);target_id=-1
        if f32(state['coast_time'])>f32(p['coast_limit']):
            state['last_range']=-1.
            if p['clear_association']:state['association_id']=-1
    return dict(accepted=accepted,direction=direction,range=distance,closure=closure,strength=strength,
        target_id=target_id,reported=reported,state=state)
