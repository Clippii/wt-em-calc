"""Independent ordinary clear-scene point radar observation composition.

Client 2.59.0.34, primary ordinary-aggregation path of 143fea740. Candidate
enumeration/qualification, illumination, target history/signature and visibility
services are explicit inputs. Alternative search selection and finite targets
remain outside this composition; no ideal-tracking policy is implied.
"""
from kernels import f32,add,mul
from optical_geometry import observation_frame
from radar_point_return import evaluate
from radar_aggregation import aggregate
from radar_measurement import finalize


def observe(p,state,old,new,illumination,targets,signature_provider,*,collect=False):
    """Targets are ordered scene-query results with point histories and services.

    Each target has unit, id, unit_type and a scene containing old_target,
    new_target, target_present, availability, water, surface_height, visibility.
    signature_provider(target,args) returns values[14] and subtarget target_id.
    Visibility is a supplied {valid,clear,weight} service result. Its use is
    deferred until the native-equivalent point-return stage reaches it.
    """
    state=dict(state)
    if p['frame']['active'] and not illumination['present']:
        state.update(association_id=-1,last_range=-1.)
        return dict(accepted=False,direction=[1.,0.,0.],range=add(p['early_range_base'],mul(p['early_range_width'],.5)),
            closure=add(p['early_range_base'],mul(p['early_closure_width'],.5)),strength=0.,target_id=-1,reported=[],state=state)
    receiver_old=dict(frame=observation_frame(old['quaternion'],state['angles'],old['position']),velocity=old['velocity'])
    receiver_new=dict(frame=observation_frame(new['quaternion'],state['angles'],new['position']),velocity=new['velocity'])
    transmitter=dict(frame=illumination['frame'],velocity=illumination['velocity']) if p['frame']['active'] else receiver_new
    weight=dict(p['weight'],receiver_axes=receiver_new['frame'][:9],receiver_velocity=new['velocity'],
        distance_control=[state['range_estimate'],0.,0.],doppler_control=[state['closure_estimate'],0.,0.],angular_window=[0.,0.])
    masks=[(int(mask)|int(p['band_override']))&0xffffffff for mask in p['base_masks']]
    initial_cache=dict(valid=state['cache_valid'],clear=state['cache_clear'],weight=state['cache_weight'])
    shared=dict(initial_cache);returns=[]
    for target in targets:
        local=dict(valid=initial_cache['valid'],clear=shared['clear'],weight=shared['weight'])
        def visibility():
            if not local['valid']:local.update(target['scene']['visibility'])
            return local
        scene=dict(target['scene'],old_transmitter=transmitter,new_transmitter=transmitter,
            old_receiver=receiver_old,new_receiver=receiver_new,t0=old['time'],t1=new['time'],masks=masks,
            side_lobes=True,multipath=p['multipath'],sensitivity=p['sensitivity'],frequency_index=p['frequency_index'],
            cross_sections=p['cross_sections'],visibility=visibility)
        result=evaluate(p['frame'],weight,scene,lambda args:signature_provider(target,args))
        returns.append(dict(result,unit=target['unit'],unit_type=target['unit_type'],cache=dict(local)))
        if local['valid'] and not initial_cache['valid']:
            if not shared['valid']:shared=dict(valid=True,clear=True,weight=1.)
            shared['clear']=shared['clear'] and local['clear'];shared['weight']=min(f32(shared['weight']),f32(local['weight']))
    aggregated=aggregate(dict(receiver_axes=receiver_new['frame'][:9],detection_scale=.5,minimum_signal=p['weight']['minimum_signal'],
        range_width=p['weight']['gates']['distance']['width'],doppler_width=p['weight']['gates']['doppler']['width']),returns,cache=initial_cache)
    final=dict(p['measurement'],time=new['time'],quaternion=new['quaternion'],collect=collect,clear_association=True,detection_scale=.5)
    result=finalize(final,state,aggregated,{t['unit']:t['id'] for t in targets})
    result['state'].update(cache_clear=aggregated['cache']['clear'],cache_weight=aggregated['cache']['weight'])
    return result
