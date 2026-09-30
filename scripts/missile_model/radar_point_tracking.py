"""Independent ordinary point observation connected to radar mode-8 history.

Prescribed body states, illumination, qualified scene units and point target
services. Native signal/visibility loss remains; ideal tracking is separate.
"""
from optical_geometry import boresight
from radar_scene_geometry import admitted
from radar_observation import observe
from radar_tracking import update as tracking_update


def observation_state(state):
    return dict(angles=state['angles'],coast_time=state['gap'],association_id=state['target_id'],
        last_range=state['target_value'],cache_time=state['los_check_time'],cache_valid=state['los_cache_valid'],
        cache_clear=state['visibility_clear'],cache_weight=state['visibility_weight'],
        range_estimate=state['range'],closure_estimate=state['closure'])


def observation_provider(p,old,new,illumination,targets,signature_provider,*,collect=True):
    """Build the ordinary point-observation service and its diagnostic records."""
    measurements=[];selected=[];queries=[]
    def measure(temporary):
        candidates=[]
        if not (p['observation']['frame']['active'] and not illumination['present']):
            axis=boresight(new['quaternion'],temporary['angles']);query=p['query']
            candidates=[t for t in targets if t.get('eligible',True) and admitted(t['scene']['new_target']['position'],
                new['position'],axis,query['cosine'],0.,query['maximum'],t.get('radius',0.))]
            selected.append([t['id'] for t in candidates])
            queries.append(dict(origin=list(new['position']),axis=axis,cone_cosine=query['cosine'],minimum_range=0.,range=query['maximum']))
        measured=observe(p['observation'],observation_state(temporary),old,new,illumination,candidates,signature_provider,collect=collect)
        measurements.append(measured);s=measured['state']
        effects=dict(target_id=s['association_id'],target_value=s['last_range'],los_check_time=s['cache_time'],
            los_cache_valid=s['cache_valid'],visibility_clear=s['cache_clear'],visibility_weight=s['cache_weight'])
        return dict(available=measured['accepted'],measurement=measured['direction'],range=measured['range'],
            closure=measured['closure'],strength=measured['strength'],extra_flags=len(measured['reported']),
            target_id=measured['target_id'],state_effects=effects)
    return measure,measurements,selected,queries


def update(p,state,old,new,illumination,targets,signature_provider,limit_age,*,suppress=False,
           authored_rate=True,include_gap_after_reject=True,los_check_timeout=-1.):
    if state['associated_id']!=-1:raise NotImplementedError('Associated-unit lookup is outside this point composition')
    measure,measurements,selected,queries=observation_provider(p,old,new,illumination,targets,signature_provider)
    result=tracking_update(p['tracking'],state,old,new,measure,limit_age,suppress=suppress,authored_rate=authored_rate,
        include_gap_after_reject=include_gap_after_reject,los_check_timeout=los_check_timeout)
    result.update(measurements=measurements,selected_ids=selected,queries=queries)
    return result
