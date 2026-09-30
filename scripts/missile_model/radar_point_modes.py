"""Independent ordinary radar point observations with acquisition and tracking.

Single authored channel, no angular scan or associated-unit lookup. Body,
illumination and eligible point-target histories remain caller inputs.
"""
from kernels import f32,add,sub
from optical_geometry import boresight
from optical_tracking import pre_slew_body
from shared_seeker import slew
from radar_scene_geometry import admitted
from radar_point_tracking import observation_provider
from radar_modes import update as modes_update
from radar_search import update as search_update


def update(p,state,old,new,illumination,targets,signature_provider,limit_age,*,mode,
           designation_present=False,designation=None,
           suppress=False,authored_rate=True,include_gap_after_reject=True,los_check_timeout=-1.,
           initial_angles=(0.,0.),initial_extra_flags=0,
           observation_factory=observation_provider):
    mode &= 0xffffffff
    dt=sub(new['time'],old['time'])
    # Reset, park and designation/coast-only branches perform no unit lookup.
    # Preserve their inherited IDs; reject unsupported lookup paths separately.
    if state['associated_id']!=-1 and not (mode==0 or dt<f32(1e-5) or mode in (1,5,6)):
        raise NotImplementedError('Associated-unit lookup remains separate')
    measure,measurements,selected,queries=observation_factory(p,old,new,illumination,targets,signature_provider)
    flags=dict(suppress=suppress,authored_rate=authored_rate,include_gap_after_reject=include_gap_after_reject)
    if mode==7 and dt>=f32(1e-5):
        scene=[]
        if not suppress:
            angles=slew(p['modes']['shared'],pre_slew_body(new['quaternion'],state['direction']),state['angles'],dt,False,False)
            axis=boresight(new['quaternion'],angles)
            minimum=0.;maximum=p['query']['maximum']
            if state['range_valid'] and p['modes']['distance']['present']:
                half=p['modes']['range_search_half']
                minimum=max(sub(add(state['range'],10.),half),0.)
                maximum=min(add(add(state['range'],-10.),half),maximum)
            scene=[t for t in targets if t.get('eligible',True) and admitted(t['scene']['new_target']['position'],
                new['position'],axis,p['modes']['beam_cosine'],minimum,maximum,t.get('radius',0.))]
            selected.append([t['id'] for t in scene])
            queries.append(dict(origin=list(new['position']),axis=axis,cone_cosine=p['modes']['beam_cosine'],
                minimum_range=minimum,range=maximum))
        source=dict(position=illumination['frame'][9:12],velocity=illumination['velocity'],direction=illumination['frame'][:3])
        # Native mode 7 does not inspect the designation. Its normal-duration
        # filter/coast path writes angles, but may preserve incoming extra flags.
        result=search_update(p['modes'],state,old,new,[t['scene']['new_target'] for t in scene],source,measure,limit_age,
            initial_extra_flags=initial_extra_flags,**flags)
    else:
        result=modes_update(p['modes'],state,old,new,measure,limit_age,mode=mode,
            designation_present=designation_present,designation=designation,los_check_timeout=los_check_timeout,
            initial_angles=initial_angles,initial_extra_flags=initial_extra_flags,**flags)
    result.update(measurements=measurements,selected_ids=selected,queries=queries)
    return result
