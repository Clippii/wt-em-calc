"""Independent point radar seeker, manager and all controllers.

An explicit provider_context connects the recovered aircraft/inertial branch.
Omitting it retains the original no-provider saved-evidence interface.
"""
from copy import deepcopy
from radar_point_modes import update as seeker_update
from radar_manager_controls import update as controls_update


def update(p,state,old,new,elapsed,clocks,targets,signature_provider,*,suppress=False,context_value=0.,height_query=None,
           seeker_step=seeker_update,provider_context=None,matrix_velocity_frame=True):
    if provider_context is not None:
        from radar_provider_controls import update as provider_update
        return provider_update(p,state,old,new,elapsed,clocks,targets,signature_provider,provider_context,
            suppress=suppress,context_value=context_value,height_query=height_query,seeker_step=seeker_step,
            matrix_velocity_frame=matrix_velocity_frame)
    ss=deepcopy(state['seeker']);child=None;details=None
    def seeker(args):
        nonlocal ss,child,details
        def body(packet):
            return dict(position=packet[:3],quaternion=packet[3:7],velocity=packet[7:10],time=packet[10])
        illumination=args['illumination']
        if not illumination['present']:
            illumination=dict(present=False,frame=[1.,0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.],velocity=[0.,0.,0.])
        details=seeker_step(p['seeker'],ss,body(args['old']),body(args['new']),illumination,targets,signature_provider,
            args['elapsed'],mode=args['mode'],suppress=args['suppress'],los_check_timeout=p['los_check_timeout'])
        ss=details['state'];out=details['outputs']
        child=dict(tracking=out['tracking'],angles=out['angles'],angular_rate=out['angular_rate'],
            strength=out['strength'],count=out['extra_flags'],
            state=dict(seeker_angles=ss['angles'],seeker_direction=ss['direction'],seeker_range=ss['range'],
                seeker_closure=ss['closure'],range_valid=ss['range_valid'],closure_valid=ss['closure_valid'],channel_index=ss['channel_index']))
        return deepcopy(child)
    result=controls_update(p,state,old,new,elapsed,clocks,seeker,suppress=suppress,context_value=context_value,height_query=height_query,
        matrix_velocity_frame=matrix_velocity_frame)
    result['state']['seeker']=ss
    result.update(seeker_output=child,seeker_details=details)
    return result
