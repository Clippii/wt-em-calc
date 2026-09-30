"""Compose ordinary point radar, aircraft support, inertial history and controls.

Loaded properties, provider callbacks, designation records and point-target
services are explicit inputs. No aircraft data or seeker lock is inferred from
the body alone. The complete seeker and inertial histories remain in state.
"""
from copy import deepcopy

from kernels import f32,sub
from flight_inertial import project, update as inertial_update, select_record
from optical_manager_controls import update as controls_update
from radar_illumination import illumination as build_illumination, absent
from radar_point_modes import update as seeker_update
from radar_provider_manager import update as manager_update


def update(p,state,old,new,elapsed,clocks,targets,signature_provider,context,*,
           suppress=False,context_value=0.,height_query=None,seeker_step=seeker_update,environment=None,matrix_velocity_frame=True):
    """One manager update with the recovered child helpers connected.

    p adds the explicit loaded ``inertial`` properties to the existing control
    properties; ``manager`` must contain the provider manager's expanded fields.
    context supplies ``provider`` (or None), ``records``, ``channel_counts``,
    ``lag_limit`` and ``source_direction``. The latter is only used when SARH
    illumination actually resolves a record. Optional ``noise_tables`` supplies
    initialized tables; otherwise the recovered fixed-seed startup tables apply.
    ``observation_range`` selects the native output global, defaulting to True.
    ``environment`` optionally supplies the recovered atmosphere inputs to
    acceleration and propulsion control; it does not change a body model here.
    For a reset step whose native stack angles have no preceding projection,
    ``caller_seeker_angles`` must explicitly supply those two caller values.

    Callbacks are per-update external services. Checkpointed mutable missile
    history is entirely within the returned state and can be round-tripped as
    JSON. Associated-unit lookup and angular scan retain their existing limits.
    """
    ss=deepcopy(state['seeker']);child=None;details=None
    provider=context['provider'];records=context['records']
    channels=context['channel_counts'];lag=context['lag_limit']
    inertial=p['inertial']

    def seeker(args):
        nonlocal ss,child,details
        def body(packet):
            return dict(position=packet[:3],quaternion=packet[3:7],velocity=packet[7:10],time=packet[10])
        illumination=args['illumination'] if args['illumination']['present'] else absent()
        designation=args['designation']
        angles=args['initial_angles']
        if angles is None:
            if args['mode']==0 or sub(args['new'][10],args['old'][10])<f32(1e-5):
                # Native reset leaves caller output angles untouched. They are
                # not persistent seeker angles and may not be inferred as zero.
                if 'caller_seeker_angles' not in context:
                    raise ValueError('This reset branch requires explicit caller_seeker_angles; '
                                     'the native manager does not initialize that output storage')
                angles=context['caller_seeker_angles']
            else:
                # Every supported normal-duration unprojected flight mode
                # overwrites this output through search, tracking or coast.
                angles=(0.,0.)
        details=seeker_step(p['seeker'],args['state'],body(args['old']),body(args['new']),
            illumination,targets,signature_provider,args['elapsed'],mode=args['mode'],
            designation_present=bool(designation['present']),designation=designation,
            suppress=args['suppress'],los_check_timeout=p['los_check_timeout'],
            initial_angles=angles,initial_extra_flags=args['initial_extra_flags'])
        ss=deepcopy(details['state']);out=details['outputs']
        child=dict(state=ss,tracking=out['tracking'],angles=out['angles'],
            angular_rate=out['angular_rate'],strength=out['strength'],count=out['extra_flags'])
        return deepcopy(child)

    def source_selector(args):
        # The standalone native selector has no enable check; its parent has
        # already checked datalink == 1 before invoking this service.
        index=select_record(dict(inertial,datalink=1),dict(source_tag=args['source_tag']),args['records'],
            args['channel_first'],args['channel_last'],args['source_mask'])
        return -1 if index is None else index

    def illumination(args):
        return build_illumination(records,args['source_tag'],args['time'],provider,
            context['source_direction'],prediction=args['prediction'],
            inertial=p['manager']['inertial_navigation'],datalink=inertial['datalink'],
            reconnect_datalink=inertial['reconnect'],lag_limit=lag)

    def estimate(args):
        return inertial_update(inertial,**args,lag_limit=lag,noise_tables=context.get('noise_tables'))

    def manager(properties,history,old_body,new_body,age,services,**kwargs):
        nonlocal ss
        services=dict(services,illumination=illumination,source_selector=source_selector,
            project=lambda args:project(**args),inertial_update=estimate)
        result=manager_update(properties,history,ss,old_body,new_body,age,services,
            provider=provider,records=records,channel_counts=channels,
            observation_range=context.get('observation_range',True),**kwargs)
        ss=result['seeker']
        return result

    result=controls_update(p,state,old,new,elapsed,clocks,seeker,suppress=suppress,
        context_value=context_value,height_query=height_query,manager_step=manager,environment=environment,
        matrix_velocity_frame=matrix_velocity_frame)
    result['state']['seeker']=ss
    result.update(seeker_output=child,seeker_details=details)
    return result
