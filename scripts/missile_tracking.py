"""Describe native tracking outputs without equating coast with observation."""


def describe(family, manager, inertial_enabled, *, guidance=None, initialization=None):
    if initialization is not None:
        tracking = initialization['tracking']
        available = initialization.get('observation_available')
        reason = initialization.get('loss_reason')
    else:
        tracking = bool(manager['tracking'])
        measurements = (guidance.get('observations',[]) if family=='optical' else
                        (guidance.get('seeker_details') or {}).get('measurements',[]))
        available = any(m['accepted' if family=='optical' else 'available'] for m in measurements) if measurements else None
        reason = measurements[-1].get('loss_reason') if measurements else None
    if tracking:
        state = 'observed' if available else 'coasting'
    elif inertial_enabled and manager['inertial_valid'] and manager['mode'] in (0,4):
        state = 'inertial'
    else:
        state = 'acquiring'
    return dict(tracking=tracking, observation_available=available, tracking_state=state, loss_reason=reason)
