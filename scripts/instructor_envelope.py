def profile(fm,config=None):
    return dict(experimental=True, kind='steady AoA schedule',
        input='native critical-angle targets and established adjusted wing AoA',
        correction='native settled wing-angle adjustment, rate feedback and reduced forward moment balance; full aircraft trim enforces control authority',
        branch='aircraft equilibrium below positive stall; same AoA constraint at boundary and interior',
        trim=(('Trim-independent endpoints evaluated directly; otherwise simultaneous one-g equilibrium with original recovery; fixed trim bounds pilot authority')),
        authority_mode=(config or {}).get('instructor_authority_mode','native'),
        power='selected engine power; native thrust force, moment and propwash retained',
        history='static schedule; transient overload reserve/release omitted',
        physical_limits='positive-AoA stall, strength, control authority and configured speed limits retained',
        capability_status='Algebraic steady effective AoA schedule (approximation). Native rate feedback, elevator compression and physical limits retained; transient overshoot, delay and control history omitted.',
        validated_live=False)


def controller_limits(solver,value,speed=None):
    from instructor_steady_aoa import controller_limits as schedule_limits
    return schedule_limits(solver,value,speed)
