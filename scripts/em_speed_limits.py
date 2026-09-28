import math
from air_state import cache,speed_of_sound
from wing_sweep import prepare,select


def speed_limits(fm,config):
    strength=select(prepare(fm),config['sweep_percent']/100.)['geometry']['strength']


    factor=cache([1.,0.,0.],config['altitude_m'])['ias_u']
    vne=strength['ias']/factor*3.6
    mne=strength['mach']*speed_of_sound(config['altitude_m'])*3.6
    if not all(math.isfinite(x) and x>0 for x in (vne,mne)):
        raise ValueError('Aircraft speed redlines must be positive and finite')
    limit=min(vne,mne)


    inside=max(0.,limit-.0036)
    return dict(speed_kmh=limit,sample_speed_kmh=inside,
        kind='VNE' if vne<=mne else 'Mach limit',vne_ias_kmh=strength['ias']*3.6,
        vne_tas_kmh=vne,mne_tas_kmh=mne,mne=strength['mach'],
        enforced=bool(config['structural_limits']),
        convention='Vertical total-speed IAS redline converted to TAS; lower of VNE and MNE; native pointwise checks retained')


def excluded_column(solver, speed):
    if not hasattr(solver,'plot_speed_limits'):
        solver.plot_speed_limits=speed_limits(solver.fm,solver.config)
    limit=solver.plot_speed_limits
    if not limit['enforced'] or speed<=limit['sample_speed_kmh']:return None
    return dict(speed_kmh=float(speed),points=[],boundary=None,sustained=[],load_checks=[],
        boundary_bracket_g=None,boundary_reason=limit['kind'],boundary_status='speed limit',
        lower_boundary=None,numerical_gap_brackets=[],interior_failures=[],unresolved_load_intervals=[],
        elapsed_s=0.,mass=solver.mass,engine=solver.engine.summary)


def excluded_point(solver, speed, load):
    column=excluded_column(solver,speed)
    if column is None:return None
    return dict(speed_kmh=float(speed),load_g=float(load),valid=False,converged=False,
        reasons=[column['boundary_reason']],not_evaluated=True,evaluations=0,
        turn_dps=math.degrees(9.8100004196167*math.sqrt(max(0.,load*load-1.))/(speed/3.6)),
        ps_mps=None,ps_continuous_mps=None,component_forces={},
        instructor_enabled=solver.config['instructor'])
