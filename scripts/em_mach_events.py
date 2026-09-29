"""Native Mach-polar events and consistent float32-Mach correction proposals.

Nothing here replaces aerodynamic coefficients or relaxes trim acceptance.
"""
import os
import numpy as np

ENABLED = os.environ.get('WT_EM_MACH_AWARE', '1') != '0'
# Heuristics select extra sampling/projection work, never relaxed acceptance.
EVENT_RELATIVE_JUMP = 1e-4
CANCELLATION_RATIO = 16384.


def model_catalog(model, flaps):
    from polar_runtime import flap_polar, mach_value
    from control_mixer import curve
    values=curve(model['flaps'],flaps,4) if model['flaps'] else [flaps]*4
    runtimes=[('WingPlane',flap_polar(model['polars']['WingPlane'],values[0]))]
    runtimes.extend((name,model['polars'][name][0][1])
                   for name in ('HorStabPlane','VerStabPlane','FuselagePlane'))
    events={};rough=[]
    for component,runtime in runtimes:
        if runtime['mode']!=3:continue
        for index,row in enumerate(runtime['mach']):
            if index==6 and runtime['base'][8]==0.:continue
            a,b,high,slope,limit,*coefficients=row
            if not 0.<a<b:continue
            cancellation=sum(abs(c)*b**i for i,c in enumerate(coefficients))
            if cancellation>CANCELLATION_RATIO*max(1.,abs(high)):
                rough.append((float(a),float(b)))
            for mach,side in ((a,'start'),(b,'end')):
                before=float(np.nextafter(np.float32(mach),np.float32(-np.inf)))
                after=float(np.nextafter(np.float32(mach),np.float32(np.inf)))
                left=mach_value(runtime,before if side=='start' else mach,index)
                right=mach_value(runtime,mach if side=='start' else after,index)
                if abs(right-left)<=EVENT_RELATIVE_JUMP*max(1.,abs(left),abs(right)):continue
                event=events.setdefault((float(mach),side),dict(mach=float(mach),side=side,components=[]))
                event['components'].append(dict(component=component,multiplier=index,
                    left_value=float(left),right_value=float(right)))
    return dict(events=[events[k] for k in sorted(events)],rough_intervals=sorted(set(rough)))


def catalog(solver):
    if not ENABLED or not hasattr(solver,'model'):return dict(events=[],rough_intervals=[])
    key=(id(solver.model),float(solver.flaps))
    cached=getattr(solver,'_mach_catalog',None)
    if cached is None or cached[0]!=key:
        cached=(key,model_catalog(solver.model,solver.flaps));solver._mach_catalog=cached
    return cached[1]


def aircraft_catalog(fm,config):
    if not ENABLED:return dict(events=[],rough_intervals=[])
    from aircraft_model import prepare,at_sweep
    from component_assembly import f32
    model=at_sweep(prepare(fm),config['sweep_percent']/100.)
    return model_catalog(model,f32(config['flaps_percent']/100.))


def left_of(mach,event):
    return mach<event['mach'] if event['side']=='start' else mach<=event['mach']


def crossings(events,left,right):
    if not left or not right or 'mach' not in left or 'mach' not in right:return []
    return [e for e in events if left_of(left['mach'],e) and not left_of(right['mach'],e)]


def annotate(solver,column):
    if column is None:return column
    events=catalog(solver)['events']
    if events and all(column.get(k) and 'mach' in column[k] for k in ('lower_boundary','boundary')):
        column['mach_branch']=[sum(not left_of(column[k]['mach'],e) for e in events)
                               for k in ('lower_boundary','boundary')]
    return column


def rough_mach(solver,mach):
    return any(a<=mach<=b for a,b in catalog(solver)['rough_intervals'])


def project_state(solver,speed,load,x,mach):
    """Small bank adjustments find the same *evaluated* Mach bin, if attainable.

    Used for derivative and corrector proposals. The coupled stencil accounts
    for its actual displacement, and acceptance uses native residuals.
    """
    from em_operating import flight_condition
    x=np.asarray(x)
    if flight_condition(solver,speed,load,x)[1]['mach']==mach:return x
    step=max(abs(float(np.spacing(np.float32(x[1])))),1e-7)
    for offset in (1.,-1.,2.,-2.,4.,-4.,8.,-8.,16.,-16.):
        q=x.copy();q[1]+=offset*step
        if not solver.trim_bounds[0][1]<=q[1]<=solver.trim_bounds[1][1]:continue
        if flight_condition(solver,speed,load,q)[1]['mach']==mach:return q
    return x


def speed_knots(info,config):
    from air_state import speed_of_sound
    scale=3.6*speed_of_sound(config['altitude_m'])
    # Wider than native Mach rounding from the body-velocity transformation.
    guard=.0036
    return [e['mach']*scale+d for e in info['events'] for d in (-guard,guard)]


def transition_width(config):
    from em_accuracy import speed_tolerance
    return max(.01,min(.05,.1*speed_tolerance(config)))


def certificate(solver,pair,sample):
    """Certify a short native formula transition by independent one-sided trims."""
    left,right=[c['boundary'] for c in pair]
    if not all(p and p.get('valid') for p in (left,right)):return None
    events=crossings(catalog(solver)['events'],left,right)
    if len(events)!=1:return None
    event=events[0]
    if left.get('roll_leveling_branch')!=right.get('roll_leveling_branch'):return None
    if right['speed_kmh']-left['speed_kmh']>transition_width(solver.config):return None
    # Bracket using measured Mach, not nominal TAS alone.
    a,b=left,right;points={p['speed_kmh']:p for p in (a,b)}
    from air_state import speed_of_sound
    target=max(.0005,8.*float(np.spacing(np.float32(event['mach'])))*3.6*speed_of_sound(solver.config['altitude_m']))
    for _ in range(10):
        if b['speed_kmh']-a['speed_kmh']<=target:break
        speed=(a['speed_kmh']+b['speed_kmh'])*.5
        p=sample(speed,a,b)
        if (p is None or not p.get('valid') or
                p.get('roll_leveling_branch')!=left.get('roll_leveling_branch')):return None
        points[speed]=p
        if left_of(p['mach'],event):a=p
        else:b=p
    else:return None
    # Validate interpolation on each one-sided outer piece, retaining its spread.
    from em_accuracy import turn_tolerance
    tolerance=turn_tolerance(solver.config['sep_tolerance_mps'])
    spread=[];uncertainty=[]
    for lo,hi in ((left,a),(b,right)):
        side_points=[lo,hi];side_errors=[]
        for fraction in (1./3.,2./3.) if lo['speed_kmh']<hi['speed_kmh'] else ():
            speed=lo['speed_kmh']+(hi['speed_kmh']-lo['speed_kmh'])*fraction
            p=sample(speed,lo,hi)
            if (p is None or not p.get('valid') or
                    p.get('roll_leveling_branch')!=lo.get('roll_leveling_branch') or
                    left_of(p['mach'],event)!=left_of(lo['mach'],event)):return None
            error=abs(p['turn_dps']-((1-fraction)*lo['turn_dps']+fraction*hi['turn_dps']))
            spread.append(error)
            side_errors.append(error);side_points.append(p)
            points[speed]=p
        if max(side_errors,default=0.)>tolerance:
            # Every retained point is independently balanced. Do not claim that
            # interpolation through quantized native values meets a smoothness
            # tolerance: retain the observed spread explicitly instead.
            uncertainty.append(dict(speed_interval_kmh=[lo['speed_kmh'],hi['speed_kmh']],
                sampled_turn_interval_dps=[min(p['turn_dps'] for p in side_points),max(p['turn_dps'] for p in side_points)],
                interpolation_error_dps=max(side_errors),rigorous_bound=False))
    return dict(speed_interval_kmh=[left['speed_kmh'],right['speed_kmh']],
        transition_speed_interval_kmh=[a['speed_kmh'],b['speed_kmh']],
        transition_kind='Mach-polar boundary transition',mach_event=event,
        method='native Mach event bracket with independently balanced one-sided samples',
        points=[points[v] for v in sorted(points)],
        one_sided_error_dps=max(spread,default=0.),
        one_sided_interpolation_verified=not uncertainty,
        native_mach_uncertainty=uncertainty,
        evaluated_mach_interval=[a['mach'],b['mach']])
