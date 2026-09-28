import math
import numpy as np
from scipy.optimize import least_squares


def _legacy_alpha_point(solver,speed,left,right,fraction):
    if not (left['valid'] and right['valid'] and left['load_g']<right['load_g'] and
            left['alpha_deg']+1e-5<right['alpha_deg']):return None
    if any(abs(p.get('sideslip_attitude_deg',0.))>1e-8 for p in (left,right)):return None
    alpha=left['alpha_deg']*(1-fraction)+right['alpha_deg']*fraction
    guess=np.array(left['solution'])*(1-fraction)+np.array(right['solution'])*fraction
    turn=lambda n:math.sqrt(max(0.,(n-1.)*(n+1.)))
    initial=np.r_[guess[1:],turn(left['load_g']*(1-fraction)+right['load_g']*fraction)]
    initial[0]=math.degrees(math.atan(initial[4]))
    bounds=tuple([*b[1:],turn(n)] for b,n in zip(solver.trim_bounds,(left['load_g'],right['load_g'])))
    memo={}
    def value(z,frozen=None):
        key=tuple(z)


        load=math.hypot(1.,z[4])
        if frozen is not None:return solver.operating_point(speed/3.6,load,[alpha,*z[:4]],propulsion_override=frozen)
        if key not in memo:memo[key]=solver.operating_point(speed/3.6,load,[alpha,*z[:4]])
        return memo[key]
    class Closed(Exception):
        pass
    def fun(z):
        v=value(z)
        if v['force_error_g']<2e-5 and max(abs(v['rate_residual']))<5e-6:raise Closed(z)
        return v['residual']
    def jac(z):
        base=value(z);frozen=base['propulsion'] if solver.is_prop and base['propulsion']['converged'] else None
        if frozen is not None:base=value(z,frozen)
        cols=[]
        for i,h in enumerate([.002,.0002,.0002,.0002,.0002]):
            for factor in (1.,.5,-.5,2.,-1.,.1,-.1):
                q=z.copy();q[i]+=h*factor
                if i==4 and q[i]<0.:continue
                trial=value(q,frozen);used_step=h*factor
                if solver.derivative_branch(trial)==solver.derivative_branch(base):break
            cols.append((trial['residual']-base['residual'])/used_step)
        return np.column_stack(cols)
    try:
        fit=least_squares(fun,initial,jac=jac,bounds=bounds,max_nfev=12,
            x_scale=[30.,.2,.2,.2,max(1.,initial[4])],ftol=1e-9,xtol=2e-7,gtol=1e-8)
        z=fit.x
    except Closed as closed:z=closed.args[0]
    v=value(z)
    if v['force_error_g']>2e-4 or max(abs(v['rate_residual']))>5e-5:return None
    point=solver.solve(speed,math.hypot(1.,z[4]),[alpha,*z[:4]],exhaustive=False,quick=True)
    if not point['valid'] or not left['load_g']<point['load_g']<right['load_g']:return None
    if not left['alpha_deg']<point['alpha_deg']<right['alpha_deg']:return None
    point['sampling_coordinate']='angle continuation; original load/force/moment equations'
    return point


def _curve(solver, speed, seed):
    from em_continuation import TrimCurve
    cached = getattr(solver, '_interior_curve', None)
    if cached is None or cached[0] != id(solver) or cached[1] != speed:
        cached = (id(solver), speed, TrimCurve(solver, speed, seed))
        solver._interior_curve = cached
    return cached[2]


def recover_fixed_load_point(solver, speed, load, neighbors):
    if (not solver.is_prop or not solver.engine.automatic or
            abs(solver.sideslip_attitude_deg) > 1e-8):
        return None
    candidates = [p for p in neighbors if p.get('valid') and p['speed_kmh'] == speed and
        abs(p.get('sideslip_attitude_deg', 0.)) < 1e-8]
    below = [p for p in candidates if p['load_g'] < load]
    above = [p for p in candidates if p['load_g'] > load]
    if not below or not above:
        return None
    left = max(below, key=lambda p: p['load_g'])
    right = min(above, key=lambda p: p['load_g'])
    fraction = (load - left['load_g']) / (right['load_g'] - left['load_g'])
    guess = np.asarray(left['solution']) * (1. - fraction) + np.asarray(right['solution']) * fraction
    nominal = lambda n: math.degrees(math.acos(1. / n))
    guess[1] = (nominal(load) + (left['bank_deg'] - nominal(left['load_g'])) * (1. - fraction)
        + (right['bank_deg'] - nominal(right['load_g'])) * fraction)
    from em_continuation import TrimCurve
    for seed in sorted((left, right), key=lambda p: abs(p['load_g'] - load)):
        curve = TrimCurve(solver, speed, seed)
        point = curve.at_load(load, guess, max_iterations=8, max_refreshes=2)
        if point is not None and point['valid']:
            point['recovery_method'] = 'bounded fixed-load correction from certified neighbors'
            return point
    return None


def _alpha_point(solver,speed,left,right,fraction):
    pass
    if not (left['valid'] and right['valid'] and left['load_g']<right['load_g']
            and left['alpha_deg']+1e-5<right['alpha_deg']):return None
    if any(abs(p.get('sideslip_attitude_deg',0.))>1e-8 for p in (left,right)):return None
    curve=_curve(solver,speed,left)
    a,b=curve.state(left),curve.state(right)
    point=curve.correct(a+fraction*(b-a),np.eye(6)[0])
    if (point is None or not point['valid'] or not left['load_g']<point['load_g']<right['load_g']
            or not left['alpha_deg']<point['alpha_deg']<right['alpha_deg']):return None
    point['sampling_coordinate']='angle continuation; independently certified full aircraft'
    return point


def alpha_point(solver,speed,left,right):


    for fraction in (.5,.4375,.5625):
        point=_alpha_point(solver,speed,left,right,fraction)
        if point is not None:return point
    return None


def interior_point(solver,speed,left,right):
    if not (left['valid'] and right['valid'] and left['load_g']<right['load_g']):return None
    if (solver.is_prop and not solver.engine.automatic or
            any(abs(p.get('sideslip_attitude_deg',0.))>1e-8 for p in (left,right))):
        return alpha_point(solver,speed,left,right)
    curve=_curve(solver,speed,left)
    for fraction in (.5,.4375,.5625):
        point=curve.between(left,right,fraction)
        if point is not None and point['valid']:
            point['sampling_coordinate']='transverse plane across the full trim curve'
            return point
    return alpha_point(solver,speed,left,right)
