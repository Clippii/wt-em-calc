import numpy as np
from scipy.optimize import root

from instructor_autotrim import autotrim_predictor
from instructor_pitch_predictor import prepare_geometry

REVISION = 'steady-authority-direct-v1'


def solve(model, ip, state):
    c = prepare_geometry(model, ip, state)
    weight = max(abs(ip[0x2c]), 1.)
    moment_scale = weight * max(abs(c['tail_lever']), 1.)
    p = c['polars'][0]
    cl = weight / max(sum(c['q_area']) * c['cos_dihedral'], 1.)
    slope = p['clLineCoeff'] * p['clKq']
    angle = (cl - p['cl0'] * p['clKq']) / slope if abs(slope) > 1e-9 else 0.
    angle = min(p['aoaCritH'], max(p['aoaCritL'], angle))
    calls = 0
    memo = {}

    def evaluate(x):
        nonlocal calls
        key = tuple(map(float, x))
        if key not in memo:
            calls += 1
            auto = autotrim_predictor(model, ip, state,
                equilibrium_state=(x[0], x[1] * weight, x[2]))
            eq = auto['equilibrium']
            residual = np.array([eq['lift_error_n'] / weight,
                eq['moment_error_nm'] / moment_scale, eq['command_error']])
            memo[key] = residual, auto
        return memo[key]

    def fun(x):return evaluate(x)[0]

    def jac(x):


        base = fun(x)
        columns = []
        for i, step in enumerate((.002, 2e-5, .0002)):
            shifted = np.array(x, dtype=float); shifted[i] += step
            columns.append((fun(shifted) - base) / step)
        return np.column_stack(columns)

    fit = root(fun, [angle, 0., 0.], jac=jac, method='hybr',
               options=dict(xtol=2e-6, maxfev=48, factor=10.))
    _, auto = evaluate(fit.x)
    eq = auto['equilibrium']

    success = (auto['success'] and abs(eq['lift_error_n']) < 20.
        and abs(eq['moment_error_nm']) < 20. and abs(eq['command_error']) < 2e-6
        and abs(auto['output'][1]) <= 1. and np.isfinite(fit.x).all())
    return dict(success=bool(success), method='simultaneous reduced equilibrium',
        equilibrium=eq, trim=[auto['output'][2], auto['output'][1], 0.],
        one_g_alpha_deg=auto['output'][0], evaluations=calls,
        reason=None if success else 'Direct one-g equilibrium unresolved',
        revision=REVISION)
