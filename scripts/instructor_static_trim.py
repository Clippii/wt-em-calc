from component_assembly import add, sub, mul, f32
from instructor_autotrim import autotrim_predictor
from instructor_pitch_predictor import unpack_inputs, prepare_geometry


def equilibrium_valid(auto):
    residual=auto['equilibrium']
    return (auto['success'] and abs(residual['lift_error_n'])<20.
            and abs(residual['moment_error_nm'])<20.)


def trim_independent_authority(controls, ranges):
    return all(not available or (lo == -1. and hi == 1.)
               for available, (lo, hi) in zip(controls['trim_available'], ranges))


def static_trim(solver, state, fixed):
    cache = solver.__dict__.setdefault('_static_instructor_trim_cache', {})
    packed = fixed['auto_inputs']


    key = (packed, fixed['rudder_trim'], solver.config.get('instructor_authority_mode','native'))
    if key in cache:
        return cache[key]
    ip = unpack_inputs(packed)
    predictor = state['predictor']
    geometry = prepare_geometry(solver.model, ip, predictor)
    areas = [add(add(s[1], s[0]), s[2]) for s in geometry['geometry']['areas']]
    asymmetric = (abs(sub(*areas)) > mul(add(*areas), f32(.01))
        or abs(sub(*geometry['tail_areas'])) > mul(geometry['tail_area'], f32(.01))
        or abs(predictor['f'].get(0x5328, 0.)) > f32(.1))


    if asymmetric:
        result = dict(success=False, trim=[0., 0., 0.],
                      reason='Asymmetric geometry one-g auto trim is not ported')
    else:
        direct = None
        from instructor_trim_equilibrium import solve
        direct = solve(solver.model, ip, predictor)
        if direct['success']:
            direct['trim'][2] = fixed['rudder_trim']
            if len(cache) >= 2048:cache.clear()
            cache[key] = direct
            return direct
        auto = autotrim_predictor(solver.model, ip, predictor, (0., 0., False))
        method='native iteration'
        if not equilibrium_valid(auto):


            for relaxation,iterations in ((.5,64),(.25,128)):
                auto=autotrim_predictor(solver.model,ip,predictor,(0.,0.,False),
                    lift_relaxation=relaxation,lift_iterations=iterations)
                method='relaxed algebraic lift solve'
                if equilibrium_valid(auto):break
        if not equilibrium_valid(auto):
            auto=autotrim_predictor(solver.model,ip,predictor,(0.,0.,False),
                lift_iterations=64,lift_bisection=True)
            method='bracketed algebraic lift solve'
        success=bool(equilibrium_valid(auto))
        result = dict(success=success,method=method,equilibrium=auto['equilibrium'],
            trim=[auto['output'][2], auto['output'][1], fixed['rudder_trim']],
            reason=None if success else 'One-g auto-trim equilibrium unresolved',
            one_g_alpha_deg=auto['output'][0])
        if direct is not None:
            result.update(direct_attempt=direct, fallback=True)
    if len(cache) >= 2048:
        cache.clear()
    cache[key] = result
    return result
