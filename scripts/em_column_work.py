import time
from em_trim_work import Work, WorkLimit, current, use


def bounded_column(task, evaluate, *, certificate=False):
    from em_sampling import worker_solver
    parent = current()

    if parent is not None and getattr(parent, 'column', None) == task[:3]:return evaluate()
    solver = worker_solver(task[0], task[1])


    limits = getattr(solver, '_column_search_limits', {})
    work = Work(limits, parent=parent, collect=True)
    work.column = task[:3]
    started = time.monotonic()
    with use(work):
        try:column = evaluate()
        except WorkLimit as failure:
            owner=failure.owner
            while owner is not None and owner is not work:owner=owner.parent
            if owner is None:raise


            if certificate:return None
            points = sorted((dict(p, surface_sample=False) for p in work.points.values()
                             if p['speed_kmh'] == task[2]), key=lambda p:p['load_g'])


            column = dict(speed_kmh=task[2], points=points, boundary=None, lower_boundary=None,
                sustained=[], load_checks=[], boundary_bracket_g=None,
                boundary_reason='Numerical search allowance exhausted',
                boundary_status='unresolved numerical boundary',
                numerical_gap_brackets=[], interior_failures=[],
                unresolved_load_intervals=[(1., solver.config['max_load_g'] or 64.)],
                elapsed_s=time.monotonic()-started, mass=solver.mass, engine=solver.engine.summary,
                search_unresolved=failure.kind+' allowance exhausted', coverage_complete=False)
    if column is not None:column['search_work'] = dict(work.counts)
    return column
