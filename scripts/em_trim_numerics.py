import numpy as np
from em_trim_work import charge


class DerivativeUnavailable(Exception):
    def __init__(self, axis):
        self.axis = axis
        super().__init__('No same-branch derivative for coordinate '+str(axis))


def jacobian(x, base, evaluate, branch, steps, factors, *, inside=None, project=None, event=None, actual_step=False, allow_secant=True):
    columns = []
    displacements = []
    event_columns = []
    key = branch(base)
    extra = event(base) if event is not None else None
    for axis, h in enumerate(steps):
        matched = False
        last = None
        for factor in factors:
            q = np.asarray(x).copy();q[axis] += factor*h
            if project is not None:q = project(q)
            if inside is not None and not inside(q):continue
            step = q[axis]-x[axis] if actual_step else factor*h
            if not step:continue
            charge('derivative_samples')
            value = evaluate(q)
            column = (value['residual']-base['residual'])/step
            if not np.isfinite(column).all():continue
            last = (column, (event(value)-extra)/step if event is not None else None, (q-x)/step)
            if branch(value) != key:continue
            columns.append(column)
            displacements.append((q-x)/step)
            if event is not None:event_columns.append((event(value)-extra)/step)
            matched = True
            break
        if not matched:
            if not allow_secant or last is None:raise DerivativeUnavailable(axis)
            charge('branch_secant_predictors')
            columns.append(last[0])
            displacements.append(last[2])
            if event is not None:event_columns.append(last[1])
    matrix = np.column_stack(columns)
    if event is not None:matrix = np.vstack((matrix, event_columns))
    if project is not None:
        # A same-Mach projection can move more than the perturbed coordinate.
        # Solve the complete stencil instead of treating it as axis aligned.
        directions=np.column_stack(displacements)
        try:matrix=np.linalg.solve(directions.T,matrix.T).T
        except np.linalg.LinAlgError:raise DerivativeUnavailable(-1)
    if not np.isfinite(matrix).all():raise DerivativeUnavailable(-1)
    return matrix
