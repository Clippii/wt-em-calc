import numpy as np


def variable_blocks(properties, paths):
    count = len(properties['transmissions'])
    parent = list(range(count))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    owners = {}
    for shaft, transmission in enumerate(properties['transmissions']):
        for kind in ('engines', 'propellers'):
            for link in transmission[kind]:
                key = (kind, link['index'])
                if key in owners:parent[root(shaft)] = root(owners[key])
                else:owners[key] = shaft
    groups = {}
    for column, path in enumerate(paths):
        kind, index = path[:2]
        shaft = owners.get(('propellers', index)) if kind == 'flow' else index
        if shaft is None or not 0 <= shaft < count:
            return (tuple(range(len(paths))),)
        groups.setdefault(root(shaft), []).append(column)
    return tuple(tuple(group) for group in groups.values())


def grouped_jacobian(evaluate, x, bounds, blocks, step_size=2e-4):
    base = evaluate(x)
    matrix = np.zeros((len(base), len(x)))
    delta = np.where(x + step_size < bounds[1], step_size, -step_size)
    for color in range(max(map(len, blocks), default=0)):
        columns = [block[color] for block in blocks if color < len(block)]
        q = x.copy()
        q[columns] += delta[columns]
        change = evaluate(q) - base
        for block in blocks:
            if color < len(block):
                column = block[color]
                rows = list(block)
                matrix[rows, column] = change[rows] / delta[column]
    return matrix
