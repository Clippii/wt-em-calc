"""Deterministic native noise-table initialization recovered from client .38.

1433cb080 builds 256 entries, shuffles the permutation and repeats 258 entries.
Startup caller144392740 supplies fixed seed0x42c5ffb1. The separate time-derived
global seed in that caller does not seed these tables. No native dependencies.
"""
import math
from kernels import f32, add, mul, div


SEED = 0x42c5ffb1
MASK = 0xffffffff


def initialize(seed=SEED):
    """All four tables, retaining original draw order and float normalization.

    gradients contains the one-dimensional values used by flight inertial drift.
    Returning fresh lists avoids shared mutable tables in simulation checkpoints.
    """
    if type(seed) is not int or not 0 <= seed <= MASK:
        raise ValueError('Noise initialization needs an unsigned 32-bit seed')
    original_seed = seed
    def draw():
        nonlocal seed
        seed = (seed*0x41c64e6d + 0x3039) & MASK
        return mul(float(((seed >> 16) & 511)-256), 1/256)
    permutation = list(range(256))
    gradients, gradients2, gradients3 = [], [], []
    for _ in range(256):
        gradients.append(draw())
        x, y = draw(), draw()
        length2 = add(mul(y,y), mul(x,x))
        if length2 > 0.:
            inverse = div(1., f32(math.sqrt(length2)))
            gradients2.append([mul(x,inverse), mul(y,inverse)])
        else:
            gradients2.append([1.,0.])
        x, y, z = draw(), draw(), draw()
        length2 = add(mul(z,z), add(mul(y,y), mul(x,x)))
        if length2 > 0.:
            inverse = div(1., f32(math.sqrt(length2)))
            gradients3.append([mul(x,inverse), mul(y,inverse), mul(z,inverse)])
        else:
            gradients3.append([1.,0.,0.])
    for i in range(255,0,-1):
        seed = (seed*0x41c64e6d + 0x3039) & MASK
        j = (seed >> 16) & 255
        permutation[i], permutation[j] = permutation[j], permutation[i]
    # Native forward copying extends the original256 with itself and then0,1.
    def repeat(table):
        return [value[:] if isinstance(value,list) else value
                for value in (table+table+table[:2])]
    return dict(seed=original_seed, permutation=repeat(permutation),
                gradients=repeat(gradients), gradients2=repeat(gradients2), gradients3=repeat(gradients3))
