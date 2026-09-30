"""Finite-input arithmetic of pinned client CRT atan2f, address 146b63468.

Float inputs, double intermediates, table reduction and final float conversion.
No native calls. Floating-point exception flags/errno and NaNs are not modeled.
"""
import hashlib
import json
import math
from pathlib import Path
import struct
from kernels import f32


def load_table():
    rows=json.loads(Path(__file__).with_name('atan2f-table.json').read_text())['rows']
    raw=struct.pack('<241d',*rows)
    if hashlib.sha256(raw).hexdigest()!='15753064342e1160c8d975d22c802d2ffac899e85825013f57a0c6aba64819d3':
        raise ValueError('atan2f table differs from pinned executable')
    return tuple(rows)


TABLE=load_table()


def atan2f(y,x):
    y,x=f32(y),f32(x)
    if not math.isfinite(y) or not math.isfinite(x):
        raise ValueError('Only finite atan2f inputs are reconstructed')
    negative_x=math.copysign(1.,x)<0.;negative_y=math.copysign(1.,y)<0.
    ax,ay=abs(x),abs(y)
    if ay==0.:return math.copysign(f32(math.pi),y) if negative_x else y
    if ax==0.:return math.copysign(f32(math.pi/2),y)
    delta=math.frexp(ay)[1]-math.frexp(ax)[1]
    if delta>26:return math.copysign(f32(math.pi/2),y)
    if delta< -13 and not negative_x:
        # The native underflow path preserves finite quotient rounding; errno
        # and exception status are deliberately outside this numerical port.
        return f32(y/x)
    if delta< -26 and negative_x:return math.copysign(f32(math.pi),y)
    swapped=ay>ax
    if swapped:ay,ax=ax,ay
    ratio=ay/ax
    if ratio>.0625:
        index=math.trunc(ratio*256.+.5)
        residual=(ay*256.-float(index)*ax)/(float(index)*ay+ax*256.)
        angle=residual+TABLE[index-16]
        angle-=((residual*residual)*residual)*.33333333333224097
    elif ratio>=.0001:
        squared=ratio*ratio
        coefficient=.19999999999393223-squared*.1428571356180717
        coefficient=.3333333333333317-coefficient*squared
        angle=ratio-coefficient*(squared*ratio)
    else:angle=ratio
    if swapped:angle=math.pi/2-angle
    if negative_x:angle=math.pi-angle
    if negative_y:angle=-angle
    return f32(angle)
