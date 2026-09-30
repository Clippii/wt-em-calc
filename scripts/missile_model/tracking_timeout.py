"""Pinned common-body lost-track timer, normal ownerless guidance path.

Native 14143d014..d048, 14143d1c3..d1ee and 14143d476..d47b.
The separate nonnegative guidance-event fraction and remote-owner suppression
paths are outside this helper. Destruction dispatch is reported, not executed.
"""
from kernels import f32,add


def threshold(rocket):
    return f32(rocket.get('guidance',rocket).get('breakLockMaxTime',1.))


def advance(clock,limit,dt,*,engaged,tracking):
    clock,limit,dt=map(f32,(clock,limit,dt))
    if not engaged or tracking:return dict(clock=0.,destroy=False)
    # Compare the OLD accumulator; equality gets another increment.
    if clock>limit:return dict(clock=clock,destroy=True)
    return dict(clock=add(clock,dt),destroy=False)
