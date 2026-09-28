from component_assembly import f32,add,sub,mul
from functools import lru_cache
from piston_model import div,boost_active
from piston_general import step as piston_step

from structural_limits import interval
from air_state import cache,speed_of_sound
from turbine_general import step as turbine_step
from rocket_general import step as rocket_step


def delivered_boost(p,s,nitro=0.):

    return boost_active(p,s.get('effective_throttle',0.),s.get('afterburner',False),s.get('gear',0),nitro)


def target_omega(p,s,nitro=0.):
    throttle=f32(s.get('throttle',1.))
    if p['boost_type']>0 and p['boost_controllable']:
        throttle=f32(1.1) if delivered_boost(p,s,nitro) else min(throttle,1.)
    rows=p['rpm_targets']
    if not rows:return 0.
    if throttle<=rows[0][0]:return rows[0][1]
    for lo,hi in zip(rows,rows[1:]):
        if throttle<hi[0]:return interval(throttle,*lo,*hi)
    return rows[-1][1]


@lru_cache(maxsize=512)
def fixed_air(velocity,height):
    return cache(velocity,height),speed_of_sound(height)


