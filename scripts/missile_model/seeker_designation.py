"""Independent shared designation pointing, native 1433226d0 (2.59.0.34).

Input direction/rate are already in missile body coordinates. Aircraft sensor
selection and provider-frame conversion are outside this component.
"""
import math
from kernels import f32, add, mul
from optical_geometry import boresight
from motor_vector import rotate_thrust
from native_atan2f import atan2f


SOURCE_NAMES=('dummy','crosshair','camera','helmet','ins','sensor','radar','radarSs','esm','shellFpv')


def source_id(name):
    """Case-sensitive enum conversion from native 14331f840."""
    return SOURCE_NAMES.index(name) if name in SOURCE_NAMES else 255


def source_mask(values, default=0):
    """14331f960: recognized/empty strings replace, not augment, default."""
    if values is None:
        return default & 0xffffffff
    values=[values] if isinstance(values,str) else values
    mask=0;replace=False
    for value in values:
        if not isinstance(value,str):
            raise ValueError('Designation source entries must be strings')
        index=source_id(value)
        if index<16:
            mask |= 1<<index
            replace=True
        elif value=='':
            replace=True
    return mask if replace else default & 0xffffffff


def masks(config):
    normal=source_mask(config.get('designationSourceType'),config.get('designationSourceTypeMask',0)) & 0xffff
    constant=source_mask(config.get('constantDesignationSourceType'),
                         config.get('constantDesignationSourceTypeMask',normal)) & 0xffff
    return dict(designation_mask=normal,constant_mask=constant)


def allowed(mode, source, designation_mask, constant_mask, additional_mask=0):
    """143fe6c00 register-BT routing; authored masks are stored as uint16."""
    if mode not in (1,2,4,5,6,7):
        return False
    mask=constant_mask if mode==5 else designation_mask
    mask=((mask & 0xffff) | additional_mask) & 0xffffffff
    return bool(mask & (1 << (source & 31)))


def point(p, quaternion, direction, angular_rate, previous_angles, *, gate_previous=True):
    q=list(map(f32,quaternion))
    x,y,z=map(f32,direction)
    angles=[atan2f(-z,x),
            atan2f(y,f32(math.sqrt(add(mul(z,z),mul(x,x)))))]
    limit=p['lock_angle_max']
    tested=list(map(f32,previous_angles)) if gate_previous else angles
    accepted=all(abs(a)<limit for a in tested)
    angles=[min(limit,max(-limit,a)) for a in angles]
    rate=rotate_thrust(q,list(map(f32,angular_rate)))
    return dict(angles=angles,direction=boresight(q,angles),
                angular_rate=[min(10.,max(-10.,a)) for a in rate],accepted=accepted)
