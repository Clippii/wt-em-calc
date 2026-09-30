"""Recovered *game* scalar laws, tied to aces.exe 2.59.0.34.

These are not real missile models. See README.md for evidence and coverage.
"""
from dataclasses import dataclass
import math
import struct


def f32(x):
    """Round to binary32 with masked overflow, nearest/even, gradual underflow.

    Native arithmetic stores signed infinity on overflow. Python's struct
    conversion instead raises for a finite out-of-range double. Floating-point
    status flags, trapping modes and DAZ/FTZ are outside this helper's contract.
    """
    try:
        return struct.unpack('<f',struct.pack('<f',x))[0]
    except OverflowError:
        return math.copysign(math.inf,x)


def add(a,b): return f32(f32(a)+f32(b))
def sub(a,b): return f32(f32(a)-f32(b))
def mul(a,b): return f32(f32(a)*f32(b))
def div(a,b): return f32(f32(a)/f32(b))


def safe_div(a,b):
    return div(a,b) if abs(f32(b))>f32(4e-19) else 0.


def atmosphere(height):
    """Native polynomial atmosphere, including the above-18.3-km density tail."""
    h=f32(height); z=min(h,f32(18300.))
    def poly(coefficients):
        value=f32(coefficients[0])
        for c in coefficients[1:]: value=add(mul(value,z),c)
        return value
    density=div(mul(mul(1.225,18300.),poly([2.28719e-19,-5.83556e-14,3.53118e-9,-9.59387e-5,1.])),max(h,18300.))
    sound=mul(f32(math.sqrt(mul(poly([3.97306e-18,-5.71104e-14,2.18069e-10,-2.27712e-5,1.]),288.16))),20.1)
    pressure=div(mul(mul(101300.,18300.),poly([1.60373e-18,-1.3738e-13,5.6763e-9,-1.18441e-4,1.])),max(h,18300.))
    return dict(density=density,sound_speed=sound,pressure=pressure)


def drag_mach(mach):
    """Native zero-incidence Mach shape; CRT powf/expf can differ by a ULP."""
    m=f32(mach)
    if m<f32(.61): return f32(.308)
    if m<1.: return add(mul(.505,f32(math.pow(sub(m,.61),f32(2.31)))),.308)
    if m<f32(1.4):
        d=sub(m,1.)
        return add(mul(mul(.4485,f32(math.pow(d,f32(.505)))),f32(math.exp(mul(d,-5.68)))),.551)
    if m<4.: return div(m,add(mul(add(mul(.356,m),2.237),m),-1.4))
    return f32(.302)


def aero_properties(rocket, globals_=None):
    """Port of 0x1435e8400's scalar geometry, inertia and control scaling."""
    params=globals_ or dict(applyWingAreaMultToCxAoA=False,props=dict(CxAoA=9.))
    defaults=params.get('props',{})
    d=f32(rocket.get('caliber',.3683));m=f32(rocket.get('mass',248.2))
    length=f32(rocket.get('length',mul(d,defaults.get('caliberToLength',4.))))
    wing=f32(rocket.get('wingAreaMult',1.))
    front=mul(mul(d,d),f32(math.pi/4))
    side=mul(mul(mul(.3,wing),length),d)
    arm=mul(safe_div(1.,wing),rocket.get('distFromCmToStab',mul(length,defaults.get('distFromCmToStabToLength',.3))))
    cy=f32(rocket.get('CyK',defaults.get('CyK',2.2)))
    r=mul(d,.5);r2=mul(r,r)
    axial=mul(m,r2)
    if r<mul(5.,length): transverse=mul(mul(add(mul(length,length),mul(3.,r2)),f32(1/12)),m)
    else:
        axial=mul(axial,.4)
        transverse=mul(mul(add(mul(mul(length,.5),mul(length,.5)),r2),.2),m)
    return dict(caliber=d,mass=m,inertia=[axial,transverse,transverse],front_area=front,
        side_area=side,stabilizer_arm=arm,length=length,cx=f32(rocket.get('CxK',defaults.get('CxK',.2))),
        cy=cy,cy_limit=f32(rocket.get('CyMaxAoA',defaults.get('CyMaxAoA',1.))),
        cx_aoa=mul(mul(wing,wing) if params.get('applyWingAreaMultToCxAoA',True) else 1.,rocket.get('CxAoA',defaults.get('CxAoA',.3))),
        fluid_resistance=f32(rocket.get('fluidResistanceMultiplier',1.)),
        fluid_rotation_resistance=f32(rocket.get('fluidRotationResistanceMultiplier',0.)),
        fins_hor=f32(rocket.get('finsAoaHor',0.)),fins_ver=f32(rocket.get('finsAoaVer',0.)),
        fin_pressure_limit=safe_div(mul(mul(rocket.get('finsLatAccel',100.),9.81),m),mul(side,cy)),
        angular_damping=[f32(x) for x in rocket.get('WdK',defaults.get('WdK',[1.,1.,1.]))],
        damping_geometry=mul(mul(length,length),front))


@dataclass(frozen=True)
class Impulse:
    duration: float
    force: float
    mass_lost: float
    vectoring: float=0.
    factor_index: int=-1


@dataclass(frozen=True)
class Propulsion:
    delay: float
    impulses: tuple


def motor_properties(rocket):
    """Authored AAM motor layouts; reject unimplemented factor-table loading."""
    m=f32(rocket['mass']);props=[]
    def impulse(row,previous_mass,previous_time):
        duration=f32(row.get('time',sub(row.get('timeEnd',previous_time),previous_time)))
        lost=f32(row.get('massLost',sub(previous_mass,row.get('massEnd',previous_mass))))
        force=f32(row.get('force',0.));flow=f32(row.get('massFlow',0.));isp=f32(row.get('isp',0.))
        if flow>0 and isp>0: force=mul(flow,isp);duration=safe_div(lost,flow)
        index=int(row.get('factorIndex',-1))
        if index>=0: raise NotImplementedError('Propulsion factor tables need their native loader')
        return Impulse(duration,force,lost,f32(row.get('thrustVectoringAngle',0.)),index)
    if 'propulsion0' in rocket:
        for i in range(4):
            row=rocket.get(f'propulsion{i}')
            if row is None: break
            impulses=[];previous=m;elapsed=0.
            for j in range(4):
                part=row.get(f'impulse{j}')
                if part is None: break
                item=impulse(part,previous,elapsed);impulses.append(item)
                previous=sub(previous,item.mass_lost);elapsed=add(elapsed,item.duration)
            props.append(Propulsion(f32(row.get('fireDelay',0.)),tuple(impulses)))
    else:
        previous=m;impulses=[]
        for suffix in ('','1'):
            if 'timeFire'+suffix not in rocket: continue
            end=f32(rocket.get('massEnd'+suffix,previous))
            lost=sub(m,add(end,impulses[0].mass_lost)) if suffix else sub(m,end)
            row=dict(time=rocket['timeFire'+suffix],force=rocket.get('force'+suffix,0. if suffix else 500.),
                massLost=lost,thrustVectoringAngle=rocket.get('thrustVectoringAngle'+suffix,rocket.get('thrustVectoringAngle',0.)),
                factorIndex=rocket.get('factorIndex',-1),massFlow=rocket.get('massFlow'+suffix,0.),isp=rocket.get('isp'+suffix,0.))
            item=impulse(row,previous,0.);impulses.append(item);previous=end
        props.append(Propulsion(f32(rocket.get('fireDelay',0.)),tuple(impulses)))
    return tuple(props)


def motor_scalar(propulsions,clocks):
    """Port of 0x143af32a0 for factorIndex=-1; clocks are per propulsion."""
    if len(propulsions)!=len(clocks): raise ValueError('One clock per propulsion is required')
    thrust=lost=vectoring=0.
    for propulsion,clock in zip(propulsions,clocks):
        t=sub(clock,propulsion.delay);elapsed=0.;used=0.;force=angle=0.
        for item in propulsion.impulses:
            end=add(elapsed,item.duration)
            if end>t:
                local=sub(t,elapsed)
                if local>0:
                    force=item.force;angle=item.vectoring
                    used=add(used,mul(safe_div(local,item.duration),item.mass_lost))
                break
            used=add(used,item.mass_lost);elapsed=end
        thrust=add(thrust,force);lost=add(lost,used);vectoring=add(vectoring,angle)
    return dict(thrust=thrust,mass_lost=lost,vectoring=vectoring)


def pressure_multiplier(rocket,height):
    a,ya,b,yb=[f32(x) for x in rocket.get('extPressureToThrustMult',[0.,1.,0.,1.])]
    if a>b:a,b,ya,yb=b,a,yb,ya
    p=atmosphere(height)['pressure']
    if p<=a:return ya
    if p>=b:return yb
    return add(ya,safe_div(mul(sub(p,a),sub(yb,ya)),sub(b,a)))


@dataclass
class PID:
    """Native single-axis controller state; derivative filter coefficient is 48."""
    integral: float=0.
    previous_error: float=0.
    derivative: float=0.

    def update(self,error,dt,p,i,d,limit,scale=1.):
        error=f32(error);dt=f32(dt)
        p=mul(p,scale);i=mul(i,scale);d=mul(d,mul(scale,scale))
        self.integral=min(f32(limit),max(f32(-limit),add(self.integral,mul(mul(error,dt),i))))
        self.derivative=add(mul(sub(error,add(mul(dt,self.derivative),self.previous_error)),48.),self.derivative)
        self.previous_error=error
        return min(1.,max(-1.,add(add(mul(self.derivative,d),mul(p,error)),self.integral)))
