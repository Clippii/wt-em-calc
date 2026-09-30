"""Isolated native-game acceleration-control arithmetic, explicit frame setting.

Loaded properties and independently evaluated scalar motor outputs are inputs.
No trajectory, seeker, or motor scheduling is assembled here.
"""
import math
from kernels import f32, add, sub, mul, div, safe_div
from control_frame import frame
from guidance_request import table_value


def lateral_axes(q):
    x, y, z, w = map(f32, q)
    hx = mul(2., add(mul(z,x), mul(w,y)))
    t = mul(add(z,z), y); u = mul(mul(-2.,x),w)
    hy = add(u,t)
    common = add(mul(2.,mul(w,w)), -1.)
    hz = add(mul(2.,mul(z,z)), common)
    vx = mul(2.,sub(mul(x,y),mul(w,z)))
    vy = add(mul(2.,mul(y,y)),common); vz = sub(t,u)
    return [hx,hy,hz], [vx,vy,vz]


def project(value, horizontal, vertical):
    x,y,z = map(f32,value); hx,hy,hz = horizontal; vx,vy,vz = vertical
    return [add(add(mul(hz,z),mul(hx,x)),mul(hy,y)),
        add(mul(vz,z),add(mul(vy,y),mul(vx,x)))]


def density_polynomial(height,height_scale=18300.):
    z = min(f32(height),f32(height_scale))
    a = f32(2.28719e-19)
    for c in (-5.83556e-14,3.53118e-9,-9.59387e-5,1.): a=add(mul(a,z),c)
    return a


def atmosphere_values(height,environment):
    """Density/sound consumers with the separately loaded native world values."""
    height=f32(height);ceiling=f32(environment['height_scale']);z=min(height,ceiling)
    density=div(mul(mul(ceiling,environment['reference_density']),density_polynomial(height,ceiling)),max(height,ceiling))
    poly=f32(3.97306e-18)
    for c in (-5.71104e-14,2.18069e-10,-2.27712e-5,1.):poly=add(mul(poly,z),c)
    sound=mul(f32(math.sqrt(mul(poly,environment['sea_temperature']))),20.1)
    return dict(density=density,sound_speed=sound)


def pid_coefficients(p,time):
    schedule=p['schedule']; coefficients=p['coefficients']; time=f32(time)
    if not schedule:
        return coefficients[0] if len(coefficients)==1 else None
    if len(schedule)==1 or time<=schedule[0][0]: return coefficients[schedule[0][2]]
    if time>=schedule[-1][0]: return coefficients[schedule[-1][2]]
    for a,b in zip(schedule,schedule[1:]):
        if time<=b[0]:
            fraction=mul(sub(time,a[0]),a[1])
            return [add(mul(sub(y,x),fraction),x) for x,y in zip(coefficients[a[2]],coefficients[b[2]])]
    raise ValueError('Invalid schedule')


def update(p,aero,motor,state,request,measured,q,velocity,height,time,dt,*,environment=None,matrix_velocity_frame=True):
    environment=dict(height_scale=18300.,reference_density=1.225,sea_density=1.225,sea_temperature=288.16) | (environment or {})
    reference=frame(q,velocity,p['velocity_frame'],matrix=matrix_velocity_frame)
    h,v=lateral_axes(reference)
    wanted=project(request,h,v); squared=add(mul(wanted[1],wanted[1]),mul(wanted[0],wanted[0]))
    vx,vy,vz=map(f32,velocity); speed_squared=add(mul(vz,vz),add(mul(vy,vy),mul(vx,vx)))
    limit=p['max_accel']; aoa_limit=None
    if aero is not None and p['limit_aoa']:
        motor=motor or dict(thrust=0.,mass_lost=0.)
        mass=sub(aero['mass'],motor['mass_lost'])
        table=aero['cy_table']
        # This controller uses speed SQUARED / sound speed as the table argument.
        # Preserve it; the body's own Mach consumer is a separate calculation.
        cy_mult=table_value(table,div(speed_squared,atmosphere_values(height,environment)['sound_speed'])) if table else 1.
        factor=mul(environment['reference_density'],.5)
        for x in (speed_squared,environment['height_scale'],density_polynomial(height,environment['height_scale']),cy_mult,aero['cy'],aero['side_area']): factor=mul(factor,x)
        factor=div(factor,max(f32(height),f32(environment['height_scale'])))
        aoa_limit=div(mul(add(factor,motor['thrust']),min(p['aoa_max'],aero['cy_limit'])),mass)
        limit=min(limit,aoa_limit)
    limit_squared=mul(limit,limit)
    if squared>limit_squared:
        reduction=f32(math.sqrt(div(limit_squared,squared)))
        wanted=[mul(x,reduction) for x in wanted]
    scale_squared=1.
    if p['base_speed_squared']>0.:
        rho=atmosphere_values(height,environment)['density']
        scale_squared=safe_div(mul(mul(environment['sea_density'],environment['sea_density']),p['base_speed_squared']),mul(mul(rho,rho),speed_squared))
    scale=f32(math.sqrt(scale_squared)); coefficients=pid_coefficients(p,time)
    if coefficients is None: return dict(frame=reference,fins=None,state=state)
    # Measurement projection order differs slightly from request projection.
    mx,my,mz=map(f32,measured)
    actual=[add(mul(h[1],my),add(mul(h[2],mz),mul(h[0],mx))),
        add(mul(v[2],mz),add(mul(v[1],my),mul(v[0],mx)))]
    errors=[sub(a,b) for a,b in zip(wanted,actual)]
    kp,ki,kd,ilim=coefficients; kp=mul(kp,scale); ki=mul(ki,scale); kd=mul(kd,scale_squared)
    outputs=[]; states=[]
    for index,(previous,error) in enumerate(zip(state,errors)):
        integral=min(ilim,max(-ilim,add(mul(mul(error,dt),ki),previous[3])))
        derivative=add(mul(sub(error,add(mul(dt,previous[7]),previous[6])),48.),previous[7])
        proportional=mul(kp,error); differential=mul(derivative,kd)
        value=add(add(differential,proportional),integral) if index==0 else add(add(integral,proportional),differential)
        outputs.append(min(1.,max(-1.,value)))
        states.append([kp,ki,ilim,integral,kd,48.,error,derivative])
    return dict(frame=reference,fins=outputs,state=states,errors=errors,limited_request=wanted,
        aoa_limit=aoa_limit,scale=scale,scale_squared=scale_squared,coefficients=coefficients)
