"""Readable pure-air vector block recovered from the pinned game's body step.

Component research only: no trajectory loop, seeker, launch or termination model.
"""
import math
from kernels import f32,add,sub,mul,div,safe_div,atmosphere,drag_mach
from body_perturbation import coefficients as perturbation_coefficients


def dot(a,b):return add(add(mul(a[0],b[0]),mul(a[1],b[1])),mul(a[2],b[2]))
def cross(a,b):return [sub(mul(a[1],b[2]),mul(a[2],b[1])),
                      sub(mul(a[2],b[0]),mul(a[0],b[2])),
                      sub(mul(a[0],b[1]),mul(a[1],b[0]))]
def scale(a,s):return [mul(x,s) for x in a]
def plus(a,b):return [add(x,y) for x,y in zip(a,b)]
def length(a):return f32(math.sqrt(dot(a,a)))
def unit(a,eps=4e-19,fallback=(0.,0.,0.)):
    n=length(a)
    return scale(a,div(1.,n)) if n>eps else list(fallback)


def composed_quaternion(q,p):
    x,y,z,w=q;a,b,c,d=p
    return [add(sub(add(mul(x,d),mul(a,w)),mul(b,z)),mul(c,y)),
            sub(add(add(mul(y,d),mul(b,w)),mul(a,z)),mul(c,x)),
            add(sub(mul(b,x),mul(a,y)),add(mul(z,d),mul(c,w))),
            sub(mul(d,w),add(mul(c,z),add(mul(b,y),mul(a,x))))]


def columns(q):
    x,y,z,w=q
    twice=lambda a: add(a,a)
    return [[sub(twice(add(mul(x,x),mul(w,w))),1.),
             twice(add(mul(x,y),mul(z,w))),twice(sub(mul(x,z),mul(y,w)))],
            [twice(sub(mul(x,y),mul(z,w))),
             sub(twice(add(mul(y,y),mul(w,w))),1.),twice(add(mul(y,z),mul(x,w)))],
            [twice(add(mul(x,z),mul(y,w))),twice(sub(mul(y,z),mul(x,w))),
             sub(twice(add(mul(z,z),mul(w,w))),1.)]]


def rotate_matrix(cols,v):
    return [add(add(mul(cols[0][i],v[0]),mul(cols[1][i],v[1])),mul(cols[2][i],v[2])) for i in range(3)]


def table_value(rows,x):
    if not rows:return 1.
    if x<=rows[0][0]:return rows[0][2]
    if x>=rows[-1][0]:return rows[-1][2]
    for left,right in zip(rows,rows[1:]):
        if x<=right[0]:return add(left[2],mul(sub(right[2],left[2]),mul(sub(x,left[0]),left[1])))
    raise AssertionError(x)


def forces(props,height,velocity,q,omega,*,wind=(0.,0.,0.),fins=(0.,0.),
           additional_cx=0.,additional_lever=0.,dt=1/48,torque=(0.,0.,0.),
           force=(0.,0.,0.),mass_lost=0.,gravity=True,use_cxi=True,
           mass_term=0.,angular_environment=(0.,0.,0.),perturbation=0.,body_random=0.):
    """Evaluate the full pure-air branch before integration, with float32 vectors.

    Requires zero immersion and the loaded
    mass/inertia/property quaternion. The separately supplied environment mass
    term and angular vector follow native arithmetic, without inferred units.
    Native CRT pow/exp and vector summation can differ by a few ULPs.
    """
    velocity,q,omega,wind,fins,torque,force=[list(map(f32,x)) for x in (velocity,q,omega,wind,fins,torque,force)]
    relative=[sub(v,w) for v,w in zip(velocity,wind)]
    cm_speed64=math.sqrt(sum(float(x)**2 for x in relative));speed=f32(cm_speed64)
    composed=composed_quaternion(q,props['axis_quaternion']);frame=columns(composed)
    axes=[]
    for col,order in zip(frame,((2,1,0),(2,0,1),(0,1,2))):
        a,b,c=[mul(col[i],col[i]) for i in order]
        n=f32(math.sqrt(add(add(a,b),c)))
        axes.append(scale(col,div(1.,n)) if n>f32(4e-19) else [0.,0.,0.])
    # Raw frame diagonals are formed differently from the separately normalized
    # direction axes in the original routine; preserve this rounding boundary.
    ww=sub(mul(2.,mul(composed[3],composed[3])),1.)
    for i in range(3):frame[i][i]=add(mul(2.,mul(composed[i],composed[i])),ww)
    forward=axes[0]
    fallback=[f32(x/cm_speed64) for x in relative] if speed>f32(.001) else forward
    arm=add(props['stabilizer_arm'],additional_lever)
    perturb=perturbation_coefficients(perturbation,body_random)
    offset=mul(mul(perturb['lever_fraction'],body_random),props['stabilizer_arm'])
    cosine_offset=mul(perturb['cosine'],offset);sine_offset=mul(offset,perturb['sine'])
    bx=sub(mul(omega[2],sine_offset),mul(omega[1],cosine_offset))
    by=add(mul(omega[2],arm),mul(omega[0],cosine_offset))
    bz=sub(mul(omega[1],-arm),mul(omega[0],sine_offset))
    local=[add(add(mul(frame[2][0],bz),relative[0]),mul(frame[1][0],by)),
           add(add(mul(frame[2][1],bz),relative[1]),mul(frame[1][1],by)),
           add(add(relative[2],mul(frame[1][2],by)),mul(frame[2][2],bz))]
    local=[add(v,mul(frame[0][i],bx)) for i,v in enumerate(local)]
    flow=unit(local,eps=1e-9,fallback=fallback)
    air=atmosphere(height);mach=div(speed,air['sound_speed'])
    z=min(f32(height),18300.);poly=f32(2.28719e-19)
    for coefficient in (-5.83556e-14,3.53118e-9,-9.59387e-5,1.):poly=add(mul(poly,z),coefficient)
    pressure=div(mul(mul(speed,speed),mul(mul(mul(1.225,.5),18300.),poly)),max(f32(height),18300.))
    k=drag_mach(mach);ki=add(mul(k,2.4352500438690186),.25001898407936096) if use_cxi else 1.
    cx_total=add(props['cx'],additional_cx)
    cx_mach=div(mul(mach,cx_total),add(mul(add(mul(.356,mach),2.237),mach),-1.4)) if f32(1.4)<=mach<4. else mul(cx_total,k)
    cy=mul(props['cy'],table_value(props['cy_table'],mach))

    def evaluate(direction):
        cosine=dot(direction,forward);sin2=max(sub(1.,mul(cosine,cosine)),0.)
        lift_coefficient=mul(f32(math.sqrt(sin2)),cy);cap=props['cy_limit']
        if lift_coefficient>cap:lift_coefficient=max(sub(add(cap,cap),lift_coefficient),0.)
        elif lift_coefficient< -cap:lift_coefficient=min(sub(mul(-2.,cap),lift_coefficient),0.)
        cd=add(cx_mach,mul(mul(sin2,mul(props['cx'],props['cx_aoa'])),ki))
        drag=scale(direction,mul(cd,mul(props['front_area'],-pressure)))
        lift_direction=unit(cross(direction,cross(direction,forward)),eps=1e-9)
        lift_scalar=mul(lift_coefficient,mul(props['side_area'],-pressure))
        if cosine<0:lift_scalar=-lift_scalar
        lift=scale(lift_direction,lift_scalar)
        return dict(drag=drag,lift=lift,force=plus(drag,lift),cosine=cosine,cd=cd,cy=lift_coefficient)

    baseline=evaluate(flow);deflection=[mul(fins[0],props['fins_hor']),mul(fins[1],props['fins_ver'])]
    active=(mul(abs(fins[0]),props['fins_hor'])>f32(1e-5) or
            mul(abs(fins[1]),props['fins_ver'])>f32(1e-5))
    limited=False;fin_flow=flow;steering=baseline
    if active:
        size2=add(mul(deflection[0],deflection[0]),mul(deflection[1],deflection[1]))
        limit=props['fin_pressure_limit']
        if mul(mul(pressure,pressure),size2)>mul(limit,limit):
            factor=div(1.,mul(pressure,f32(math.sqrt(size2))))
            deflection=[mul(mul(x,limit),factor) for x in deflection];limited=True
        fin_flow=unit(plus(plus(flow,scale(axes[2],deflection[0])),scale(axes[1],deflection[1])),
                      eps=1e-9,fallback=fallback)
        steering=evaluate(fin_flow)
    body_force=[dot(c,steering['force']) for c in frame]
    fx,fy,fz=body_force
    moment=[sub(mul(fy,cosine_offset),mul(fz,sine_offset)),
            sub(mul(-arm,fz),mul(fx,cosine_offset)),
            add(mul(arm,fy),mul(fx,sine_offset))]
    raw_torque=plus(moment,torque)
    damping=[];damping_clipped=[];inertia=props['inertia'];dt=f32(dt)
    qgeom=mul(pressure,props['damping_geometry'])
    for i in range(3):
        d=mul(mul(mul(-.01 if i==0 else -.05,omega[i]),qgeom),props['angular_damping'][i])
        stopping=add(mul(mul(omega[i],inertia[i]),div(1.,dt)),raw_torque[i])
        if i==0:stopping=add(add(mul(mul(omega[i],inertia[i]),div(1.,dt)),torque[i]),moment[i])
        clipped=abs(stopping)<abs(d)
        if clipped:d=-stopping
        damping_clipped.append(clipped)
        damping.append(d)
    gyro=[mul(mul(omega[2],omega[1]),sub(inertia[2],inertia[1])),
          mul(sub(inertia[0],inertia[2]),mul(omega[2],omega[0])),
          mul(mul(omega[0],omega[1]),sub(inertia[1],inertia[0]))]
    alpha=[div(add(add(t,g),d),i) for t,g,d,i in zip(raw_torque,gyro,damping,inertia)]
    alpha[0]=div(add(add(add(gyro[0],torque[0]),moment[0]),damping[0]),inertia[0])
    base_alpha=alpha[:]
    # This block uses the BODY quaternion, not the aerodynamic-axis composition.
    # Its x/z angular additions execute even with translational gravity disabled.
    x,y,z,w=q;ex,ey,ez=map(f32,angular_environment)
    zx=mul(add(z,z),x);yw=mul(add(y,y),w);ey2=add(ey,ey)
    ww=sub(add(mul(w,w),mul(w,w)),1.)
    plus_x=add(mul(add(add(mul(z,z),mul(z,z)),ww),ez),
               add(mul(add(mul(z,y),mul(x,w)),ey2),mul(sub(zx,yw),ex)))
    plus_z=sub(sub(mul(sub(mul(z,w),mul(y,x)),ey2),
                      mul(add(add(mul(x,x),mul(x,x)),ww),ex)),mul(add(yw,zx),ez))
    alpha=[add(mul(plus_x,9.81),alpha[0]),alpha[1],add(mul(plus_z,9.81),alpha[2])]
    mass=sub(add(mul(props['mass'],mass_term),props['mass']),mass_lost);inverse=safe_div(1.,mass)
    # Original translation uses pre-fin drag/lift, adding their quantized
    # components in double precision before inverse-mass multiplication.
    acceleration=[((float(d)+float(l))*perturb['force_scale']+f)*inverse for d,l,f in zip(baseline['drag'],baseline['lift'],force)]
    if gravity:acceleration[1]-=float(mul(mass,9.81))*inverse
    n2=sum(x*x for x in acceleration)
    if n2>=6000.**2:acceleration=[x*math.sqrt(6000.**2/n2) for x in acceleration]
    return dict(frame=frame,axes=axes,cm_speed=speed,mach=mach,pressure=pressure,local_flow=local,flow=flow,
                baseline=baseline,fin_active=active,fin_limited=limited,deflection=deflection,
                fin_flow=fin_flow,steering=steering,moment=moment,damping=damping,damping_clipped=damping_clipped,
                angular_acceleration_before_environment=base_alpha,
                angular_acceleration=alpha,acceleration=acceleration,effective_mass=mass,
                perturbation=perturb,perturbed_lever=[arm,-sine_offset,-cosine_offset])
