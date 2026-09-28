import math
from component_assembly import f32,add,sub,mul

TINY = f32(4e-19)
G = f32(9.81)


def detailed_path(state_flags, player_controlled):
    return bool(state_flags & 0x1000000 or player_controlled and state_flags & 0x200000)


def preprocess_angular_rate(stored_omega, longitudinal_ias):
    scale=max(1.0+float(f32(longitudinal_ias))*(-1e-5),.8)
    x,y,z=[w*scale for w in stored_omega]
    norm2=z*z+(x*x+y*y)
    if norm2>25.0:
        cap=5.0/math.sqrt(norm2)
        x,y,z=x*cap,y*cap,cap*z
    return [x,y,z]


def parasite_force(body_velocity, density, cockpit_door_cd, door_fraction,
                   attachment_drag_area=0., cockpit_intact=True):
    x,y,z=map(f32,body_velocity)
    norm2=add(mul(z,z),add(mul(x,x),mul(y,y)))
    speed=f32(math.sqrt(norm2))
    if speed>1e-9:
        reciprocal=f32(1./speed)
        direction=[mul(v,reciprocal) for v in (x,y,z)]
    else:
        direction=[0.,0.,0.]
    q=mul(norm2,mul(f32(density),.5))
    door=f32(door_fraction) if cockpit_intact else 1.
    area=add(mul(door,f32(cockpit_door_cd)),f32(attachment_drag_area))
    xy_scale=mul(area,-q)
    return [mul(xy_scale,direction[0]),mul(xy_scale,direction[1]),
            mul(mul(-q,direction[2]),area)]


def gravity_body(quaternion, mass, gravity=G):
    x,y,z,w=map(f32,quaternion); weight=mul(gravity,mass)
    negative_twice_weight=mul(-2,weight)
    return [mul(add(mul(y,x),mul(z,w)),negative_twice_weight),
            mul(add(-1,mul(2,add(mul(y,y),mul(w,w)))),-weight),
            mul(sub(mul(z,y),mul(x,w)),negative_twice_weight)]


def rotate_acceleration_to_world(quaternion, acceleration):
    x,y,z,w=map(f32,quaternion);ax,ay,az=map(f32,acceleration)
    wx2,xx2,yx2=add(w,w),add(x,x),add(y,y)
    xy2=mul(y,xx2);yz2=mul(z,yx2);wy2=mul(wx2,y)
    wz2=mul(wx2,z);xw2=mul(xx2,w);xz2=mul(z,xx2)
    yy2=mul(2,mul(y,y));ww=mul(w,w);ww2m1=add(add(ww,ww),-1)
    r00=add(add(mul(x,x),mul(x,x)),ww2m1)
    r01=sub(xy2,wz2);r02=add(xz2,wy2)
    r10=add(wz2,xy2);r11=add(yy2,ww2m1);r12=sub(yz2,xw2)
    r20=sub(xz2,wy2);r21=add(xw2,yz2);r22=add(add(mul(z,z),mul(z,z)),ww2m1)
    return [add(add(mul(r02,az),mul(ay,r01)),mul(r00,ax)),
            add(mul(az,r12),add(mul(r10,ax),mul(ay,r11))),
            add(mul(az,r22),add(mul(r21,ay),mul(ax,r20)))]


def limit_total_moment(moment, mass, gravity=G):
    x,y,z=moment; length=math.sqrt((x*x+y*y)+z*z)
    limit=float(mul(mul(gravity,mass),150))
    scale=limit/length if length>limit else 1.0
    return [scale*x,scale*y,scale*z]


def limit_aerodynamic_force(force, mass, gravity=G):
    x,y,z=force;length=math.sqrt((z*z+x*x)+y*y)
    limit=float(mul(mul(f32(gravity),f32(mass)),50))
    scale=limit/length if length>limit else 1.
    return [scale*x,scale*y,scale*z]


def angular_acceleration(omega, inertia, applied_moment, contact_moment=(0.,0.,0.)):
    wx,wy,wz=omega;ix,iy,iz=inertia
    tx,ty,tz=[contact_moment[i]+applied_moment[i] for i in range(3)]
    numerators=[(wy*wz)*(iz-iy)+tx,(wz*wx)*(ix-iz)+ty,(wy*wx)*(iy-ix)+tz]
    return [n/d if abs(d)>TINY else 0. for n,d in zip(numerators,inertia)]


def integrate_translation(position,velocity,world_acceleration,dt):
    halfdt2=(dt*dt)*.5
    return ([position[i]+(velocity[i]*dt+world_acceleration[i]*halfdt2) for i in range(3)],
            [world_acceleration[i]*dt+velocity[i] for i in range(3)])


def compose_force(aerodynamic_force,external_force,engine_force,engine_scale=1.):
    scale=float(f32(engine_scale))
    return [(aerodynamic_force[i]+float(f32(external_force[i])))+engine_force[i]*scale for i in range(3)]


def compose_moment(aerodynamic_moment,external_moment,engine_moment,gyro=(0.,0.,0.)):
    return [((aerodynamic_moment[i]+float(f32(external_moment[i])))+
             engine_moment[i])+gyro[i] for i in range(3)]


def airborne_linear_acceleration(body_force,body_gravity,mass):
    mass=f32(mass)
    inverse_mass=float(f32(1./mass)) if abs(mass)>TINY else 0.
    return [(body_gravity[i]+body_force[i])*inverse_mass for i in range(3)]


def physical_angular_vector(stored_vector):
    return [-v for v in stored_vector]


def realistic_engine_scale(ext_thrust_mult=1., ext_thrust_base_mult=1.):
    return add(mul(sub(f32(ext_thrust_mult),1),f32(ext_thrust_base_mult)),1)


def nozzle_direction_from_sincos(basis, sin_a, cos_a, sin_b, cos_b):
    b0,b1,b2=[list(map(f32,b)) for b in basis]
    sin_a,cos_a,sin_b,cos_b=map(f32,(sin_a,cos_a,sin_b,cos_b))
    s=[add(mul(b2[i],sin_a),mul(b0[i],cos_a)) for i in range(3)]
    return [add(mul(s[i],cos_b),mul(b1[i],-sin_b)) for i in range(2)]+[
        add(mul(b1[2],-sin_b),mul(s[2],cos_b))]


def accumulate_nozzle(direction, nozzle_position, cg, capped_thrust,
                      control_multiplier=1., force=(0.,0.,0.), moment=(0.,0.,0.)):
    thrust=mul(f32(control_multiplier),f32(capped_thrust))
    fx,fy,fz=[mul(f32(x),thrust) for x in direction]
    rx,ry,rz=[sub(f32(p),f32(c)) for p,c in zip(nozzle_position,cg)]
    force=list(map(f32,force));moment=list(map(f32,moment))
    accumulated_force=[add(old,new) for old,new in zip(force,(fx,fy,fz))]
    accumulated_moment=[add(mul(fy,rz),sub(moment[0],mul(fz,ry))),
                        add(moment[1],sub(mul(fz,rx),mul(fx,rz))),
                        add(sub(mul(fx,ry),mul(fy,rx)),moment[2])]
    return accumulated_force,accumulated_moment


def gyroscopic_moment(engine_angular_momentum, stored_omega, scale=1.):
    hx,hy,hz=engine_angular_momentum;wx,wy,wz=stored_omega
    return [scale*(wz*hy-hz*wy),scale*(wx*hz-hx*wz),scale*(hx*wy-wx*hy)]
