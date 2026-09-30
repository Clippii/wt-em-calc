"""Independent optical frame and zero-size target response, build 2.59.0.34.

Research component. The point response is the AAM candidate's shared numerical
kernel, not a full observation/selection policy. Signal and geometry are exposed
separately: a zero response can still pass the native kernel's zero threshold.
"""
import math
from kernels import f32, add, sub, mul, div, safe_div
from body_integration import half_angle_trig
from shared_seeker import normalize


def boresight(quaternion, angles):
    """1433212e0: polynomial half angles, local normalization, world rotation."""
    trig=[half_angle_trig(-mul(angle,.9999999403953552)) for angle in angles]
    sy,sp=[t['sine'] for t in trig]
    cy,cp=[t['cosine'] for t in trig]
    a,b,c,d=mul(sy,sp),mul(sy,cp),mul(cy,sp),mul(cy,cp)
    z=sub(mul(a,c),mul(d,b));z=add(z,z)
    y=add(mul(c,d),mul(b,a));y=add(y,y)
    x=add(mul(d,d),mul(a,a));x=sub(add(x,x),1.)
    vx,vy,vz=normalize([x,y,z],'predict')
    x,y,z,w=map(f32,quaternion)
    a=mul(add(z,z),x);b=mul(y,add(z,z));c=mul(add(z,z),w)
    d=mul(add(w,w),y);e=mul(w,add(x,x));f=mul(add(x,x),y)
    ww=sub(add(mul(w,w),mul(w,w)),1.)
    xx=add(add(mul(x,x),mul(x,x)),ww)
    yy=add(add(mul(y,y),mul(y,y)),ww)
    zz=add(add(mul(z,z),mul(z,z)),ww)
    return [add(add(mul(add(d,a),vz),mul(sub(f,c),vy)),mul(xx,vx)),
            add(add(mul(sub(b,e),vz),mul(yy,vy)),mul(add(c,f),vx)),
            add(add(mul(zz,vz),mul(add(e,b),vy)),mul(sub(a,d),vx))]


def observation_frame(quaternion, angles, origin):
    """143fe14a0 prefix: axes follow world up, with a vertical fallback."""
    forward=boresight(quaternion,angles)
    x,y,z=forward
    norm=f32(math.sqrt(add(mul(x,x),mul(z,z))))
    if norm>1e-9:
        inverse=div(1.,norm)
        rx,rz=mul(inverse,-z),mul(inverse,x)
    else:rx,rz=0.,1.
    up=[mul(-y,rz),sub(mul(x,rz),mul(z,rx)),mul(rx,y)]
    return [*forward,*up,rx,0.,rz,*map(f32,origin)]


def local_point(frame, target):
    """Native inverse-frame affine expression order, not transpose-dot-delta."""
    a,d,g,b,e,h,c,f,i,ox,oy,oz=map(f32,frame)
    tx,ty,tz=map(f32,target)
    det=sub(add(mul(mul(f,b),g),add(mul(mul(h,d),c),mul(mul(e,a),i))),
            add(mul(mul(g,e),c),add(mul(mul(f,h),a),mul(mul(b,d),i))))
    if det==0.:
        raise ValueError('Singular optical observation frame')
    inverse=div(1.,det)
    rows=((sub(mul(i,e),mul(f,h)),sub(mul(c,h),mul(b,i)),sub(mul(f,b),mul(c,e))),
          (sub(mul(g,f),mul(d,i)),sub(mul(i,a),mul(g,c)),sub(mul(c,d),mul(f,a))),
          (sub(mul(h,d),mul(g,e)),sub(mul(g,b),mul(h,a)),sub(mul(e,a),mul(b,d))))
    dx=sub(tx,ox)
    output=[]
    for row in rows:
        r0,r1,r2=[mul(v,inverse) for v in row]
        output.append(sub(add(mul(r0,dx),add(mul(r2,tz),mul(r1,ty))),
                          add(mul(r2,oz),mul(r1,oy))))
    return output


def point_response(frame,target,cosine,*,target_height=None,earth_radius_multiplier=1.):
    """14331fca0 with AAM lobe defaults and zero target bounds/centroid offset.

    Applies native front/minimum-distance/horizon gates and lobe response.
    Signature/threshold are the AAM packet's 1/0, not the later signal test.
    RangeMax belongs to scene enumeration and is intentionally not applied here.
    """
    frame=list(map(f32,frame));target=list(map(f32,target));cosine=f32(cosine)
    local=local_point(frame,target)
    delta=[sub(t,o) for t,o in zip(target,frame[9:12])]
    squared=add(add(mul(delta[2],delta[2]),mul(delta[1],delta[1])),mul(delta[0],delta[0]))
    if local[0]<0.:return dict(admitted=False,reason='behind',local=local)
    if squared<100.:return dict(admitted=False,reason='minimum_distance',local=local)
    earth=mul(earth_radius_multiplier,6371000.)
    height=target[1] if target_height is None else f32(target_height)
    h1=add(max(frame[10],1.),earth);h2=add(max(height,1.),earth)
    earth2=mul(earth,earth)
    a=sub(mul(h1,h1),earth2);b=sub(mul(h2,h2),earth2)
    root=f32(math.sqrt(mul(b,a)))
    horizon_squared=add(add(b,a),add(root,root))
    if squared>horizon_squared:
        return dict(admitted=False,reason='horizon',local=local,horizon_squared=horizon_squared)
    distance=f32(math.sqrt(squared));inverse=safe_div(1.,distance)
    unit_local=[mul(v,inverse) for v in local]
    yy=mul(unit_local[1],unit_local[1]);zz=mul(unit_local[2],unit_local[2])
    radial=add(yy,zz);cosine_squared=mul(abs(cosine),cosine)
    in_primary=sub(1.,radial)>cosine_squared
    in_secondary=sub(1.,yy)>cosine_squared
    primary=1. if in_primary else f32(math.exp(mul(-radial,1e6)))
    secondary=1. if in_secondary else f32(math.exp(mul(-yy,1e6)))
    lobe=min(primary,secondary)
    return dict(admitted=True,reason=None,local=local,world_los=[mul(v,inverse) for v in delta],
        distance=distance,inverse_distance=inverse,unit_local=unit_local,
        horizon_squared=horizon_squared,core_lobe=in_primary and in_secondary,
        lobe=lobe,response=mul(mul(inverse,inverse),lobe))
