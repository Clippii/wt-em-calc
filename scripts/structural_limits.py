import math,struct
from component_assembly import f32,add,sub,mul


def wing_load_ratios(wing_y_forces,crit_overload):
    reciprocal=[f32(1./f32(x)) if abs(f32(x))>f32(4e-19) else 0. for x in crit_overload]
    return [max(mul(f32(force),r) for r in reciprocal) for force in wing_y_forces]


def random_fraction(seed):
    seed=(seed+0x9e3779b9)&0xffffffff
    mixed=(seed^(seed>>16))*0x21f0aaad
    mantissa=(((mixed&0xffffffff)^(mixed>>32))&0xffffffff)>>9
    return seed,sub(struct.unpack('<f',struct.pack('<I',mantissa|0x3f800000))[0],1.)


def interval(x,x0,y0,x1,y1):
    x,x0,y0,x1,y1=map(f32,(x,x0,y0,x1,y1))
    if x1<x0:x0,x1,y0,y1=x1,x0,y1,y0
    if x<=x0:return y0
    if x>=x1:return y1
    delta=sub(x1,x0)
    fraction=f32(mul(sub(x,x0),sub(y1,y0))/delta) if abs(delta)>f32(4e-19) else 0.
    return add(y0,fraction)


def overload_step(ratios,health,stress,dt,seed,stress_curve=(-3.,-10.,3.,10.),break_multiplier=6.):
    out=[];events=[];excess=[];draws=[];dt=f32(dt)
    for index,(ratio,h,old) in enumerate(zip(ratios,health,stress)):
        e=sub(mul(f32(ratio),sub(f32(1.6),mul(f32(.6),f32(h)))),1.)
        rate=interval(e,0.,0.,*stress_curve[2:]) if e>=0 else interval(e,0.,0.,*stress_curve[:2])
        value=add(mul(rate,dt),f32(old));excess.append(e)
        if value>1.:
            value=1.;probability=mul(mul(e,dt),f32(break_multiplier));seed,u=random_fraction(seed)
            draws.append(dict(wing=index,value=u,threshold=probability))
            if u<probability:events.append([11,14][index])
        out.append(max(0.,value))
    return dict(stress=out,seed=seed,break_events=events,excess=excess,draws=draws)


def position_tick_seed(position,tick):
    seed=((tick<<16)^tick)&0xffffffff
    for value in position:
        value=float(value)*10.
        n=math.trunc(value) if math.isfinite(value) else -(1<<31)
        seed^=(n if -(1<<31)<=n<(1<<31) else -(1<<31))&0xffffffff
    return seed


def wing_ias_step(ias_u,vne,health,dt,seed,speed_multiplier=1.,break_multiplier=.03):
    vne=mul(f32(vne),f32(speed_multiplier));ias_u=f32(ias_u);dt=f32(dt)
    events=[];draws=[]
    for index,h in enumerate(health):
        excess=sub(ias_u,mul(sub(f32(1.6),mul(f32(.6),f32(h))),vne))
        if excess>0.:
            threshold=mul(mul(excess,dt),f32(break_multiplier));seed,u=random_fraction(seed)
            draws.append(dict(part=[11,14][index],value=u,threshold=threshold))
            if u<threshold:events.append([11,14][index])
    return dict(seed=seed,break_events=events,draws=draws)


def mach_step(mach,mne,dt,seed,additional_mne=(),control_speed_buffer=10.):
    mach,mne,dt=map(f32,(mach,mne,dt));events=[];parts=[];draws=[]
    wing_excess=sub(mach,mne)
    control_excess=add(mul(f32(control_speed_buffer),f32(-.0030303029343485832)),
                       sub(mach,max([mne]+list(map(f32,additional_mne)))))
    for kind,ids,excess in [('wing',[11,14],wing_excess),('part',[17,18,19],control_excess)]:
        if excess<=0.:continue
        threshold=mul(mul(10.,dt),excess)
        for part in ids:
            seed,u=random_fraction(seed);draws.append(dict(kind=kind,part=part,value=u,threshold=threshold))
            if u<threshold:(events if kind=='wing' else parts).append(part)
    return dict(seed=seed,break_events=events,part_events=parts,draws=draws)


def control_ias_step(ias_excess,dt,seed,buffer=10.,break_multiplier=.05):
    excess=sub(f32(ias_excess),f32(buffer));events=[];draws=[]
    if excess>0.:
        threshold=mul(mul(excess,f32(dt)),f32(break_multiplier))
        for part in range(13,18):
            seed,u=random_fraction(seed);draws.append(dict(part=part,value=u,threshold=threshold))
            if u<threshold:events.append(part)
    return dict(seed=seed,part_events=events,draws=draws)


def tail_ias_step(ias_u,maximum_vne,dt,seed,buffer=40.,break_multiplier=.05):
    excess=sub(sub(f32(ias_u),f32(maximum_vne)),f32(buffer));events=[];draws=[]
    if excess>0.:
        threshold=mul(mul(excess,f32(dt)),f32(break_multiplier));seed,u=random_fraction(seed)
        draws.append(dict(value=u,threshold=threshold))
        if u<threshold:events=[4,5,6,3]
    return dict(seed=seed,part_events=events,draws=draws)


def flutter_commands(commands,ias_excess,tick):
    excess=f32(ias_excess)
    if excess<=0.:return list(map(f32,commands))
    lo=mul(excess,f32(-.03));width=mul(excess,f32(.06));seed=tick&0xffffffff;out=[]
    for command in commands:
        seed,u=random_fraction(seed)
        jitter=max(f32(-.03),min(f32(.03),add(mul(u,width),lo)))
        out.append(max(-1.,min(1.,add(f32(command),jitter))))
    return out
