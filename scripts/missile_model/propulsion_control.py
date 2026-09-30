"""Recovered game propulsion-controller loader and update, client 2.59.0.34.

Four stage enums share one PID record. These commands scale motor clocks in the
surrounding projectile code; this component does not itself integrate a motor.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,div,safe_div


def properties(config):
    stages=[];large=f32(2147440000.);convert=f32(.2777777910232544)
    for index in range(4):
        if f'propulsion{index}' not in config:break
        c=config[f'propulsion{index}'];pid=c.get('PidControllerCruise',{})
        base=mul(c.get('baseIndSpeed',-1.),convert)
        stages.append(dict(start=f32(c.get('startTimeMin',0.)),altitude=list(map(f32,c.get('altitudeRange',[-large,large]))),
            speed_squared=mul(c.get('velocityMin',-large),c.get('velocityMin',-large)),
            closure_min=f32(c.get('targetClosureRateMin',-large)),tgo_min=f32(c.get('timeToHitMin',-large)),
            use_ias=bool(c.get('useIasVelocity',False)),cruise=mul(c.get('cruiseVelocity',large),convert),
            cruise_error_limit=mul(c.get('cuiseVelocityDiffMax',large),convert),base_ias_squared=mul(abs(base),base),
            coefficients=[f32(pid.get(key,default)) for key,default in (('prop',.1),('intg',.01),('diff',.01),('intgLim',.5))],
            pursuit_closure=[mul(v,convert) for v in c.get('pursuitClosingVelMin',[-large,large])],
            terminal_tgo=f32(c.get('terminalTimeToHit',-large))))
    return stages


def atmosphere_terms(height,environment):
    ceiling,reference,sea=[f32(environment[k]) for k in ('height_scale','reference_density','sea_density')]
    height=f32(height);z=min(height,ceiling);poly=f32(2.28719e-19)
    for c in (-5.83556e-14,3.53118e-9,-9.59387e-5,1.):poly=add(mul(poly,z),c)
    numerator=mul(poly,mul(reference,ceiling));denominator=max(height,ceiling)
    density=div(numerator,denominator)
    ias_factor=f32(math.sqrt(div(numerator,mul(denominator,sea))))
    return density,ias_factor


def update(p,state,time,position,velocity,tgo,closure,dt,*,environment=None):
    if len(p)>4:raise ValueError('Native controller supports at most four stages')
    if len(state['stages'])!=4 or len(state['pid'])!=8:raise ValueError('Four enums and one eight-float shared PID required')
    s=deepcopy(state);factors=[1.,1.,1.,1.]
    if not p:return dict(state=s,factors=factors)
    environment=environment or dict(height_scale=18300.,reference_density=1.225,sea_density=1.225)
    time,tgo,closure,dt=map(f32,(time,tgo,closure,dt));height=f32(position[1]);vx,vy,vz=map(f32,velocity)
    speed_squared=add(mul(vz,vz),add(mul(vy,vy),mul(vx,vx)))
    for index,stage in enumerate(p):
        mode=s['stages'][index]
        if mode==0:
            start=time>stage['start'] and (height<stage['altitude'][0] or height>stage['altitude'][1]
                or speed_squared<stage['speed_squared'] or closure<stage['closure_min'] or tgo<stage['tgo_min'])
            if start:s['stages'][index]=1
            else:factors[index]=0.
        elif mode in (1,2):
            density,ias=atmosphere_terms(height,environment) if stage['use_ias'] or (mode==2 and stage['base_ias_squared']>0.) else (0.,1.)
            speed=mul(f32(math.sqrt(speed_squared)),ias if stage['use_ias'] else 1.)
            if mode==1:
                if speed>stage['cruise']:s['stages'][index]=2
                continue
            ratio=root=1.
            if stage['base_ias_squared']>0.:
                ratio=safe_div(mul(stage['base_ias_squared'],mul(environment['sea_density'],environment['sea_density'])),mul(mul(density,density),speed_squared))
                root=f32(math.sqrt(ratio))
            prop,intg,diff,limit=stage['coefficients']
            kp,ki,kd=mul(root,prop),mul(root,intg),mul(ratio,diff)
            error=min(stage['cruise_error_limit'],max(-stage['cruise_error_limit'],sub(stage['cruise'],speed)))
            previous=s['pid'];integral=min(limit,max(-limit,add(mul(mul(ki,dt),error),previous[3])))
            derivative=add(mul(add(sub(mul(previous[7],-dt),previous[6]),error),48.),previous[7])
            factors[index]=min(1.,max(0.,add(add(integral,mul(kp,error)),mul(derivative,kd))))
            s['pid']=[kp,ki,limit,integral,kd,48.,error,derivative]
            if tgo<stage['terminal_tgo']:s['stages'][index]=4
            elif closure<stage['pursuit_closure'][0]:s['stages'][index]=3
        elif mode==3:
            if tgo<stage['terminal_tgo']:s['stages'][index]=4
            elif closure>stage['pursuit_closure'][1]:s['stages'][index]=2
        # Mode 4 and out-of-range enums retain the initialized unit factor.
    return dict(state=s,factors=factors)
