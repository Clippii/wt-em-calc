"""Independent radar range/Doppler and angular filter helpers, client 2.59.0.34.

Ports 143fecea0 (measurement) and 143fed490 (coast). Observations and limit-age
are supplied. This is not a radar observation generator or complete mode update.
State rates retain native signs; range_rate is not assumed to equal closure.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub,mul,safe_div
from shared_seeker import properties as shared_properties,update as shared_update

UNBOUNDED=f32(2147440000.)


def properties(config):
    if 'mode' in config:
        raise NotImplementedError('Repeated radar mode loading remains separate')
    def axis(name,gate,limit):
        c=config.get(name,{})
        present=bool(c.get('presents',False))
        low=f32(c['minValue']) if present else 0.
        high=f32(c['maxValue']) if present else 0.
        width=f32(c.get('width',low)) if present else 0.
        g=config.get(gate,{})
        return dict(present=present,minimum=low,maximum=high,width=width,
            alpha=f32(g.get('filterAlpha',1.)),beta=f32(g.get('filterBetta',1.)),
            limit=list(map(f32,g.get(limit,[UNBOUNDED,UNBOUNDED]))),
            limit_timeout=f32(g.get('limitTimeOut',UNBOUNDED)))
    return dict(shared=shared_properties(config),distance=axis('distance','distGate','accelLimit'),
                doppler=axis('dopplerSpeed','dopplerSpeedGate','rateLimit'))


def coefficients(p,interval):
    n=min(mul(48.,interval),10.)
    if n<=0.:return 1.,1.
    return sub(1.,f32(math.pow(sub(1.,p['alpha']),n))),sub(1.,f32(math.pow(sub(1.,p['beta']),mul(n,n))))


def clamp_channel(value,p):
    half=mul(p['width'],.5)
    return min(sub(p['maximum'],half),max(add(half,p['minimum']),value))


def rounded_state(state):
    out=deepcopy(state)
    for key in ('angles','direction','angular_rate'):out[key]=list(map(f32,out[key]))
    for key in ('gap','range','range_rate','closure','closure_rate'):out[key]=f32(out[key])
    return out


def update(p,state,quaternion,measurement,range_measurement,closure_measurement,dt,limit_age,
           *,authored_rate=True,include_gap_after_reject=True,initial_angles=(12.,-13.),
           initial_rate=(14.,-15.,16.)):
    """Run the measurement helper. Failure may still change angular state.

    rejected is native +24. Gap (+20) is not cleared by this helper. Numeric
    channel writes commit only after both channel gates and angular acceptance.
    Outputs remain untouched on failure, including angular-gate rejection.
    """
    dt=f32(dt);limit_age=f32(limit_age)
    if not math.isfinite(dt) or dt<=0.:raise ValueError('Positive finite dt required')
    out=rounded_state(state)
    outputs=dict(angles=list(map(f32,initial_angles)),angular_rate=list(map(f32,initial_rate)))
    result=dict(state=out,outputs=outputs,accepted=False,rejected_by=None)
    gap=f32(state['gap'])
    rewind=not state['rejected'] or include_gap_after_reject
    interval=add(gap if rewind else 0.,dt)
    values={}
    for name,value_key,rate_key,observed in (
        ('distance','range','range_rate',range_measurement),
        ('doppler','closure','closure_rate',closure_measurement)):
        axis=p[name];value=f32(state[value_key]);rate=f32(state[rate_key])
        if not axis['present']:continue
        if rewind:value=sub(value,mul(rate,gap))
        alpha,beta=coefficients(axis,interval)
        observed=min(axis['maximum'],max(f32(observed),axis['minimum']))
        predicted=add(value,mul(rate,interval));residual=sub(observed,predicted)
        change=mul(mul(beta,residual),safe_div(1.,interval))
        next_rate=add(change,rate)
        if limit_age>=axis['limit_timeout']:
            limit=add(mul(sub(axis['limit'][1],axis['limit'][0]),min(1.,gap)),axis['limit'][0])
            if name=='distance':
                rejected=abs(change)>=mul(limit,interval) or abs(next_rate)>=UNBOUNDED
            else:
                rejected=abs(change)>=mul(UNBOUNDED,interval) or abs(next_rate)>=limit
            if rejected:
                out['rejected']=True;result['rejected_by']=name
                return result
        values[value_key]=clamp_channel(add(mul(alpha,residual),predicted),axis)
        values[rate_key]=next_rate
    shared=shared_update(p['shared'],quaternion,measurement,state,dt,authored_rate=authored_rate)
    for key in ('angles','direction','angular_rate'):out[key]=shared[key]
    if not shared['accepted']:
        out['rejected']=True;result['rejected_by']='angular'
        return result
    out.update(values,rejected=False)
    if p['distance']['present']:out['range_valid']=True
    if p['doppler']['present']:out['closure_valid']=True
    outputs.update(angles=out['angles'][:],angular_rate=out['angular_rate'][:])
    result['accepted']=True
    return result


def coast(p,state,quaternion,dt,*,authored_rate=True):
    """Extrapolate both numeric channels, even when disabled or invalid."""
    out=rounded_state(state);dt=f32(dt)
    for name,key,rate,valid in (('distance','range','range_rate','range_valid'),
                               ('doppler','closure','closure_rate','closure_valid')):
        value=add(mul(dt,state[rate]),state[key])
        if p[name]['present'] and state[valid]:value=clamp_channel(value,p[name])
        out[key]=value
    shared=shared_update(p['shared'],quaternion,None,state,dt,authored_rate=authored_rate,coast=True)
    for key in ('angles','direction','angular_rate'):out[key]=shared[key]
    return dict(state=out,outputs=dict(angles=out['angles'][:],angular_rate=out['angular_rate'][:]))
