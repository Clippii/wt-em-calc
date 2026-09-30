"""Independent clear point-target optical observation, client 2.59.0.34.

Implements the global 147983b9e=1 enumeration/aggregation path with no Sun,
surface target, background attenuation, finite bounds or target clustering.
The enumerated target list, per-band signatures and auxiliary vector are inputs.
This is not yet the requested geometric-only policy or aircraft scene provider.
"""
import math
from copy import deepcopy
from kernels import f32,add,sub,mul,div,safe_div
from optical_geometry import observation_frame,point_response
from optical_tracking import pre_slew_body
from shared_seeker import normalize


def properties(config):
    half=mul(config.get('fov',config.get('angleHalfSens',5.)),f32(math.pi/360.))
    ranges=[config.get('rangeBand0',config.get('range',4000.))]
    ranges += [config.get(f'rangeBand{i}',0.) for i in range(1,10)]
    # Original two divisions, not a simplified range-squared / constant.
    gains=[safe_div(1.,div(24034.501953125,max(mul(a,a),1.))) for a in ranges]
    if config.get('bandToReject') is not None:
        raise NotImplementedError('Repeated bandToReject loader remains separate')
    return dict(gains=gains,cosine=f32(math.cos(half)),sine=f32(math.sin(half)),
        range_max=f32(config.get('rangeMax',10000.)),
        target_size_max=f32(config.get('targetSizeToFovMax',1000.)),
        out_gate_transparency=f32(config.get('outGateTransparency',0.)),
        reject_mask=int(config.get('bandMaskToReject',0)),
        reject_fraction=f32(config.get('signalRelRejectedTreshold',.001)),
        reject_delay=f32(config.get('rejectionReactionTime',0.)),
        prolongation=f32(config.get('prolongationTimeMax',1.)),
        targets_resolution=bool(config.get('targetsResolution',False)))


def gate_coordinates(frame,direction):
    """143fe1ad8 inverse angular-frame rows, preserving its expression order."""
    a,d,g,b,e,h,c,f,i=frame[:9]
    det=sub(add(add(mul(mul(e,a),i),mul(mul(h,d),c)),mul(mul(f,b),g)),
            add(add(mul(mul(b,d),i),mul(mul(f,h),a)),mul(mul(e,g),c)))
    if det==0.:raise ValueError('Singular optical observation frame')
    inverse=div(1.,det)
    rows=((sub(mul(g,f),mul(d,i)),sub(mul(i,a),mul(g,c)),sub(mul(c,d),mul(f,a))),
          (sub(mul(h,d),mul(g,e)),sub(mul(g,b),mul(h,a)),sub(mul(e,a),mul(b,d))))
    result=[]
    for row in rows:
        terms=[mul(mul(v,inverse),u) for v,u in zip(row,direction)]
        result.append(add(add(terms[0],terms[1]),terms[2]))
    return result


def point_band(p,frame,target,*,signal_threshold=.01):
    response=point_response(frame,target['position'],p['cosine'])
    if not response['admitted']:return None
    signatures=target.get('signatures',{})
    best=0.;band=0
    for i,gain in enumerate(p['gains']):
        if gain<f32(5e-5):continue
        strength=mul(mul(gain,signatures.get(1<<i,1.)),response['response'])
        if strength>=f32(signal_threshold) and strength>best:
            best=strength;band=1<<i
    if band==0:return None
    return dict(direction=response['world_los'],distance=response['distance'],strength=best,
        band=band,auxiliary=list(map(f32,target.get('auxiliary',[0.,0.,0.]))),id=target.get('id',-1))


def observe(p,state,quaternion,origin,targets,*,dt,new_time,collect=False,signal_threshold=.01):
    """Already-enumerated zero-size targets, clear LOS, common per-band auxiliary.

    state contains angles, widths, gap, distance, target_id, reject_time and
    los_check_time/los_cache_valid. Candidate bodies/signatures do not come from
    the native routine. Returned measurement is in missile body coordinates.
    RangeMax is reported as query_range and is not applied twice here.
    """
    if p['targets_resolution']:
        raise NotImplementedError('Clustered optical observations are not reconstructed here')
    out=deepcopy(state)
    frame=observation_frame(quaternion,state['angles'],origin)
    def empty():return dict(strength=0.,direction=[0.]*3,distance=0.,auxiliary=[0.]*3,rejected=0.)
    inside=empty();outside=empty();reports=[];candidates=[]
    for target in targets:
        candidate=point_band(p,frame,target,signal_threshold=signal_threshold)
        if candidate is None:continue
        candidates.append(candidate)
        if not out['los_cache_valid']:
            out.update(los_cache_valid=True,los_check_time=f32(new_time))
        y,z=gate_coordinates(frame,candidate['direction'])
        in_gate=abs(z)<f32(state['widths'][0]) and abs(y)<f32(state['widths'][1])
        strength=candidate['strength']
        weighted=mul(strength,1. if in_gate else p['out_gate_transparency'])
        for group,weight in ((inside,weighted),(outside,0. if in_gate else strength)):
            group['strength']=add(group['strength'],weight)
            group['distance']=add(group['distance'],mul(candidate['distance'],weight))
            for key in ('direction','auxiliary'):
                group[key]=[add(a,mul(b,weight)) for a,b in zip(group[key],candidate[key])]
            if candidate['band'] & p['reject_mask']:
                group['rejected']=add(group['rejected'],weight)
        if collect and len(reports)<8:
            reports.append(dict(id=candidate['id'],inside=strength if in_gate else 0.,outside=0. if in_gate else strength))
    group=inside if inside['strength']>1. else outside
    total=group['strength']
    strength=max(outside['strength'],inside['strength'])
    rejecting=group['rejected']>mul(p['reject_fraction'],total)
    out['reject_time']=add(state['reject_time'],dt) if rejecting else 0.
    size_limit=mul(mul(add(p['target_size_max'],p['target_size_max']),group['distance']),p['sine'])
    accepted=total>1. and (not rejecting or out['reject_time']<=p['reject_delay'])
    accepted=accepted and group['auxiliary'][2]<size_limit and group['auxiliary'][1]<size_limit
    measurement=[1.,0.,0.];auxiliary=[0.,0.,0.];reported=[]
    if accepted:
        inverse=safe_div(1.,total)
        rotated=pre_slew_body(list(map(f32,quaternion)),group['direction'])
        measurement=normalize([mul(v,inverse) for v in rotated],'predict')
        auxiliary=[mul(v,inverse) for v in group['auxiliary']]
        out.update(distance=mul(group['distance'],inverse),target_id=-1)
        chosen='inside' if inside['strength']>1. else 'outside'
        reported=[dict(id=row['id'],weight=mul(row[chosen],inverse)) for row in reports]
    elif f32(state['gap'])>p['prolongation']:
        out.update(distance=-1.,target_id=-1)
    return dict(accepted=bool(accepted),measurement=measurement,strength=strength,auxiliary=auxiliary,flag=False,
        state=out,reported_targets=reported,reported_target_count=len(reported),frame=frame,
        candidates=candidates,inside=inside,outside=outside,query_range=p['range_max'])
