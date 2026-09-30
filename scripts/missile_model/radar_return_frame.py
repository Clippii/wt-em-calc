"""Independent radar return-frame kernel 143fd0da0, point targets.

Client 2.59.0.34. Explicit transmitter/receiver frames, target state and loaded
properties. Finite target geometry and optional Doppler-width mode are separate.
"""
import math
from kernels import f32,add,sub,mul,safe_div
from sensor_point_response import point_response


def response(p,s,*,initial=None,doppler_width=False):
    if doppler_width:raise NotImplementedError('Optional Doppler-width global is outside the pinned point specialization')
    if any(s['offset']) or any(s['extent']):raise NotImplementedError('Finite target/centroid geometry remains separate')
    out=list(map(f32,initial)) if initial is not None else [0.]*24
    target=list(map(f32,s['target']));effective=target.copy()
    if s['multipath'] and p['signature_type'] in (1,2):
        delta=max(sub(target[1],s['surface_height']),0.)
        a,va,b,vb=p['multipath_curve']
        if delta<=b:
            if b<a:a,va,b,vb=b,vb,a,va
            coefficient=va if delta<=a else vb if delta>=b else add(va,safe_div(mul(sub(delta,a),sub(vb,va)),sub(b,a)))
            effective[1]=sub(target[1],mul(delta,add(coefficient,coefficient)))
    def sensor(frame,lobes,signature):
        packet=[*frame,*lobes,0.,f32(1.333),*effective,*s['quaternion'],target[1],*s['offset'],*s['extent'],
            signature,s['minimum_signal'],0.,*p['range_parameters']]
        return point_response(packet,side_lobes=s['side_lobes'],range_scaling=p['range_scaling'])
    tx=s['transmitter'];rx=s['receiver']
    baseline=[sub(a,b) for a,b in zip(rx[9:12],tx[9:12])]
    baseline_squared=add(add(mul(baseline[2],baseline[2]),mul(baseline[1],baseline[1])),mul(baseline[0],baseline[0]))
    bistatic=p['active'] and (baseline_squared>1. or p['force_bistatic'])
    if p['active']:
        signature=mul(mul(mul(p['tx_scale'],s['availability']),p['tx_strength']),p['rx_gain'])
        first=sensor(tx,p['tx_lobes'],signature)
        if not first['accepted']:return dict(accepted=False,output=out)
        t=first['output']
        if bistatic:
            out[21:24]=target
            second=sensor(rx,p['rx_lobes'],mul(signature,t[11]))
            if not second['accepted']:return dict(accepted=False,output=out)
            v=second['output']
        else:
            if mul(mul(t[11],t[11]),signature)<f32(s['minimum_signal']):return dict(accepted=False,output=out)
            v=t
    else:
        second=sensor(rx,p['rx_lobes'],mul(p['rx_gain'],s['availability']))
        if not second['accepted']:return dict(accepted=False,output=out)
        v=second['output'];t=v
    out[21:24]=target
    out[0]=(mul(mul(p['tx_strength'],p['tx_scale']),t[11]) if bistatic else mul(mul(p['tx_scale'],t[11]),p['tx_strength'])) if p['active'] else 0.
    out[1]=mul(v[11],p['rx_gain'])
    out[2]=mul(v[12],t[12]) if p['active'] else v[12]
    out[3:6]=v[:3];out[6:9]=t[:3]
    out[9:12]=[v[3],t[3],mul(add(sub(v[3],f32(math.sqrt(baseline_squared))),t[3]),.5) if bistatic else v[3]]
    out[12]=v[4]
    out[13:16]=[mul(add(a,b),.5) for a,b in zip(v[5:8],t[5:8])] if bistatic else v[5:8]
    out[16:19]=[0.,0.,0.]
    velocity=list(map(f32,s['velocity']))
    rv=[mul(sub(a,b),c) for a,b,c in zip(s['receiver_velocity'],velocity,v[:3])]
    if bistatic:
        tv=[mul(sub(a,b),c) for a,b,c in zip(s['transmitter_velocity'],velocity,t[:3])]
        bv=[mul(sub(a,b),c) for a,b,c in zip(s['transmitter_velocity'],s['receiver_velocity'],baseline)]
        base=mul(add(add(bv[2],bv[1]),bv[0]),safe_div(-1.,f32(math.sqrt(baseline_squared))))
        out[19]=mul(add(add(add(rv[0],add(tv[2],add(tv[1],tv[0]))),base),add(rv[2],rv[1])),.5)
    else:out[19]=add(rv[2],add(rv[1],rv[0]))
    out[20]=0.
    return dict(accepted=True,output=out)
