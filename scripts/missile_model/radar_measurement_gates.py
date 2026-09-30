"""Independent radar measurement-channel gain, aces.exe 2.59.0.34.

143fd2300 and 143fd2db0: signed ambiguity, swept signal support, adaptive gate center/width,
excluded intervals, leakage and reference-width contribution. This is not the
tracking filter, scene admission, or the requested ideal tracking policy.
"""
import math
from kernels import f32,add,sub,mul,div,safe_div


def channel_properties(config):
    if not config.get('presents',False):
        return dict(present=False,minimum=0.,period=1.,maximum=0.,signal_width_min=0.,
            width=0.,reference_width=0.,reference_multiplier=1.,leak=0.)
    low=f32(config['minValue']);high=f32(config['maxValue']);width=f32(config.get('width',low))
    ref=f32(config.get('refWidth',width))
    return dict(present=True,minimum=low,period=f32(config.get('maxUnambiguousValue',add(high,high))),
        maximum=high,signal_width_min=f32(config.get('signalWidthMin',mul(width,.1))),width=width,
        reference_width=ref,reference_multiplier=safe_div(width,sub(ref,width)),leak=f32(config.get('leakMult',0.)))


def properties(config):
    angle=config.get('angles',{});present=bool(angle.get('presents',False))
    resolution=mul(angle.get('resolution',1e-6),f32(math.pi/180.)) if present else f32(1e-6)
    if resolution>f32(4e-19):inverse=div(1.,resolution)
    elif resolution<f32(-4e-19):inverse=div(f32(180./math.pi),angle.get('resolution',1e-6))
    else:inverse=0.
    return dict(distance=channel_properties(config.get('distance',{})),doppler=channel_properties(config.get('dopplerSpeed',{})),
        absolute_doppler=bool(config.get('absDopplerSpeed',False)),beam_relative_doppler=bool(config.get('mainBeamDopplerSpeed',False)),
        zero_notch_half=mul(config.get('zeroDopplerNotchWidth',-2.),.5),
        beam_notch_half=mul(config.get('mainBeamNotchWidth',-2.),.5),
        beam_notch_elevation=f32(math.sin(mul(config.get('mainBeamNotchMaxElevation',90.),f32(math.pi/180.)))),
        doppler_to_distance=f32(config.get('dopplerSpeedToDistMult',0.)),distance_to_doppler=f32(config.get('distToDopplerSpeedMult',0.)),
        angular=present,resolution=resolution,inverse_resolution=inverse)


def channel_gain(p,signal,*,exclusions=(),use_reference=False):
    """Signal fields: start/end, width, center, center_window, adaptive_width_cap.

    The caller supplies signal widths (including its minimum-width adjustment),
    exclusions in native iteration order and the reference-width global flag.
    Preserve signed widths: native ground quadrature can produce a tiny negative
    width through rounding. A positive finite ambiguity period is required.
    Returned fields are gain, reference_gain, value, accepted. Both numeric
    gains may be zero while accepted is true; acceptance is not detection.
    """
    start,end,width,center,window,cap=[f32(signal[k]) for k in
        ('start','end','width','center','center_window','adaptive_width_cap')]
    result=dict(gain=0.,reference_gain=0.,value=end,accepted=True)
    if not p['present']:return dict(result,gain=1.)
    period=p['period']
    if period<=0.:raise ValueError('Channel gain requires a positive ambiguity period')
    start=f32(math.fmod(start,period));wrapped=f32(math.fmod(end,period))
    lower=min(start,wrapped);upper=max(start,wrapped)
    inverse=safe_div(1.,width);cycles=f32(math.ceil(div(width,period)))
    gate=p['width'];low=p['minimum'];high=p['maximum'];leak=p['leak']
    if width<period:
        lower=sub(lower,mul(width,.5));upper=add(upper,mul(width,.5))
        if gate<=0.:lower=max(lower,low);upper=min(upper,high)
    else:lower=low;upper=high
    if gate>0.:
        value=wrapped
        if width>=period:value=add(f32(math.fmod(sub(wrapped,low),sub(high,low))),low)
        result['value']=value
        gate=max(gate,min(cap,width))
        if window>0.:
            lo=sub(center,mul(window,.5));hi=add(center,mul(window,.5))
            if lower<hi and upper>lo:center=min(hi,max(lo,mul(add(start,wrapped),.5)))
        gate_low=min(high,max(low,sub(center,mul(gate,.5))))
        gate_high=min(high,max(low,add(center,mul(gate,.5))))
    else:gate_low=low;gate_high=high
    if exclusions:
        total=0.;previous=low
        for a,b in [*exclusions,(high,high)]:
            lo=max(max(f32(previous),gate_low),lower)
            hi=min(min(f32(a),gate_high),upper)
            overlap=min(width,max(0.,sub(hi,lo)))
            total=add(add(total,overlap),mul(sub(width,overlap),leak))
            previous=f32(b)
    else:
        overlap=min(width,max(0.,sub(min(gate_high,upper),max(gate_low,lower))))
        total=add(mul(sub(width,overlap),leak),overlap)
    scale=mul(inverse,cycles);result['gain']=mul(total,scale)
    if gate>0. and f32(.001)<p['reference_multiplier']<f32(.999) and use_reference:
        half=mul(p['reference_width'],.5)
        lo=max(lower,min(high,max(low,sub(center,half))))
        hi=min(min(upper,high),max(low,add(center,half)))
        overlap=sub(hi,lo)
        reference=add(mul(sub(width,overlap),leak),overlap)
        if reference<0.:result['accepted']=False;return result
        result['reference_gain']=mul(mul(max(0.,sub(min(reference,width),total)),scale),p['reference_multiplier'])
    return result


def gains(p,signal,*,use_reference=False,initial=(11.,12.,13.,14.)):
    """143fd2db0 combined range/Doppler/angular gate packet.

    Signal supplies distance/doppler channel dicts, start_angles/end_angles,
    angular_width/window, main_beam_speed and elevation_sine. The native notch
    overlap operation is intentionally preserved (intersection, not union).
    Failure preserves the caller's four output slots.
    """
    d={k:f32(v) for k,v in signal['distance'].items()};v={k:f32(v) for k,v in signal['doppler'].items()}
    range_signal=dict(d,start=add(d['start'],mul(p['doppler_to_distance'],v['start'])),
        end=add(d['end'],mul(p['doppler_to_distance'],v['end'])))
    distance=channel_gain(p['distance'],range_signal,use_reference=use_reference)
    beam=f32(signal['main_beam_speed'])
    if p['beam_relative_doppler']:
        for key in ('start','end','center'):v[key]=sub(v[key],beam)
    sign=1. if v['end']>0. else -1. if v['end']<0. else 0.
    if p['absolute_doppler']:
        for key in ('start','end','center'):v[key]=abs(v[key])
    holes=[]
    if p['zero_notch_half']>0.:holes.append([-p['zero_notch_half'],p['zero_notch_half']])
    if p['beam_notch_half']>0. and f32(signal['elevation_sine'])<p['beam_notch_elevation']:
        center=0. if p['beam_relative_doppler'] else beam
        lower=sub(center,p['beam_notch_half']);upper=add(center,p['beam_notch_half'])
        if holes:
            a=max(holes[0][0],lower);b=min(holes[0][1],upper)
            if b>a:holes=[];lower=a;upper=b
        holes.append([lower,upper])
    v.update(start=add(mul(d['start'],p['distance_to_doppler']),v['start']),
        end=add(mul(d['end'],p['distance_to_doppler']),v['end']))
    doppler=channel_gain(p['doppler'],v,exclusions=holes,use_reference=use_reference)
    value=doppler['value']
    if p['absolute_doppler']:value=mul(sign,value)
    if p['beam_relative_doppler']:value=add(value,beam)
    result=dict(zip(('gain','reference_gain','distance','doppler'),map(f32,initial)),accepted=False)
    angular_gain=1.
    if p['angular']:
        overlaps=[]
        for i in range(2):
            half=mul(signal['angular_width'][i],.5);lo=mul(signal['angular_window'][i],-.5);hi=mul(signal['angular_window'][i],.5)
            spans=[sub(min(add(t[i],half),hi),max(sub(t[i],half),lo)) for t in (signal['start_angles'],signal['end_angles'])]
            overlaps.append(max(spans))
        if min(overlaps)<0.:return result
        counts=[f32(math.floor(mul(span,p['inverse_resolution']))) for span in overlaps]
        area=mul(mul(mul(p['resolution'],p['resolution']),counts[1]),counts[0])
        angular_gain=safe_div(area,mul(signal['angular_width'][1],signal['angular_width'][0]))
    if not distance['accepted'] or not doppler['accepted']:return result
    refs=[distance['reference_gain'],doppler['reference_gain']]
    reference=mul(*refs) if min(refs)>f32(4e-19) else max(refs)
    return dict(gain=mul(mul(angular_gain,distance['gain']),doppler['gain']),reference_gain=reference,
        distance=distance['value'],doppler=value,accepted=True)
