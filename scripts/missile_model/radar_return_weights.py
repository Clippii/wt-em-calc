"""Radar per-target signature/gate weighting, client 2.59.0.34.

143fd33a0 after its target/frame/environment inputs have been supplied.
Target services, source-frequency factors and final visibility remain inputs.
"""
import math
from kernels import f32,add,sub,mul,div
from motor_vector import rotate_thrust
from radar_measurement_gates import gains
from radar_aggregation import inverse_lateral


def combine(p,old,new,signatures,quaternion,visibility):
    values=[0.]*12;best=0.;target_id=-1
    def result(accepted):return dict(accepted=accepted,values=values,target_id=target_id)
    for sig in signatures:
        raw=list(map(f32,sig['values']));mask=int(sig['mask'])&0xffffffff
        base=mul(mul(raw[0],p['source_factor']),max(f32(new[1]),f32(old[1])))
        strength=base
        for bit,factor in enumerate(p['band_factors']):
            if mask&(1<<bit):strength=mul(strength,factor)
        if strength<p['minimum_signal']:continue
        distance_start=add(raw[7],old[11]);distance_end=add(raw[7],new[11])
        if p['gates']['distance']['present']:
            period=p['gates']['distance']['period']
            def positive(value):return add(value,mul(f32(math.ceil(div(-value,period))),period)) if value<0. else value
            distance_start=positive(distance_start);distance_end=positive(distance_end)
        distance_width=add(p['gates']['distance']['signal_width_min'],raw[8])
        doppler_width=add(add(max(new[20],old[20]),raw[12]),p['gates']['doppler']['signal_width_min'])
        signal=dict(distance=dict(zip(('start','end','width','center','center_window','adaptive_width_cap'),
                [distance_start,distance_end,distance_width,*p['distance_control']])),
            doppler=dict(zip(('start','end','width','center','center_window','adaptive_width_cap'),
                [add(raw[11],old[19]),add(raw[11],new[19]),doppler_width,*p['doppler_control']])),
            start_angles=[old[15],old[14]],end_angles=[new[15],new[14]],
            angular_width=[mul(new[12],new[18]),mul(new[12],new[17])],angular_window=p['angular_window'],
            main_beam_speed=add(mul(new[5],p['receiver_velocity'][2]),add(mul(new[4],p['receiver_velocity'][1]),mul(new[3],p['receiver_velocity'][0]))),
            elevation_sine=new[4])
        gate=gains(p['gates'],signal,use_reference=p['use_reference'])
        if not gate['accepted']:return result(False)
        weight=mul(strength,gate['gain'])
        values[0]=add(values[0],weight);values[1]=add(values[1],mul(base,gate['reference_gain']))
        values[2]=add(values[2],mul(max(old[2],new[2]),weight))
        for i in range(3):values[3+i]=add(values[3+i],mul(raw[1+i],weight))
        angular=mul(new[12],weight)
        values[6]=add(values[6],mul(angular,raw[10]));values[7]=add(values[7],mul(raw[9],angular))
        for i,value in enumerate((gate['distance'],distance_width,gate['doppler'],doppler_width),8):values[i]=add(values[i],mul(value,weight))
        if weight>best:best=weight;target_id=sig['target_id']
    values[3:6]=rotate_thrust(quaternion,values[3:6])
    if values[0]<p['minimum_signal']:return result(False)
    if p['angular_alias']:
        y,z=inverse_lateral(p['receiver_axes'],values[3:6])
        z_period=mul(max(mul(p['antenna_half'][0],p['alias_multiplier']),p['alias_minimum']),values[0])
        y_period=mul(max(mul(p['antenna_half'][1],p['alias_multiplier']),p['alias_minimum']),values[0])
        if abs(z)>z_period or abs(y)>y_period:
            if z_period==0. or y_period==0.:raise ValueError('Degenerate native angular-alias period')
            z=f32(math.fmod(z,z_period));y=f32(math.fmod(y,y_period))
            x=f32(math.sqrt(max(sub(mul(values[0],values[0]),add(mul(z,z),mul(y,y))),0.)))
            axes=p['receiver_axes']
            values[3:6]=[add(add(mul(axes[6+i],z),mul(axes[3+i],y)),mul(axes[i],x)) for i in range(3)]
    # Native visibility services are reached only after signal/gate checks.
    if callable(visibility):visibility=visibility()
    if not visibility['clear']:
        values[:2]=[0.,0.];return result(False)
    if f32(visibility['weight'])<1.:
        for i in (0,1,3,4,5,6,7,8,9,10,11):values[i]=mul(values[i],visibility['weight'])
        if values[0]<p['minimum_signal']:return result(False)
    return result(True)
