import copy,math
from component_assembly import f32,add,mul
from piston_model import inlet_pressure,mixture
from propulsion_general import target_omega


def mixture_setting(p,velocity,height):
    if p['mixer_type']!=2:return f32(.5)
    inlet=inlet_pressure(height,velocity[0],p['ram_recovery'])
    candidates=[]
    for index in range(1,256):
        value=mul(float(index),f32(.005));r=mixture(p,inlet,value)
        if not r['requires_stop']:candidates.append((r['multiplier'],-abs(index-100),value))
    if not candidates:raise ValueError('No healthy delivered mixture at this inlet pressure')
    return max(candidates)[2]


def initial_state(p,velocity,height,throttle=1.1,afterburner=True,commands=None,gears=None,automatic=None,nitro=0.,engine_control_mode="optimized"):
    if engine_control_mode not in ("automatic","optimized"):raise ValueError("Unknown engine control mode")
    aec=engine_control_mode=="automatic"
    engines=[]
    for i,e in enumerate(p['engines']):
        ep=e['properties'];t=f32(0. if e['family']==3 else throttle)
        engines.append(dict(throttle=t,effective_throttle=t,running=0 if e['family']==3 else 7,afterburner=bool(afterburner),
            omega=ep['max_omega'],mechanical=1.,mixture=1. if aec else mixture_setting(ep,velocity,height),
            automatic_mixture=aec,automatic_compressor=aec,
            reservoir=ep['reservoir_capacity'],gear=gears[i] if gears is not None else 0,
            regulator=-1.,turbo=ep['turbo_min'],automatic_turbo=True))
    props=[]
    for i,prop in enumerate(p['propellers']):
        pp=prop['properties'];auto=(automatic[i] if automatic is not None else (aec or prop['automatic']))
        if auto and not aec and not prop['automatic']:raise ValueError('Automatic propeller control is unavailable')
        command=commands[i] if commands is not None else 255
        if not isinstance(command,int) or not 0<=command<=255:raise ValueError('Propeller command must be a delivered byte')
        if not auto and not prop['manual'] and command!=255:raise ValueError('Manual propeller control is unavailable')
        pitch=pp['pitch_min']
        props.append(dict(command=mul(float(command),f32(1/255)),auto=bool(auto),
                          pitch=pitch,governor_pitch=pitch,flow=[0.,0.,0.]))
    transmissions=[]
    for t in p['transmissions']:
        desired=[target_omega(p['engines'][l['index']]['properties'],engines[l['index']],nitro)*l['inverse_ratio'] for l in t['engines']]
        omega=f32(max(desired))
        transmissions.append(dict(omega=omega,previous_omega=omega))
    return dict(engines=engines,propellers=props,transmissions=transmissions,seed=12345)


def retained_key(state):
    values=[state.get('seed',12345)]
    for t in state['transmissions']:values.extend([t['omega'],t.get('previous_omega',t['omega'])])
    for p in state['propellers']:values.extend([p['pitch'],p.get('governor_pitch',p['pitch']),*p['flow']])
    for e in state['engines']:
        values.extend(e.get(k,0.) for k in ['omega','effective_throttle','torque','friction','regulator','gear',
            'turbo','turbo_command','mechanical','extra_amplitude','reservoir'])
    return tuple(values)


