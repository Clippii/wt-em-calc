"""Independent point-target composition of 143fd33a0, client 2.59.0.34.

Supplied scene/target-history, signature and visibility services remain explicit.
No finite-target geometry, optional Doppler-width mode or ideal-loss policy is
implied. This composes recovered components, not a complete flight simulator.
"""
from kernels import f32,add,sub,mul
from radar_return_frame import response
from radar_return_weights import combine


def frequency_factor(signature_type,index,cross_sections):
    """143fd5340 radar-family scalar and copied cross-section multiplier."""
    if signature_type not in (1,2):raise NotImplementedError('Conventional radar signatures only')
    return f32(cross_sections[index]) if 0<=index<=4 else 1.


def signature_frame(quaternion,receiver_los,transmitter):
    """143fd3cb8..143fd4092 target-local incoming direction and source axes."""
    x,y,z,w=map(f32,quaternion)
    if not any((x,y,z,w)):
        matrix=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
    else:
        ww=mul(w,w);nw=-w
        diagonal=lambda v:sub(mul(add(mul(v,v),ww),2.),1.)
        xy=mul(x,y);znw=mul(z,nw);xz=mul(x,z);ynw=mul(y,nw);yz=mul(y,z);xnw=mul(nw,x)
        matrix=[diagonal(x),mul(sub(xy,znw),2.),mul(add(xz,ynw),2.),
            mul(add(znw,xy),2.),diagonal(y),mul(sub(yz,xnw),2.),
            mul(sub(xz,ynw),2.),mul(add(xnw,yz),2.),diagonal(z)]
    result=[]
    for v in (receiver_los,transmitter[3:6],transmitter[6:9]):
        for i in (0,3,6):
            result.append(sub(mul(matrix[i+1],-f32(v[1])),add(mul(matrix[i],v[0]),mul(matrix[i+2],v[2]))))
    return result+[0.,0.,0.]


def evaluate(frame_properties,weight_properties,scene,signature_provider):
    """Evaluate one point unit using explicit services and previous/current state.

    scene contains old/new transmitter and receiver {frame,velocity}, old/new
    target {position,quaternion,velocity}, masks, t0/t1, availability, water
    {present,height}, surface_height, side_lobes, multipath, sensitivity,
    frequency_index, cross_sections and final visibility. signature_provider
    receives the actual native argument values represented as a dictionary.
    """
    failure=dict(accepted=False,values=[0.]*12,target_id=-1)
    if not scene['target_present'] or f32(scene['availability'])<f32(1e-6):return failure
    if scene['water']['present'] and f32(scene['water']['height'])>f32(scene['new_target']['position'][1]):return failure
    frames=[]
    for when in ('old','new'):
        tx=scene[when+'_transmitter'];rx=scene[when+'_receiver'];target=scene[when+'_target']
        signal=dict(transmitter=tx['frame'],transmitter_velocity=tx['velocity'],
            receiver=rx['frame'],receiver_velocity=rx['velocity'],target=target['position'],
            quaternion=target['quaternion'],velocity=target['velocity'],offset=[0.]*3,extent=[0.]*3,
            surface_height=scene['surface_height'],side_lobes=scene['side_lobes'],multipath=scene['multipath'],
            availability=scene['availability'],minimum_signal=mul(scene['sensitivity'],weight_properties['minimum_signal']))
        result=response(frame_properties,signal)
        if not result['accepted']:return failure
        frames.append(result['output'])
    old,new=frames;q=scene['new_target']['quaternion']
    local=signature_frame(q,new[3:6],scene['new_transmitter']['frame'])
    properties=dict(weight_properties,source_factor=frequency_factor(frame_properties['signature_type'],scene['frequency_index'],scene['cross_sections']))
    def signatures():
        for mask in scene['masks']:
            args=dict(signature_type=frame_properties['signature_type'],mask=mask,
                transmitter_gain=max(new[0],old[0]),frame=local,extent=new[16:19],range=new[11],
                t0=f32(scene['t0']),t1=f32(scene['t1']))
            yield dict(signature_provider(args),mask=mask)
    return combine(properties,old,new,signatures(),q,scene['visibility'])
