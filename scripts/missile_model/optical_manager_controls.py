"""Independent primary optical manager and its three controller services.

Prescribed old/new body states, motor clocks and a supplied seeker callback.
This research composition is not a trajectory simulator or an ideal seeker.
"""
from copy import deepcopy
from kernels import motor_scalar
from optical_manager import update as manager_update
from orientation_control import update as orientation_update
from propulsion_control import update as propulsion_update
from guidance_request import request
from acceleration_control import update as acceleration_update


def update(p,state,old,new,elapsed,clocks,seeker,*,suppress=False,context_value=0.,height_query=None,
           manager_step=manager_update,environment=None,matrix_velocity_frame=True):
    s=deepcopy(state);requests=[];children={}
    def orientation(args):
        result=orientation_update(p['orientation'],s['orientation'],args['time'],args['range_squared'],args['quaternion'],
            args['omega'],args['angles'],args['context_value'],args['dt'])
        s['orientation']=result['state'];children['orientation']=result
        return {k:result[k] for k in ('override','fins','auxiliary')}
    def guidance(args):
        gs=s['guidance'];command=request(p['guidance'],gs['mode'],gs['course'],args['direction'],args['angular_rate'],
            args['navigation'],args['quaternion'],args['velocity'],args['position'],args['time'],args['time_to_hit'],args['range_squared'],height_query)
        gs.update(mode=command['mode'],course=command['course']);fins=[0.,0.]
        if command['acceleration']:
            requests.append(command['acceleration'])
            control=acceleration_update(p['acceleration'],p['aero'],motor_scalar(p['motors'],clocks[:len(p['motors'])]),
                gs['pid'],command['acceleration'],args['feedback'],args['quaternion'],args['velocity'],args['position'][1],args['time'],args['dt'],
                environment=environment,matrix_velocity_frame=matrix_velocity_frame)
            gs['pid']=control['state'];fins=args['fins'] if control['fins'] is None else control['fins']
        children['guidance']=dict(fins=fins,request=command)
        return dict(fins=fins)
    def propulsion(args):
        result=propulsion_update(p['propulsion'],s['propulsion'],args['time'],args['position'],args['velocity'],args['time_to_hit'],args['closure'],args['dt'],
            environment=environment)
        s['propulsion']=result['state'];children['propulsion']=result
        return dict(factors=result['factors'])
    result=manager_step(p['manager'],s['manager'],old,new,elapsed,
        dict(seeker=seeker,orientation=orientation,guidance=guidance,propulsion=propulsion),suppress=suppress,context_value=context_value)
    s['manager']=result['state']
    return dict(state=s,events=result['events'],requests=requests,children=children)
