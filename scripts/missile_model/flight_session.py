"""Standalone geometric-policy propagation and complete-state checkpoints.

No native harness imports or executable access. Propagation accepts complete
snapshots; from_release additionally combines a world record with explicitly
supplied constructed guidance histories and projectile controls.
This interface propagates the recovered pure-air core; it does not determine
collision, fuze, destruction, or the live game's outer scheduling decisions.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
from kernels import f32,add,motor_properties
from geometric_optical_flight import update as optical_update
from geometric_radar_flight import update as radar_update
from geometric_radar import configure as radar_configure
from state_binary32 import ENCODING as STATE_ENCODING,encode as encode_state,decode as decode_state,contains_special,validate_flight_numbers

FORMAT='wt-flight-session-v1'
BINARY='c48218a7bca4429d5e22b344293c967b66d3ea29afad571a890c13c133050e21'
SCOPE='Pure-air game-model propagation with geometric-only observation; complete supplied state, no aircraft providers, collision/fuze/termination or live-game validation.'


def finite(value,path='state'):
    if isinstance(value,float) and not math.isfinite(value):raise ValueError(path+' must be finite')
    if isinstance(value,dict):
        for key,item in value.items():finite(item,path+'.'+key)
    elif isinstance(value,(list,tuple)):
        for i,item in enumerate(value):finite(item,f'{path}[{i}]')


def digest(properties):
    return hashlib.sha256(json.dumps(properties,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


class FlightSession:
    """Advance from an explicit full body, seeker and controller snapshot.

    step() accepts the existing family-specific point-target records. No target
    trajectory is inferred from its initial condition. The caller supplies each
    interval's observation input; an optional radar illumination provider takes
    (old_body, new_body) and returns the explicitly documented frame packet.
    """
    def __init__(self,document):
        if 'final_checkpoint' in document:document=document['final_checkpoint']
        if document.get('format')!=FORMAT or document.get('binary_sha256')!=BINARY:
            raise ValueError('A checkpoint/profile for the pinned 2.59.0.34 model is required')
        self.asset=document['asset'];self.family=document['family']
        if self.family not in ('optical','radar'):raise ValueError('Unsupported guidance family')
        self.serialized_properties=deepcopy(document['properties'])
        if document.get('properties_sha256')!=digest(self.serialized_properties):
            raise ValueError('Profile properties do not match their recorded digest')
        if self.serialized_properties['rocket']['guidanceType']!=self.family:
            raise ValueError('Profile guidance family mismatch')
        expected_policy='geometric_'+self.family+'_v1'
        if document.get('policy')!=expected_policy:raise ValueError('Explicit geometric policy required')
        self.policy=expected_policy;self.properties=deepcopy(self.serialized_properties)
        motors=motor_properties(self.properties['rocket'])
        if [asdict(m) for m in motors]!=self.properties['guidance']['motors']:
            # JSON changes tuples to lists; compare their serialized forms.
            if json.loads(json.dumps([asdict(m) for m in motors]))!=self.properties['guidance']['motors']:
                raise ValueError('Serialized motor properties disagree with authored motor data')
        self.properties['guidance']['motors']=motors
        if self.family=='radar':self.properties=radar_configure(self.properties)
        state_encoding=document.get('state_encoding')
        if state_encoding not in (None,STATE_ENCODING):raise ValueError('Unsupported flight state encoding')
        self.state=decode_state(document['state']) if state_encoding else deepcopy(document['state'])
        validate_flight_numbers(self.state)
        if not {'body','guidance','controls'}<=self.state.keys():raise ValueError('Complete flight state required')
        self.launch_metadata=deepcopy(document.get('launch_metadata'))

    @classmethod
    def from_release(cls,document,record,constructed_guidance,controls,launch_seed,global_seed,**options):
        """Launch from supplied world record and complete constructed histories.

        The document supplies verified properties; its saved flight state is
        replaced. This does not infer prelaunch seeker state from body pose.
        The launch epoch defaults to record.time and is retained across resumes.
        """
        from release_state import initialize
        session=cls(document)
        finite(record,'release');finite(constructed_guidance,'constructed_guidance');finite(options,'launch_options')
        result=initialize(session.properties,record,constructed_guidance,controls,launch_seed,global_seed,**options)
        finite(result,'initialized_release')
        session.state=deepcopy(result['state'])
        session.launch_metadata={key:deepcopy(value) for key,value in result.items() if key!='state'}
        return session

    def checkpoint(self):
        result=dict(format=FORMAT,binary_sha256=BINARY,asset=self.asset,family=self.family,policy=self.policy,
            properties=deepcopy(self.serialized_properties),properties_sha256=digest(self.serialized_properties),
            state=deepcopy(self.state),state_origin='complete supplied or independently propagated snapshot',scope=SCOPE)
        if self.launch_metadata is not None:result['launch_metadata']=deepcopy(self.launch_metadata)
        if contains_special(self.state):
            result['state']=encode_state(self.state);result['state_encoding']=STATE_ENCODING
        return result

    def step(self,targets,dt,*,illumination_provider=None,flight_step=None):
        """Advance with the default composition or an explicit flight_step(p,state,targets,dt).

        The optional composition owns external provider services; it must return
        the same complete result/state contract. Checkpoint validation and atomic
        state commit are unchanged. Callables are not serialized into snapshots.
        """
        dt=f32(dt)
        if not math.isfinite(dt) or not f32(1e-5)<=dt<=1.:raise ValueError('Step must lie between 1e-5 and 1 second')
        if len(targets)>1:raise ValueError('Choose one point target explicitly')
        finite(targets,'targets');validate_flight_numbers(self.state)
        if add(self.state['body']['time'],dt)<=self.state['body']['time']:
            raise ValueError('Step does not advance the binary32 clock')
        if flight_step is not None:
            result=flight_step(self.properties,self.state,targets,dt)
        elif self.family=='radar':
            result=radar_update(self.properties,self.state,targets,dt,illumination_provider=illumination_provider)
        else:
            if illumination_provider is not None:raise ValueError('Optical profiles do not use radar illumination')
            result=optical_update(self.properties,self.state,targets,dt)
        validate_flight_numbers(result['state']);self.state=deepcopy(result['state'])
        return result


def linear_point(motion,time):
    if motion.get('kind')!='constant_velocity':raise ValueError('CLI motion must explicitly specify constant_velocity')
    position=motion['position'];velocity=motion['velocity']
    if len(position)!=3 or len(velocity)!=3:raise ValueError('Three position and velocity components required')
    finite(motion,'motion');delta=time-motion.get('epoch',0.)
    return dict(position=[f32(p+v*delta) for p,v in zip(position,velocity)],velocity=list(map(f32,velocity)),
        quaternion=list(map(f32,motion.get('quaternion',[0.,0.,0.,1.]))))


def run_scenario(session,scenario):
    """CLI convenience for an explicitly chosen constant-velocity target.

    Arbitrary target motion is supported by the Python step() input interface.
    Target motion and optional illumination here are scenario inputs, not
    reconstructed aircraft dynamics or aircraft-radar simulation.
    """
    steps=scenario['steps'];dt=f32(scenario['dt'])
    if type(steps) is not int or steps<=0:raise ValueError('Positive integer steps required')
    target=scenario['target'];rows=[];source=scenario.get('illumination')
    def illumination(old,new):
        body=linear_point(source,new['time'])
        basis=source['basis']
        if len(basis)!=9:raise ValueError('Illuminator basis requires nine components')
        return dict(present=True,frame=list(map(f32,basis))+body['position'],velocity=body['velocity'])
    for _ in range(steps):
        old_time=session.state['body']['time'];new_time=add(old_time,dt)
        point=linear_point(target,new_time)
        if session.family=='optical':
            # Auxiliary is kept explicit because it also sets angular gate width.
            targets=[dict(position=point['position'],auxiliary=target['auxiliary'],id=target.get('id',123))]
        else:
            targets=[dict(id=target.get('id',123),unit=1,unit_type=1,
                scene=dict(old_target=linear_point(target,old_time),new_target=point))]
        result=session.step(targets,dt,illumination_provider=illumination if source is not None else None)
        manager=result['state']['guidance']['manager']
        rows.append(dict(body=deepcopy(result['state']['body']),tracking=manager['tracking'],
            manager_mode=manager['mode'],controls=result['state']['controls'][:],target=point))
    return dict(format='wt-trajectory-v1',scope=SCOPE,asset=session.asset,policy=session.policy,
        scenario=deepcopy(scenario),trajectory=rows,final_checkpoint=session.checkpoint())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True,help='Exported reference profile, checkpoint or previous run JSON')
    parser.add_argument('--scenario',type=Path,required=True,help='Constant-velocity scenario JSON')
    parser.add_argument('--output',type=Path,required=True,help='Trajectory and complete final checkpoint JSON')
    args=parser.parse_args()
    session=FlightSession(json.loads(args.profile.read_text(encoding='utf-8')))
    result=run_scenario(session,json.loads(args.scenario.read_text(encoding='utf-8')))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(output=str(args.output.resolve()),updates=len(result['trajectory']),
        time=session.state['body']['time'],tracking=session.state['guidance']['manager']['tracking'],scope=SCOPE)))


if __name__=='__main__':main()
