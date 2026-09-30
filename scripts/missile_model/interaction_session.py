"""Standalone frame propagation with recovered single-point proximity events.

The supplied point history follows the unadjusted, ownerless aircraft-candidate
path. Physical meshes/contact response, scene collection and effect dispatch
are outside this interface. A reported interaction stops future advancement.
"""
from copy import deepcopy
from dataclasses import dataclass
import math
from frame_session import FrameSession
from flight_session import finite
from kernels import f32,add
from proximity_request import properties,advance_clock,build,prepare
from proximity_motion import sample_segment
from proximity_geometry import point_candidate
from trace_events import limit_pending,consume_no_contacts

FORMAT='wt-point-interaction-session-v1'
SCOPE=('Geometric-policy pure-air flight, native frame scheduling and expiry, '
       'single supplied moving-point proximity geometry with no owner/radius adjustment, '
       'and no-record event consumption. No physical mesh/contact collection, damage/removal '
       'effects, live scene scheduling or live-game validation.')


@dataclass
class PointHistory:
    """Callbacks position(time)->xyz and attitude(time)->xyzw; time is absolute."""
    position: object
    attitude: object


class InteractionSession:
    def __init__(self,document):
        if document.get('format')!=FORMAT:raise ValueError('Explicit interaction checkpoint required')
        self.frame=FrameSession(document['frame'])
        self.proximity=deepcopy(document['proximity']);finite(self.proximity,'proximity')
        self.mode=document['mode'];self.delay=f32(document['delay'])
        self.event=deepcopy(document['event']);finite(self.event,'event')
        self.offsets=list(map(f32,document['time_offsets']))
        if self.mode not in (1,2,4) or (self.mode==4)!=(self.event is not None):
            raise ValueError('Active mode 1/2 or mode 4 with an event required')
        if not math.isfinite(self.delay) or len(self.offsets)!=2 or not all(map(math.isfinite,self.offsets)):
            raise ValueError('Finite delay and two timestamp offsets required')
        if len(self.proximity['origin'])!=3:raise ValueError('Three origin coordinates required')
        self.props=properties(self.frame.flight.serialized_properties['rocket'])

    @classmethod
    def from_frame(cls,frame,*,clock,origin,enabled,guidance_flag,mode=1,delay=0.,time_offsets=(-.05,0.)):
        """Proximity history/runtime flags are explicit; no lock or launch inference."""
        return cls(dict(format=FORMAT,frame=frame.checkpoint(),proximity=dict(clock=f32(clock),
            origin=list(map(f32,origin)),enabled=bool(enabled),guidance_flag=bool(guidance_flag)),
            mode=mode,delay=f32(delay),event=None,time_offsets=list(map(f32,time_offsets))))

    def checkpoint(self):
        return dict(format=FORMAT,frame=self.frame.checkpoint(),proximity=deepcopy(self.proximity),
                    mode=self.mode,delay=self.delay,event=deepcopy(self.event),time_offsets=self.offsets[:],scope=SCOPE)

    def advance(self,now,frame_dt,target_provider,*,collision_time,point_history=None,
                illumination_provider=None,flight_step=None,surface=lambda endpoint:0.):
        """Advance one visit and consume one optional moving-point candidate.

        target_provider supplies guidance observations; point_history supplies
        collision history. Both must describe the intended target. collision_time
        is the explicit scene/consumer base clock, separate from body time.
        State commits only if propagation and every scene callback succeed.
        """
        if self.event is not None:
            return dict(status='event_dispatch_due',event=deepcopy(self.event),updates=[],scope=SCOPE)
        if self.frame.expiry_flag:
            return dict(status='expiry_dispatch_due',event=None,updates=[],scope=SCOPE)
        now,frame_dt,collision_time=map(f32,(now,frame_dt,collision_time))
        if not all(map(math.isfinite,(now,frame_dt,collision_time))) or frame_dt<0.:
            raise ValueError('Finite caller clocks and nonnegative frame_dt required')
        work=FrameSession(self.frame.checkpoint());old=deepcopy(work.flight.state['body'])
        result=work.advance(now,target_provider,illumination_provider=illumination_provider,flight_step=flight_step)
        rocket=work.flight.serialized_properties['rocket'];has_fuse=rocket.get('hasProximityFuse',False)
        clock=advance_clock(self.proximity['clock'],frame_dt,has_fuse,not result['expiry']['returned'])
        body=result['state']['body'];manager=result['state']['guidance']['manager']
        descriptor=build(start=old['position'],end=body['position'],origin=self.proximity['origin'],elapsed=clock,
            enabled=self.proximity['enabled'],has_fuse=has_fuse,detect_shells=self.props['detectShells'],mode=False,
            prior_body_time=old['time'],body_time=body['time'],guidance_distance=manager['distance'],
            guidance_flag=self.proximity['guidance_flag'])
        descriptor=limit_pending(descriptor,mode=self.mode,delay=self.delay,velocity=body['velocity'])
        preparation=motion=None;candidate=-1.;consumed=None
        if descriptor['queued']:
            preparation=prepare(descriptor,self.props,surface)
            descriptor=preparation['descriptor']
            if preparation['air_enabled'] and point_history is not None:
                end=preparation['endpoint'];lower,upper=[add(collision_time,x) for x in self.offsets]
                motion=sample_segment(descriptor['start'],end,
                    (descriptor['prior_body_time'],descriptor['body_time']),lower,upper,
                    point_history.position,point_history.attitude)
                finite(motion,'point_history')
                candidate=point_candidate(point=motion['matrix'][9:],start=descriptor['start'],end=motion['end'],
                    origin=descriptor['origin'],elapsed=clock,timeout=self.props['timeOut'],
                    arm_distance=self.props['armDistance'],radius=self.props['radius'],scale=descriptor['length'])
                descriptor['candidate_results'][1]=candidate
            consumed=consume_no_contacts(descriptor,mode=self.mode,delay=self.delay,velocity=body['velocity'],
                                          base_time=collision_time,offsets=self.offsets)
        self.frame=work;self.proximity['clock']=clock
        if consumed is not None:self.mode=consumed['mode'];self.event=consumed['event']
        return dict(status='event_pending' if self.event is not None else result['status'],
            frame=result,descriptor=descriptor,preparation=preparation,motion=motion,candidate=candidate,
            consumer=consumed,event=deepcopy(self.event),scope=SCOPE)
