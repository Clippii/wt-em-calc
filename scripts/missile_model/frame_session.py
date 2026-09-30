"""No-helper catch-up scheduler and pending expiry around a FlightSession.

This represents supplied ownerless entity visits with dry flight, no prediction helper,
collision/fuze/secondary objects, or render postprocessor side effects. Removal
effects are not executed: a subsequent visit reports the pending dispatch.
"""
from copy import deepcopy
import math
from kernels import f32,add,sub
from flight_session import FlightSession,finite
from flight_limits import distance_setting,evaluate
from tracking_timeout import advance as advance_tracking_timeout,threshold

FORMAT='wt-flight-frame-session-v2'
STEP=f32(1/48)


class FrameSession:
    def __init__(self,document):
        if document.get('format')!=FORMAT:raise ValueError('Explicit frame-session checkpoint required')
        self.flight=FlightSession(document['flight'])
        self.limits=deepcopy(document['limits']);finite(self.limits,'limits')
        self.iteration_limit=document['iteration_limit']
        if type(self.iteration_limit) is not int or self.iteration_limit<1:
            raise ValueError('Positive integer native iteration limit required')
        self.expiry_flag=bool(document['expiry_flag'])
        self.lost_tracking_clock=f32(document['lost_tracking_clock'])
        finite(self.lost_tracking_clock,'lost_tracking_clock')

    @classmethod
    def from_release(cls,document,*,record,constructed_guidance,controls,launch_seed,global_seed,
                     owner_distance_scale=1.,requested_distance=None,iteration_limit=15,**options):
        finite(owner_distance_scale,'owner_distance_scale');finite(requested_distance,'requested_distance')
        if 'distance_limit' in options:raise ValueError('Use owner_distance_scale/requested_distance for frame launch')
        rocket=document['properties']['rocket'];limit=distance_setting(rocket,owner_distance_scale,requested_distance)
        flight=FlightSession.from_release(document,record,constructed_guidance,controls,launch_seed,global_seed,
            distance_limit=limit,**options)
        launch=flight.launch_metadata['body_launch']
        return cls(dict(format=FORMAT,flight=flight.checkpoint(),iteration_limit=iteration_limit,expiry_flag=False,lost_tracking_clock=0.,
            limits=dict(launch_time=launch['launch_time'],lifetime=launch['lifetime'],distance_limit=limit,origin=launch['range_origin'])))

    def checkpoint(self):
        return dict(format=FORMAT,flight=self.flight.checkpoint(),limits=deepcopy(self.limits),
            iteration_limit=self.iteration_limit,expiry_flag=self.expiry_flag,lost_tracking_clock=self.lost_tracking_clock)

    def advance(self,now,target_provider,*,illumination_provider=None,flight_step=None):
        """One supplied entity visit; provider(old_body, next_time) supplies targets.

        The native no-helper scheduler uses fixed 1/48 body intervals, caps its
        requested target at old time + 0.5, and permits iteration_limit - 1
        updates. It can overshoot the requested time. Expiry uses supplied now
        and the post-update position, not an interpolated threshold crossing.
        The engaged/untracked timer is updated after each guidance evaluation;
        its destruction callback marks pending expiry without interrupting the
        current catch-up loop. Remote-owner suppression and nonnegative
        guidance-event fractions remain outside this pure-air interface.
        """
        now=f32(now)
        if not math.isfinite(now):raise ValueError('Finite caller time required')
        if self.expiry_flag:
            return dict(status='expiry_dispatch_due',now=now,updates=[],state=deepcopy(self.flight.state),
                position=self.flight.state['body']['position'][:])
        # Commit only after all supplied inputs and independent steps succeed.
        working=FlightSession(self.flight.checkpoint());start=working.state['body']['time']
        lost_clock=self.lost_tracking_clock;destroy=False
        target=min(add(start,.5),now);updates=[];counter=0
        while working.state['body']['time']<target:
            counter+=1
            if counter>=self.iteration_limit:break
            old=deepcopy(working.state['body']);next_time=add(old['time'],STEP)
            targets=target_provider(old,next_time)
            result=working.step(targets,STEP,illumination_provider=illumination_provider,flight_step=flight_step)
            manager=working.state['guidance']['manager']
            loss=advance_tracking_timeout(lost_clock,threshold(working.properties['rocket']),STEP,
                engaged=manager['engaged'],tracking=manager['tracking'])
            lost_clock=loss['clock'];destroy=destroy or loss['destroy']
            updates.append(result)
        expiry=evaluate(**self.limits,position=working.state['body']['position'],now=now,initial_flag=self.expiry_flag or destroy)
        self.flight=working;self.expiry_flag=expiry['expiry_flag'];self.lost_tracking_clock=lost_clock
        return dict(status='expiry_pending' if self.expiry_flag else 'active',now=now,target_time=target,
            advanced_time=sub(working.state['body']['time'],start),updates=updates,expiry=expiry,state=deepcopy(working.state))
