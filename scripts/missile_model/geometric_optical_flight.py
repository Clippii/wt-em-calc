"""Independent optical flight with the explicit geometric_optical_v1 policy.

Complete released state and loaded properties supplied; primary no-provider
flight guidance. No termination/launch initialization is synthesized here.
"""
from optical_point_flight import update as flight_update
from optical_point_controls import update as controls_update
from geometric_optical_observation import observe


def update(p,state,targets,dt):
    def guidance(*args,**kwargs):
        return controls_update(*args,observation_step=observe,**kwargs)
    return flight_update(p,state,targets,dt,guidance_step=guidance)
