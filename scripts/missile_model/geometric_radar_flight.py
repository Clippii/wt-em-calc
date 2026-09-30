"""Independent radar flight under the explicit geometric_radar_v1 policy."""
from optical_point_flight import update as flight_update
from radar_point_controls import update as controls_update
from geometric_radar import update as seeker_update


def update(p,state,targets,dt,*,illumination_provider=None):
    if p.get('policy')!='geometric_radar_v1':raise ValueError('Use geometric_radar.configure on flight properties')
    def seeker(*args,**kwargs):
        return seeker_update(*args,illumination_provider=illumination_provider,**kwargs)
    def guidance(*args,**kwargs):
        return controls_update(*args,None,seeker_step=seeker,**kwargs)
    return flight_update(p,state,targets,dt,guidance_step=guidance)
