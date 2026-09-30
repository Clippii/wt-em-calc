"""Independent no-provider radar manager with recovered shared controllers."""
from optical_manager_controls import update as controls_update
from radar_manager import update as manager_update


def update(p,state,old,new,elapsed,clocks,seeker,*,suppress=False,context_value=0.,height_query=None,environment=None,matrix_velocity_frame=True):
    return controls_update(p,state,old,new,elapsed,clocks,seeker,suppress=suppress,
        context_value=context_value,height_query=height_query,manager_step=manager_update,environment=environment,
        matrix_velocity_frame=matrix_velocity_frame)
