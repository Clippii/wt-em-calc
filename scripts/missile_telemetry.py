"""Read-only flight diagnostics, kept outside the hash-pinned model bundle.

Re-evaluate the final substep's pure force function with the exact inputs used
by the completed update. Never advance propulsion again or commit any state.
"""
import math

from aero_vectors import forces
from kernels import atmosphere, div


FORCE_FIELDS = ('force_time_s', 'thrust_n', 'mass_kg', 'propellant_used_kg',
                'drag_n', 'lift_n', 'drag_coefficient', 'aoa_deg',
                'dynamic_pressure_pa', 'acceleration_mps2')


def initial(properties):
    # No force integration has occurred at release. Do not invent a zero thrust.
    return dict.fromkeys(FORCE_FIELDS) | dict(mass_kg=properties['body']['mass'])


def completed(properties, before, controls, update, dt):
    substeps = update['substeps']
    body = substeps[-2]['state'] if len(substeps) > 1 else before
    propulsion = update['propulsion']
    aero = forces(properties['body'], body['position'][1], body['velocity'],
                  body['quaternion'], body['omega'], fins=controls[:2],
                  dt=div(dt, properties['precision']),
                  force=propulsion['world_force'], torque=propulsion['body_torque'],
                  mass_lost=propulsion['mass_lost'], body_random=body['body_random'])
    scale = aero['perturbation']['force_scale']
    return dict(force_time_s=body['time'],
                thrust_n=math.hypot(*propulsion['world_force']),
                mass_kg=aero['effective_mass'], propellant_used_kg=propulsion['mass_lost'],
                drag_n=math.hypot(*aero['baseline']['drag']) * abs(scale),
                lift_n=math.hypot(*aero['baseline']['lift']) * abs(scale),
                drag_coefficient=aero['baseline']['cd'],
                aoa_deg=math.degrees(math.acos(max(-1., min(1., aero['baseline']['cosine'])))),
                dynamic_pressure_pa=aero['pressure'],
                acceleration_mps2=math.hypot(*substeps[-1]['acceleration']))


def mach(body):
    return math.hypot(*body['velocity']) / atmosphere(body['position'][1])['sound_speed']
