import math
from component_assembly import f32, add, mul


def scalar_stage(x, transverse, inverse_speed2, travel_cap, roll_rate,
                 current_cl, cl_history_rate, inverse_halfspan,
                 longitudinal_exponent, amplitude, coefficient, inverse_span2):
    a, b = transverse
    distance2 = add(add(mul(a,a), mul(b,b)), mul(x,x))
    travel = min(f32(math.sqrt(mul(distance2,inverse_speed2))), travel_cap)
    angle = f32(float(travel) * roll_rate)
    lateral = mul(add(mul(f32(math.cos(angle)),a), mul(f32(math.sin(angle)),b)), inverse_halfspan)
    delayed_cl = add(mul(travel,cl_history_rate),current_cl)
    longitudinal = add(f32(math.exp(mul(x,longitudinal_exponent))),f32(.9))
    strength = mul(mul(longitudinal,amplitude),delayed_cl)
    attenuation = f32(math.exp(mul(abs(lateral),f32(-1.4096779823303223))))
    return mul(mul(attenuation,inverse_span2),mul(strength,coefficient))
