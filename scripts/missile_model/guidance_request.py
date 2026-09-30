"""Independent request arithmetic for the pinned game's PN-family routine.

Inputs are loaded properties and supplied tracking/flight state, not a sensor
model. The returned request precedes the acceleration controller and body.
Finite inputs and valid ordered tables are the investigated domain.
"""
import math
from kernels import f32, add, sub, mul
from guidance_modes import transition, loft_acceleration


def table_value(rows, value):
    value = f32(value)
    if not rows:
        return 0.
    if len(rows) == 1 or value <= rows[0][0]:
        return rows[0][2]
    if value >= rows[-1][0]:
        return rows[-1][2]
    for previous, following in zip(rows, rows[1:]):
        if value <= following[0]:
            fraction = mul(sub(value, previous[0]), previous[1])
            return add(mul(sub(following[2], previous[2]), fraction), previous[2])
    raise ValueError('Unordered or invalid native table')


def target_angles(los, q):
    """Quaternion LOS transform, preserving the native scalar operation order."""
    lx, ly, lz = map(f32, los)
    x, y, z, w = map(f32, q)
    twice_z = add(z, z); negative_twice_w = mul(-2., w)
    x_twice_z = mul(x, twice_z); y_negative_twice_w = mul(y, negative_twice_w)
    y_twice_z = mul(twice_z, y); x_negative_twice_w = mul(x, negative_twice_w)
    z_negative_twice_w = mul(negative_twice_w, z)
    twice_xy = mul(add(x, x), y)
    twice_ww_minus_one = add(add(mul(w, w), mul(w, w)), -1.)
    bz = add(mul(add(add(mul(z, z), mul(z, z)), twice_ww_minus_one), lz),
        add(mul(add(x_negative_twice_w, y_twice_z), ly), mul(sub(x_twice_z, y_negative_twice_w), lx)))
    by = add(add(mul(sub(y_twice_z, x_negative_twice_w), lz), mul(add(twice_xy, z_negative_twice_w), lx)),
        mul(add(add(mul(y, y), mul(y, y)), twice_ww_minus_one), ly))
    bx = add(add(mul(add(y_negative_twice_w, x_twice_z), lz), mul(sub(twice_xy, z_negative_twice_w), ly)),
        mul(add(add(mul(x, x), mul(x, x)), twice_ww_minus_one), lx))
    pitch = f32(math.atan2(by, f32(math.sqrt(add(mul(bz, bz), mul(bx, bx))))))
    yaw = f32(math.atan2(-bz, bx))
    return pitch, yaw


def request(p, mode, course, los, rate, navigation_velocity, quaternion, velocity,
            position, time, time_to_hit, range_squared, height_query=None):
    gain = mul(table_value(p['time_gain'], time), table_value(p['time_to_hit_gain'], time_to_hit))
    if gain < f32(.001):
        return dict(mode=mode, course=course, gain=gain, acceleration=[])
    pitch, yaw = target_angles(los, quaternion)
    rx, ry, rz = map(f32, rate)
    omega = p['omega_gain']; angle = p['angle_gain']
    command = [mul(mul(omega, gain), rx),
        mul(sub(mul(ry, omega), mul(angle, yaw)), gain),
        mul(sub(mul(rz, omega), mul(add(pitch, p['elevation_add']), angle)), gain)]
    if course == 1:
        if abs(yaw) > p['course_final_angle']:
            command[1] = mul(-yaw, p['course_gain'])
        else:
            course = 0
    vx, vy, vz = map(f32, navigation_velocity)
    cx, cy, cz = command
    acceleration = [sub(mul(vy, cz), mul(vz, cy)),
        sub(mul(cx, vz), mul(vx, cz)), sub(mul(vx, cy), mul(vy, cx))]
    mode = transition(mode, p, los, rate, quaternion, time_to_hit, gain)
    if mode == 1:
        acceleration[1] = loft_acceleration(p, quaternion, range_squared, acceleration[1])
    elif mode == 2:
        altitude = f32(position[1]) if p['absolute_altitude'] else height_query
        # None represents the caller's unsuccessful height query.
        if altitude is not None:
            vx, vy, vz = map(f32, velocity)
            target = table_value(p['altitude_table'], time_to_hit)
            climb = sub(mul(sub(target, altitude), p['altitude_diff_gain']), mul(p['altitude_rate_gain'], vy))
            climb = min(max(climb, p['climb_limits'][0]), p['climb_limits'][1])
            horizontal_squared = add(mul(vz, vz), mul(vx, vx))
            pitch_velocity = f32(math.atan2(vy, f32(math.sqrt(horizontal_squared))))
            speed = f32(math.sqrt(add(mul(vy, vy), horizontal_squared)))
            acceleration[1] = mul(mul(speed, p['climb_accel_gain']), sub(climb, pitch_velocity))
    return dict(mode=mode, course=course, gain=gain, acceleration=acceleration,
        target_pitch=pitch, target_yaw=yaw, angular_command=command)
