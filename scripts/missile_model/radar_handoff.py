"""Radar launcher history inherited by constructor 1452aeaf0 (client 2.59.0.34).

This transfer requires a supplied, verified cold flight-constructor template.
Controller construction and release initialization are separate operations.
Semantic codecs accept canonical flags and finite binary32 values; arbitrary
native byte blocks must use transferred_blocks without decoding them.
"""
from copy import deepcopy
import math
import struct


SEEKER_SIZE = 88
SEEKER_PREFIX_SIZE = 76
INERTIAL_SIZE = 44
SEEKER_PADDING = (0x26, 0x27, 0x31, 0x32, 0x33, 0x52, 0x53)
SEEKER_FLOATS = {0x20: 'gap', 0x28: 'range', 0x2c: 'range_rate',
                 0x34: 'closure', 0x38: 'closure_rate', 0x48: 'target_value',
                 0x4c: 'los_check_time', 0x54: 'visibility_weight'}
SEEKER_FLAGS = {0x24: 'rejected', 0x25: 'range_valid', 0x30: 'closure_valid',
                0x50: 'los_cache_valid', 0x51: 'visibility_clear'}
SEEKER_INTS = {0x3c: 'associated_id', 0x40: 'channel_index', 0x44: 'target_id'}
SEEKER_VECTORS = {0: ('angles', 2), 8: ('direction', 3), 0x14: ('angular_rate', 3)}
INERTIAL_PADDING = (1, 2, 3, 41, 42, 43)
INERTIAL_VECTORS = {4: 'point', 16: 'velocity', 28: 'offset'}


def _raw(value, size, label):
    value = bytes(value)
    if len(value) != size:
        raise ValueError(f'{label} requires {size} bytes')
    return value


def _byte(value):
    if not isinstance(value, int) or not 0 <= value <= 255:
        raise ValueError('Unsigned byte required')
    return value


def _flag(value):
    value = _byte(value)
    if value not in (0, 1):
        raise ValueError('Semantic states require canonical 0/1 flags; keep arbitrary blocks raw')
    return value


def _float(value):
    if not math.isfinite(value):
        raise ValueError('Semantic states require finite floats; keep arbitrary blocks raw')
    # Packing also checks that a supplied finite Python float fits binary32.
    return value


def _padding(raw, offsets, values):
    if len(values) != len(offsets):
        raise ValueError(f'{len(offsets)} padding bytes required')
    for offset, value in zip(offsets, values):
        raw[offset] = _byte(value)


def pack_seeker(state):
    """Encode the complete canonical 88-byte seeker state, preserving padding."""
    raw = bytearray(SEEKER_SIZE)
    for offset, key in SEEKER_FLOATS.items():
        struct.pack_into('<f', raw, offset, _float(state[key]))
    for offset, key in SEEKER_FLAGS.items():
        raw[offset] = _flag(state[key])
    for offset, key in SEEKER_INTS.items():
        struct.pack_into('<i', raw, offset, state[key])
    for offset, (key, count) in SEEKER_VECTORS.items():
        if len(state[key]) != count:
            raise ValueError(f'{key} requires {count} floats')
        struct.pack_into(f'<{count}f', raw, offset, *map(_float, state[key]))
    _padding(raw, SEEKER_PADDING, state['padding'])
    return bytes(raw)


def unpack_seeker(raw):
    """Decode finite canonical state; reject flags/payloads that would be lossy."""
    raw = _raw(raw, SEEKER_SIZE, 'Seeker state')
    out = dict(padding=[raw[offset] for offset in SEEKER_PADDING])
    for offset, key in SEEKER_FLOATS.items():
        out[key] = _float(struct.unpack_from('<f', raw, offset)[0])
    for offset, key in SEEKER_FLAGS.items():
        out[key] = bool(_flag(raw[offset]))
    for offset, key in SEEKER_INTS.items():
        out[key] = struct.unpack_from('<i', raw, offset)[0]
    for offset, (key, count) in SEEKER_VECTORS.items():
        out[key] = list(map(_float, struct.unpack_from(f'<{count}f', raw, offset)))
    return out


def pack_inertial(state):
    """Encode all 44 inertial bytes, including velocity, source and six padding bytes."""
    raw = bytearray(INERTIAL_SIZE)
    raw[0] = _flag(state['valid'])
    for offset, key in INERTIAL_VECTORS.items():
        if len(state[key]) != 3:
            raise ValueError(f'{key} requires three floats')
        struct.pack_into('<3f', raw, offset, *map(_float, state[key]))
    raw[40] = _byte(state['source_tag'])
    _padding(raw, INERTIAL_PADDING, state['padding'])
    return bytes(raw)


def unpack_inertial(raw):
    raw = _raw(raw, INERTIAL_SIZE, 'Inertial state')
    out = dict(valid=bool(_flag(raw[0])), source_tag=raw[40],
               padding=[raw[offset] for offset in INERTIAL_PADDING])
    for offset, key in INERTIAL_VECTORS.items():
        out[key] = list(map(_float, struct.unpack_from('<3f', raw, offset)))
    return out


def manager_inertial(manager):
    """Read full flat flight storage; absent legacy fields are not assumed zero."""
    return {key: deepcopy(manager['inertial_' + key]) for key in
            ('valid', 'point', 'velocity', 'offset', 'source_tag', 'padding')}


def transferred_blocks(cold_seeker, launcher_seeker_prefix, launcher_inertial,
                       association_flag, lock_after_launch):
    """Raw native transfer, retaining noncanonical flags and NaN payloads.

    The final 12 seeker bytes come from the supplied cold constructor block.
    This is a byte-level copy operation, not acceptance of these arbitrary
    states as engine-valid flight conditions. Authored LAL normally is 0/1;
    the raw operation retains the native byte xor/multiply for completeness.
    """
    cold_seeker = _raw(cold_seeker, SEEKER_SIZE, 'Cold seeker state')
    prefix = _raw(launcher_seeker_prefix, SEEKER_PREFIX_SIZE, 'Launcher seeker prefix')
    inertial = _raw(launcher_inertial, INERTIAL_SIZE, 'Launcher inertial state')
    return dict(seeker=prefix + cold_seeker[SEEKER_PREFIX_SIZE:], inertial=inertial,
                association_flag=_byte(association_flag),
                mode=3 * (_byte(lock_after_launch) ^ 1), transition_time=0.)


def constructed_guidance(launcher, template, *, lock_after_launch):
    """Transfer full launcher history into a supplied cold flight template.

    Copies seeker +00..+4b, all inertial bytes and association byte. The seeker
    cache tail (including its final two padding bytes), controller histories,
    activation status and published outputs remain from the cold template.
    Launcher phase does not determine flight mode. Normal release initialization
    must follow this constructor transfer.
    """
    _flag(lock_after_launch)
    out = deepcopy(template)
    transfer = transferred_blocks(pack_seeker(template['seeker']),
        pack_seeker(launcher['seeker'])[:SEEKER_PREFIX_SIZE],
        pack_inertial(launcher['inertial']), launcher['association_flag'],
        lock_after_launch)
    ss = out['seeker'] = unpack_seeker(transfer['seeker'])
    manager = out['manager']
    manager.update(mode=transfer['mode'], transition_time=transfer['transition_time'],
        association_flag=transfer['association_flag'], seeker_angles=ss['angles'][:],
        seeker_direction=ss['direction'][:], seeker_range=ss['range'],
        seeker_closure=ss['closure'], range_valid=ss['range_valid'],
        closure_valid=ss['closure_valid'], channel_index=ss['channel_index'])
    for key, value in unpack_inertial(transfer['inertial']).items():
        manager['inertial_' + key] = value
    return out
