"""Expand saved ordinary radar properties using the recovered .34 loader.

The inertial loader is 1452974a0; parent 1452ac3cc..3d3 selects the
inertialGuidance child or falls back to the guidance block itself. No executable
is read. Missing authored values use defaults present in those instructions,
not simulator-selected approximations. Raw/noncanonical DataBlock flags and
repeated seeker modes remain outside this catalog-oriented property adapter.
"""
from copy import deepcopy
import math
from kernels import f32
from radar_activation import properties as activation_properties
from radar_measurement_gates import channel_properties


def flag(value, label):
    if type(value) not in (bool, int) or value not in (0, 1):
        raise ValueError(label + ' must be a canonical 0/1 property flag')
    return int(value)


def same(actual, expected, label):
    if actual != expected:
        raise ValueError(f'{label} disagrees with ordinary authored/native properties: {actual!r} != {expected!r}')


def inertial_properties(guidance):
    """Six fields read by the recovered flight inertial helpers.

    An explicitly empty child is different from an absent child: it receives
    loader defaults instead of inheriting parent guidance fields.
    """
    config = guidance['inertialGuidance'] if 'inertialGuidance' in guidance else guidance
    if not isinstance(config, dict):
        raise ValueError('A single inertialGuidance block is required')
    drift = f32(config.get('inertialNavigationDriftSpeed', .5))
    if not math.isfinite(drift):
        raise ValueError('Finite authored inertial drift speed required')
    channels = config.get('datalinkChannelsMax', 32)
    if type(channels) is not int or not -(1 << 31) <= channels < (1 << 31):
        raise ValueError('datalinkChannelsMax must be a signed 32-bit integer')
    return dict(drift_speed=drift,
        gnss=flag(config.get('gnss', abs(drift) < f32(4e-19)), 'gnss'),
        datalink=flag(config.get('datalink', False), 'datalink'),
        reconnect=flag(config.get('reconnectDatalink', False), 'reconnectDatalink'),
        channels_max=channels,
        keep_channels_for_last=flag(config.get('keepDatalinkChannelsForLast', False), 'keepDatalinkChannelsForLast'))


def extend(rocket, loaded_guidance, *, launcher_properties=None):
    """Return ordinary control properties plus full inertial/provider fields.

    ``loaded_guidance`` is the guidance dictionary from saved ordinary
    radar-point-controls/flight evidence, not a geometric-policy profile.
    ``launcher_properties`` may be the matching saved radar-prelaunch property
    dictionary; when supplied, its native drift/GNSS/mask values must agree.
    Known loaded fields and authored channel limits are always cross-checked.
    This reconstructs .34 properties; use by a relocated .38 caller does not
    establish universal build equivalence or re-execute the .38 loader.
    """
    if rocket.get('guidanceType') != 'radar':
        raise ValueError('Radar rocket required')
    config = rocket['guidance']; seeker = config['radarSeeker']
    signature = seeker.get('targetSignatureType', seeker.get('visibilityType', 'radar'))
    if signature != 'radar' or 'mode' in seeker:
        raise NotImplementedError('One conventional radar channel is required')
    out = deepcopy(loaded_guidance)
    manager = out['manager']
    if len(manager['search_values']) != 1:
        raise NotImplementedError('One saved radar channel is required')
    inertial = inertial_properties(config)
    mask = activation_properties(rocket)['primary_mask']
    same(flag(manager['designation_notification'], 'designation_notification'), inertial['datalink'], 'parent+475 datalink')
    same(flag(manager['inertial_navigation'], 'inertial_navigation'),
         flag(config.get('inertialNavigation', False), 'inertialNavigation'), 'parent+15 inertialNavigation')

    distance_config = seeker.get('distance', {})
    flag(distance_config.get('presents', False), 'distance.presents')
    authored = channel_properties(distance_config)
    tracked = out['seeker']['modes']['distance']
    observed = out['seeker']['observation']['weight']['gates']['distance']
    for label, axis in (('mode distance', tracked), ('loaded observation distance', observed)):
        same(flag(axis['present'], label + '.present'), int(authored['present']), label + '.present')
        for key in ('minimum', 'maximum', 'width'):
            same(axis[key], authored[key], label + '.' + key)
    same(out['seeker']['observation']['early_range_base'], authored['minimum'], 'channel+158 range base')
    # Parent+1d4 == radar+16c == channel+158 is MINIMUM; maximum is +1dc.
    # The native inertial-update skip uses this minimum*1.1 threshold.
    extra = dict(designation_mask=mask, datalink_reconnect=inertial['reconnect'],
        datalink_limit=inertial['channels_max'], datalink_counter=inertial['keep_channels_for_last'],
        distance_limits=[dict(present=int(authored['present']), minimum=authored['minimum'])])
    if launcher_properties is not None:
        same(launcher_properties['primary_mask'], mask, 'saved launcher designation mask')
        same(launcher_properties['inertial_drift_speed'], inertial['drift_speed'], 'saved parent+470 drift')
        same(flag(launcher_properties['inertial_no_drift'], 'saved parent+474 GNSS'), inertial['gnss'], 'saved parent+474 GNSS')
        same(flag(launcher_properties['inertial'], 'saved parent+15 inertial'), flag(manager['inertial_navigation'], 'inertial_navigation'), 'saved launcher inertial flag')
    if 'inertial' in out:
        same(out['inertial'], inertial, 'existing expanded inertial properties')
    for key, value in extra.items():
        if key in manager:
            same(manager[key], value, 'existing manager.' + key)
        manager[key] = value
    out['inertial'] = inertial
    return out
