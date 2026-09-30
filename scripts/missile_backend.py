"""Optional acceleration, bound only inside the isolated missile interpreter.

WT_MISSILE_BACKEND=reference keeps the full reference path. 'python' uses fast
session orchestration without an extension. 'auto' uses a valid local extension
when present; missing/stale binaries fall back to fast Python without a build.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from build_missile_backend import DIRECTORY, signature


def activate():
    mode = os.environ.get('WT_MISSILE_BACKEND', 'auto')
    if mode not in ('auto', 'python', 'reference', 'compiled'):
        raise ValueError('Unknown WT_MISSILE_BACKEND')
    if mode in ('python', 'reference'):
        return mode
    try:
        manifest = json.loads((DIRECTORY/'manifest.json').read_text())
        if manifest['signature'] != signature():
            raise ValueError('Compiled helper source/ABI mismatch')
        library = manifest['library']
        if Path(library).name != library:
            raise ValueError('Invalid compiled library directory')
        binaries = manifest['binaries']
        if len(binaries) != 1:
            raise ValueError('Expected one compiled helper')
        name, digest = next(iter(binaries.items()))
        if Path(name).name != name:
            raise ValueError('Invalid compiled helper path')
        path = DIRECTORY/library/name
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('Compiled helper hash mismatch')
        spec = importlib.util.spec_from_file_location('_missile_accel', path)
        accelerator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(accelerator)
    except (OSError, ValueError, KeyError, TypeError, AttributeError, ImportError):
        if mode == 'compiled':
            raise RuntimeError('Compiled missile helpers unavailable; run scripts/build_missile_backend.py')
        return 'python'

    import copy
    import kernels
    import flight_session
    import state_binary32
    replacements = {kernels.f32: accelerator.f32, kernels.add: accelerator.add,
                    kernels.sub: accelerator.sub, kernels.mul: accelerator.mul,
                    kernels.div: accelerator.div, copy.deepcopy: accelerator.deepcopy,
                    flight_session.finite: accelerator.finite,
                    state_binary32.validate_flight_numbers: accelerator.validate_flight_numbers}
    scripts = Path(__file__).resolve().parent
    model = scripts/'missile_model'
    # Avoid touching EM, standard-library modules, or unrelated interpreters.
    for module in list(sys.modules.values()):
        filename = getattr(module, '__file__', None)
        if not filename:
            continue
        path = Path(filename).resolve()
        if path.parent != model and not (path.parent == scripts and path.stem in
                {'missile_worker', 'missile_support', 'missile_initialization',
                 'missile_telemetry', 'missile_fast'}):
            continue
        for key, value in list(vars(module).items()):
            if callable(value):
                try:
                    replacement = replacements.get(value)
                except TypeError:
                    continue
                if replacement is not None:
                    setattr(module, key, replacement)
    return 'compiled'
