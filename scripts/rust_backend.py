"""Optional dependency-free Rust kernels; no compilation during app startup.

WT_NUMERIC_BACKEND=auto accelerates Python modules when a verified build exists.
The existing Cython backend stays preferred for compiled modules. Explicit
'rust' also uses Rust inside Cython; 'python' disables these kernels entirely.
"""
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / '.native_rust'
FIELDS = ('cl0 cd0 indCoeff clLineCoeff cyCritH cyCritL aoaCritH aoaCritL '
          'aoaLineH aoaLineL parabCyCoeffH parabCyCoeffL aerCenterOffset '
          'clToCm0 clToCm1 parabAngle declineCoeff maxDistAng cdAfterCoeff '
          'clAfterCritL clAfterCritH kq clKq cyMult').split()
_library = None
_attempted = False
D = ctypes.c_double
P = ctypes.POINTER(D)


def signature():
    digest = hashlib.sha256()
    for file in (ROOT/'native/Cargo.toml', ROOT/'native/Cargo.lock',
                 ROOT/'native/src/lib.rs', Path(__file__)):
        digest.update(file.read_bytes())
    digest.update((sys.platform + platform.machine().lower()).encode())
    return digest.hexdigest()


def load(module_file):
    global _library, _attempted
    mode = os.environ.get('WT_NUMERIC_BACKEND', 'auto')
    if mode not in ('auto', 'rust', 'python'):
        raise ValueError('Unknown WT_NUMERIC_BACKEND: ' + mode)
    if mode == 'python' or (mode == 'auto' and not module_file.endswith('.py')):
        return None
    if not _attempted:
        _attempted = True
        try:
            manifest = json.loads((DIRECTORY/'manifest.json').read_text())
            if manifest['signature'] != signature():
                raise ValueError('Rust kernel source/platform mismatch')
            name = manifest['binary']
            if Path(name).name != name:
                raise ValueError('Invalid Rust library path')
            path = DIRECTORY/name
            if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['sha256']:
                raise ValueError('Rust library hash mismatch')
            library = ctypes.CDLL(str(path))
            library.wt_numeric_abi.argtypes = []
            library.wt_numeric_abi.restype = ctypes.c_uint32
            if library.wt_numeric_abi() != 1:
                raise ValueError('Unsupported Rust numeric ABI')
            library.wt_polar.argtypes = [P, D, D, D, D, ctypes.c_uint32, P]
            library.wt_polar.restype = None
            library.wt_polar_batch.argtypes = [P, P, ctypes.c_size_t, P]
            library.wt_polar_batch.restype = None
            library.wt_atmosphere.argtypes = [D, P]
            library.wt_atmosphere.restype = None
            library.wt_orientation.argtypes = [P, P]
            library.wt_orientation.restype = ctypes.c_uint32
            for name in ('wt_force', 'wt_moment'):
                function = getattr(library, name)
                function.argtypes = [P, P]
                function.restype = None
            _library = library
        except (OSError, ValueError, KeyError, AttributeError) as error:
            if mode == 'rust':
                raise RuntimeError('Rust kernels unavailable; run scripts/build_rust_backend.py') from error
    if mode == 'rust' and _library is None:
        raise RuntimeError('Rust kernels unavailable; run scripts/build_rust_backend.py')
    return _library


def _array(values):
    values = tuple(values)
    # Preserve the reference's struct.pack overflow and trig domain errors.
    if any(not math.isfinite(v) or abs(v) > 3.4028234663852886e38 for v in values):
        return None
    return (D * len(values))(*values)


def polar(library, p, a, angle=0., cl_add=0., cd_coeff=1., mode=2):
    try:
        packed = _array(p[key] for key in FIELDS)
    except KeyError:
        return None
    if packed is None or _array((a, angle, cl_add, cd_coeff)) is None:
        return None
    result = (D * 2)()
    library.wt_polar(packed, a, angle, cl_add, cd_coeff, mode, result)
    return list(result) if all(math.isfinite(v) for v in result) else None


def assembly(library, values, moment=False):
    packed = _array(values)
    if packed is None:
        return None
    expected = 45 if moment else 24
    if len(packed) != expected:
        return None
    result = (D * 3)()
    (library.wt_moment if moment else library.wt_force)(packed, result)
    return list(result) if all(math.isfinite(v) for v in result) else None


def polar_batch(p, rows):
    """Evaluate [angle, rotation, added lift, drag multiplier] rows in one call."""
    library = load(__file__)
    rows = [tuple(row) for row in rows]
    if any(len(row) != 4 for row in rows):
        raise ValueError('Each polar row must contain four values')
    packed = _array(p[key] for key in FIELDS)
    inputs = _array(v for row in rows for v in row)
    if library is None or packed is None or inputs is None:
        from polar_f32 import calc_c
        return [calc_c(p, *row) for row in rows]
    result = (D * (len(rows)*2))()
    library.wt_polar_batch(packed, inputs, len(rows), result)
    if not all(math.isfinite(v) for v in result):
        from polar_f32 import calc_c
        return [calc_c(p, *row) for row in rows]
    return [list(result[i:i+2]) for i in range(0, len(result), 2)]


def atmosphere_function(library, reference):
    def atmosphere(height):
        if not math.isfinite(height) or abs(height)>3.4028234663852886e38:
            return reference(height)
        result = (D * 3)()
        library.wt_atmosphere(height, result)
        if not all(math.isfinite(v) for v in result):
            return reference(height)
        return dict(zip(('density', 'sound_speed', 'pressure'), result))
    return atmosphere


def orientation_function(library, reference):
    def orientation(quaternion, increment):
        values = (*quaternion, *increment)
        packed = _array(values)
        if packed is None or len(quaternion)!=4 or len(increment)!=3:
            return reference(quaternion, increment)
        result = (D * 24)()
        if not library.wt_orientation(packed, result) or not all(math.isfinite(v) for v in result):
            return reference(quaternion, increment)
        trig = [dict(sine=result[i],cosine=result[i+1],quadrant=int(result[i+2]),reduced=result[i+3]) for i in (0,4,8)]
        return dict(trig=trig,delta=list(result[12:16]),raw=list(result[16:20]),quaternion=list(result[20:24]))
    return orientation
