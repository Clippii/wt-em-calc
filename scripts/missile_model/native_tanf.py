"""Finite binary32 tanf arithmetic from client .38's scalar SSE path.

No native execution, executable inspection, or host tangent call. Constants and
the fixed-point reduction table are independently hash guarded. Scope is CPU
dispatch0 and round-to-nearest with gradual underflow (probe MXCSR0x1f80).
Floating-point status, errno, nonfinite inputs and AVX/FMA dispatch are outside
this numerical port.
"""
import hashlib
import json
import math
from pathlib import Path
import struct

from kernels import f32


DATA_HASH = '61aae7bb0a47e170eaa5d2a9351fb3ee1b8b082711801bf23db591405ab8608a'
TABLE_HASH = '12598143dcc0994f6a4711a260fd086f065c20560573e3030cf154dca9b8284c'
MASK64 = (1 << 64)-1


def load_data():
    saved = json.loads(Path(__file__).with_name('tanf-constants.json').read_text())
    data = {int(a,16):bytes.fromhex(raw) for a,raw in saved['constants'].items()}
    raw = b''.join(struct.pack('<QI',a,len(value))+value for a,value in sorted(data.items()))
    if hashlib.sha256(raw).hexdigest() != DATA_HASH:
        raise ValueError('tanf constants differ from the audited SSE implementation')
    table = data[0x147545700]
    if len(table) != 156 or hashlib.sha256(table).hexdigest() != TABLE_HASH:
        raise ValueError('tanf argument-reduction table differs from the audited implementation')
    return data


DATA = load_data()
TABLE = DATA[0x147545700]


def _double(address):
    return struct.unpack('<d',DATA[address][:8])[0]


QUARTER_PI = _double(0x147541438)
SMALL = _double(0x147541400)
CUBIC = _double(0x147541408)
LARGE = _double(0x1475413e8)
THIRD = _double(0x1475413f8)
TWO_OVER_PI = _double(0x147541380)
PI2_HEAD = _double(0x147541418)
PI2_TAIL = _double(0x147541420)
HALF_PI = _double(0x147541428)
P0,P1 = (_double(a) for a in (0x1475413c0,0x1475413c8))
Q0,Q1,Q2 = (_double(a) for a in (0x1475413d0,0x1475413d8,0x1475413e0))


def _polynomial(x):
    squared = x*x
    numerator = P1*squared
    numerator = numerator+P0
    denominator = Q2*squared
    denominator = denominator+Q1
    denominator = denominator*squared
    denominator = denominator+Q0
    ratio = numerator/denominator
    correction = squared*x
    correction = correction*ratio
    return x+correction


def _large_reduce(magnitude):
    """Translate the three-word integer reduction at146b6c228..146b6c386."""
    word = struct.unpack('<Q',struct.pack('<d',magnitude))[0]
    exponent = (word >> 52)-0x3ff
    offset = 0x86-(exponent >> 3)
    mantissa = (word & ((1 << 52)-1)) | (1 << 52)
    first,second,third = struct.unpack_from('<3Q',TABLE,offset)
    product = first*mantissa
    low = product & MASK64
    product = second*mantissa+(product >> 64)
    middle = product & MASK64
    high = ((product >> 64)+(third*mantissa & MASK64)) & MASK64
    residual_exponent = exponent & 7
    shift = 0x36-residual_exponent
    quadrant = high >> shift
    carry = (high >> (shift-1)) & 1
    sign = 0
    if carry:
        high ^= MASK64
        middle ^= MASK64
        low ^= MASK64
        sign = 1 << 63
    quadrant = (quadrant+carry) & 3
    shift = residual_exponent+10
    high = ((high << shift) & MASK64) >> shift
    result_exponent = shift-64
    if high == 0:
        high,middle,low = middle,low,0
        result_exponent -= 64
    if high == 0:
        raise ArithmeticError('Unresolved zero significand in finite tanf reduction')
    leading = high.bit_length()-1
    result_exponent += leading
    shift = leading-52
    if shift > 0:
        previous = high
        high >>= shift
        middle = (middle >> shift) | ((previous << (64-shift)) & MASK64)
    elif shift < 0:
        shift = -shift
        previous = middle
        high = ((high << shift) & MASK64) | (previous >> (64-shift))
        middle = ((middle << shift) & MASK64) | (low >> (64-shift))
    # Native code discards the remaining low words without rounding them.
    result_exponent += 0x3ff
    high = (high & ~(1 << 52)) | sign | (result_exponent << 52)
    reduced = struct.unpack('<d',struct.pack('<Q',high))[0]
    return reduced*HALF_PI,quadrant


def tanf(x):
    """Return the recovered finite SSE tanf result, preserving signed zero."""
    x = f32(x)
    if not math.isfinite(x):
        raise ValueError('Only finite tanf inputs are reconstructed')
    magnitude = abs(x)
    if magnitude <= QUARTER_PI:
        if magnitude < SMALL:
            # The original performs an inexact-status operation in XMM1 but
            # returns its unchanged XMM0 input. Status flags are outside scope.
            return x
        if magnitude < CUBIC:
            correction = x*x
            correction = correction*x
            correction = correction*THIRD
            return f32(correction+x)
        return f32(_polynomial(x))
    if magnitude < LARGE:
        index = math.trunc(magnitude*TWO_OVER_PI+.5)
        head = float(index)*PI2_HEAD
        residual = magnitude-head
        tail = float(index)*PI2_TAIL
        reduced = residual-tail
        quadrant = index & 3
    else:
        reduced,quadrant = _large_reduce(magnitude)
    result = _polynomial(reduced)
    if quadrant & 1:
        result = -1./result if result != 0. else math.copysign(math.inf,-result)
    if math.copysign(1.,x) < 0.:
        result = -result
    return f32(result)
