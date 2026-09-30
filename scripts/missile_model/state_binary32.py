"""Portable JSON encoding for native-produced non-finite binary32 state words.

Finite values keep their existing JSON representation. Only infinity and quiet
NaN words use a reserved tagged object; their sign and payload survive restart.
This codec does not make arbitrary non-finite body/property inputs supported.
"""
import math
import re
import struct

ENCODING='binary32-special-v1'
TAG='$binary32'


def encode(value):
    if isinstance(value,float) and not math.isfinite(value):
        return {TAG:struct.pack('<f',value).hex()}
    if isinstance(value,dict):
        if TAG in value:raise ValueError('Reserved binary32 state tag in source object')
        return {key:encode(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [encode(item) for item in value]
    return value


def decode(value):
    if isinstance(value,dict):
        if TAG in value:
            raw=value[TAG]
            if set(value)!={TAG} or not isinstance(raw,str) or re.fullmatch('[0-9a-fA-F]{8}',raw) is None:
                raise ValueError('Invalid binary32 special-state tag')
            word=struct.unpack('<I',bytes.fromhex(raw))[0]
            if word&0x7f800000!=0x7f800000 or (word&0x7fffff and not word&0x400000):
                raise ValueError('State tags support only infinity and quiet NaN words')
            return struct.unpack('<f',bytes.fromhex(raw))[0]
        return {key:decode(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [decode(item) for item in value]
    return value


def contains_special(value):
    if isinstance(value,float):return not math.isfinite(value)
    if isinstance(value,dict):return any(contains_special(v) for v in value.values())
    if isinstance(value,(list,tuple)):return any(contains_special(v) for v in value)
    return False


def validate_flight_numbers(state):
    """Permit special words only in the represented controller PID records.

    Body, seeker, manager outputs, clocks and applied controls remain finite.
    The ordinary state schema is otherwise enforced by the existing consumers.
    """
    def pid_slot(path):
        if len(path)==5 and path[:1]==('guidance',) and path[1] in ('orientation','guidance') and path[2]=='pid':
            return type(path[3]) is int and 0<=path[3]<2 and type(path[4]) is int and 0<=path[4]<8
        return len(path)==4 and path[:3]==('guidance','propulsion','pid') and type(path[3]) is int and 0<=path[3]<8
    def visit(value,path):
        if isinstance(value,float) and not math.isfinite(value) and not pid_slot(path):
            raise ValueError('state.'+'.'.join(map(str,path))+' must be finite')
        if isinstance(value,dict):
            for key,item in value.items():visit(item,path+(key,))
        elif isinstance(value,(list,tuple)):
            for index,item in enumerate(value):visit(item,path+(index,))
    visit(state,())
