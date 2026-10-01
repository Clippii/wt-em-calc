"""Fast snapshot copies for the website's mostly plain-data state graphs.

Retain deepcopy's memo/alias/cycle contract. Subclasses, tuples and custom
objects delegate to the standard implementation with the same memo.
"""
from copy import deepcopy as reference_copy

_ATOMIC = frozenset((float, int, str, bool, bytes, type(None)))


def deepcopy(value, memo=None):
    kind = type(value)
    if kind in _ATOMIC:
        return value
    if kind is not dict and kind is not list:
        return reference_copy(value, memo)
    if memo is None:
        memo = {}
    identity = id(value)
    if identity in memo:
        return memo[identity]
    if kind is dict:
        output = {}
        memo[identity] = output
        for key, item in value.items():
            output[key if type(key) in _ATOMIC else deepcopy(key, memo)] = (
                item if type(item) in _ATOMIC else deepcopy(item, memo))
    else:
        output = []
        memo[identity] = output
        for item in value:
            output.append(item if type(item) in _ATOMIC else deepcopy(item, memo))
    # Match copy._keep_alive for custom callbacks that create transient objects.
    memo.setdefault(id(memo), []).append(value)
    return output
