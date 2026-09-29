"""Lookup of the frozen chart-search domain; no reference aircraft calculations."""
from bisect import bisect_right

from em_search_region_data import SEARCH_REGION, REFERENCE_CONDITIONS

TOLERANCE_G = 1.
TOLERANCE_DPS = 2.
SPEEDS = tuple(row[0] for row in SEARCH_REGION)


def reference_values(speed):
    """Return saved composite and ceiling loads, with endpoint holds outside TAS range.

    The final ceiling was calculated once using the smaller of +1 g and
    +2 degrees/s. Runtime work is only linear lookup in the frozen table.
    """
    speed=float(speed)
    if speed<=SPEEDS[0]:return SEARCH_REGION[0][1:]
    if speed>=SPEEDS[-1]:return SEARCH_REGION[-1][1:]
    i=bisect_right(SPEEDS,speed)
    a,b=SEARCH_REGION[i-1:i+1]
    t=(speed-a[0])/(b[0]-a[0])
    return tuple(a[j]+t*(b[j]-a[j]) for j in (1,2))


def reference_ceiling(speed):
    return reference_values(speed)[1]
