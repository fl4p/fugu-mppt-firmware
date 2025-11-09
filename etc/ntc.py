import math


def v2t(vr):
    beta = 3380
    T0 = 25.
    R0 = 10e3
    Rp = 10e3
    return 1 / (math.log(Rp / R0 * (1 / vr - 1)) / beta + 1 / (T0+273.15)) - 273.15


def r2t(r):
    beta = 3380
    T0 = 25.
    R0 = 10e3
    return 1 / (1 / beta * math.log(r / R0) + 1 / T0)


print(v2t(2.8810/3.3))