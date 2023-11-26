from math import *
from numpy import float32
import numpy as np

from matplotlib import pyplot as plt

WHEELBASE_2 = 84/2


def update_odometry(dl, dr):
    dl = float32(dl)
    dr = float32(dr)

    r = float32(0.0)
    straight = False
    if (dl == dr):
        straight = True
    else:
        r = (WHEELBASE_2 * (dl + dr)) / (dl - dr)
    theta = 0 if straight else dl / (r + WHEELBASE_2)

    dx = float32(0)
    dy = float32(0)
    if (straight):
        dx = dr
        dy = float32(0.0)
    else:
        dx = cos(theta) * r - r
        dy = sin(theta) * r

    return dx, dy, theta


def clamp(mi, ma, v):
    return max(mi, min(ma, v))


def lerp(a,  b,  t):
    return a + (b - a) * clamp(0, 1, t)


def sin_profile(start,  end,  tmax,  t):
    return lerp(start, end, 0.5 * (1.0 + sin((pi * (clamp(0.0, tmax, t) - tmax * 0.5)) / tmax)))


# print(update_odometry(1.077, 1.077))
# print(update_odometry(1.077, 1.257))
xs = np.linspace(0.0, 10.0, 1000)
plt.plot(xs, np.array([sin_profile(0, 1000, 10, x) for x in list(xs)]))
plt.show()
