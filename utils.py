from math import atan, acos, cos, sin, atan2, sqrt
import matplotlib.pyplot as plt
from IPython import embed
from numpy.linalg import inv
import numpy as np
from numpy import pi

                      #        x,y
#System size          #        /\
l1 = 0.06             #       /  \
l2 = 0.165            # l2-> /    \<-r2
r1 = 0.060            # l1 ->\_dd_/<-r1
r2 = 0.163            #  |   q1  q2
d  = 0.150 / 2        # Y|
                      #  |____>
                      #    X


def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    d=sqrt((x1-x0)**2 + (y1-y0)**2)
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < abs(r0-r1):
        return np.array([np.nan,np.nan])
    # coincident circles
    if d == 0 and r0 == r1:
        return np.array([np.nan,np.nan])
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 
        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        return (x3, y3, x4, y4)

def fg(q1, q2, positive=True):
    sign = 1 if positive else -1
    x0 = -d + l1 * -sin(q1)
    y0 =      l1 * cos(q1)
    x1 =  d + r1 * -sin(q2)
    y1 =      r1 * cos(q2)

    (x3, y3, x4, y4) = get_intersections(x0, y0, l2, x1, y1, r2)
    return np.array([x4, y4])

def J(q1, q2):
    eps = 1e-6
    dq1 = eps * 1 
    dq2 = eps * 1

    return 1./eps * np.array([fg(q1+dq1, q2) - fg(q1, q2),fg(q1, q2+dq2) - fg(q1, q2)]).T