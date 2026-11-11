import math as m
from random import randint
from geometry_msgs.msg import Point


def distance(x1, y1, x2, y2):
    return m.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def mapToRealPoint(occGrid, w, h):
    x = __mapToRealX(occGrid, w)
    y = __mapToRealY(occGrid, h)
    return Point(x=x, y=y)


def findWBorders(occGrid):
    return __findOccupancyGridBorders(occGrid, lambda w, h: (h, w), __mapToRealX)
  

def findHBorders(occGrid):
    return __findOccupancyGridBorders(occGrid, lambda w, h: (w, h), __mapToRealY)


def randfloat(a, b):
    return randint(int(a*100), int(b*100))/100


def logg(instance, obj):
    print(' %s'%(obj))


# non-public functions

def __findOccupancyGridBorders(occGrid, arrangeDim, realCoordinate):
    _ = occGrid.info
    parallellN, orthagonalN = arrangeDim(_.width, _.height)
    borders = []
    for p in range(parallellN):
        oStart = __findEdge(occGrid, orthagonalN, lambda o: arrangeDim(p, o))
        oStop = __findEdge(occGrid, orthagonalN, lambda o: arrangeDim(p, orthagonalN-o-1))
        borders.append(oStart, oStop)
    return borders

def __findEdge(occGrid, orthagonalLen, wh):
    status = 0
    for o in range(1, orthagonalLen):
        w, h = wh(o)
        cell = occGrid.data[occGrid.info.width*h + w]
        if status == cell: continue
        if not cell: break
        status = 1
    return o

def __mapToRealX(occGrid, w):
    O = occGrid.info.origin.position
    cellUnit = occGrid.info.resolution
    return O.x + cellUnit*w

def __mapToRealY(occGrid, h):
    O = occGrid.info.origin.position
    cellUnit = occGrid.info.resolution
    return O.y + cellUnit*h





