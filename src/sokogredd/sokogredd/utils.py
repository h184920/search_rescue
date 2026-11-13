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


def logg(loggerName, obj):
    print('\n%s:\n%s\n'%(loggerName, str(obj).replace('\n', '\n'+' '*4)))
 

# non-public 

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

if __name__ == '__main__':
    import random

    def pl(liste):
        for row in liste:
            print(row)

    occg = [[(0 if 1 < i < 4 else 0) for i in range(20)] for j in range(20)]
    ones = [1 for i in range(20)]
    occg.insert(2, ones)
    occg.insert(17, ones)
    occg.insert(18, ones)
    for row in occg:
        if random.randint(0, 5) != 5:
            continue
        j = random.randint(7, len(occg)-7)
        length = random.randint(2, 5)
        for i in range(length):
            row[j] = 1
    pl(occg)






