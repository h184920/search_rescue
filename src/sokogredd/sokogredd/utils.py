import math as m
from random import randint
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid


def distance(x1, y1, x2, y2):
    return m.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def mapToRealPoint(occGrid, w, h):
    x = __mapToRealX(occGrid, w)
    y = __mapToRealY(occGrid, h)
    return Point(x=x, y=y)


def findWBorders(occGrid):
    return __findOccupancyGridBorders(occGrid, lambda w, h: (h, w))
  

def findHBorders(occGrid):
    return __findOccupancyGridBorders(occGrid, lambda w, h: (w, h))


def logg(loggerName, *args):
    if len(args) == 1:
        args = args[0]
    s = '%s:\n%s\n'%(loggerName, str(args))
    return s.replace('\n', '\n'+' '*10)

def convertNamespaceID(tb_id, typeof):
    if isinstance(tb_id, typeof):
        return tb_id
    if typeof is int:
        return int(tb_id.strip()[-1])
    return 'tb3_'+str(tb_id)

def pl(matrix, a='*', b='#'):
    return '\n'.join([' '.join([b if item==1 else a for item in row]) for row in matrix])
 

# non-public 

def __findOccupancyGridBorders(occGrid, arrangeDim):
    print('enter')
    _ = occGrid.info
    parallellN, orthagonalN = arrangeDim(_.width, _.height)
    borders = []
    count, lock, mini, maxi = 0, False, 0, 0
    for p in range(parallellN):
        oStart = __findEdge(occGrid, p, orthagonalN, lambda o: o, arrangeDim)
        oStop = __findEdge(occGrid, p, orthagonalN, lambda o: orthagonalN-o-1, arrangeDim)
        if oStart > oStop and count < 2:
            if not lock:
                count += 1
                lock = True
                if count == 1:
                    mini = p
        elif oStart < oStop:
            if lock:
                lock = False
        if count == 2:
            maxi = p
            break
        borders.append((oStart, oStop))
    return (borders, mini, maxi)

def __findEdge(occGrid, p, orthagonalLen, sidePicker, arrangeDim):
    status = 0
    for i in range(orthagonalLen):
        o = sidePicker(i)
        w, h = arrangeDim(p, o)
        cell = occGrid.data[occGrid.info.width*h + w]
        if status == cell: continue
        elif not cell: break
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

    m, n = 40, 45
    matrix = [[(1 if (0<j<3 or n-4<j<n-1) and 0<i<m-1 else 0) for j in range(n)] for i in range(m)]
    ones = [0, *[1 for i in range(n-2)], 0]
    matrix.insert(1, ones)
    matrix.insert(1, ones)
    matrix.insert(len(matrix)-1, ones)
    matrix.insert(len(matrix)-1, ones)
    for _ in range(20):
        length = random.randint(1, 20)
        i0 = random.randint(4, len(matrix)-length-1)
        j = random.choice(list(range(4, len(matrix[0])-4, 4)))
        for d in range(length):
            matrix[i0+d][j] = 1
    pl(matrix, str(0), str(1))
    occg = OccupancyGrid(data=[bit for row in matrix for bit in row])
    occg.info.width, occg.info.height = len(matrix[0]), len(matrix) 
    print(findWBorders(occg))
    print(findHBorders(occg))
    






