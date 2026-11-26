import math as m
import random
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid


def distance(x1, y1, x2, y2):
    return m.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def mapToRealPoint(occGrid, w, h):
    x = __mapToRealX(occGrid, w)
    y = __mapToRealY(occGrid, h)
    return Point(x=x, y=y)

def equalPoints(a, b):
    return a.x == b.x and a.y == b.y

def findWBorders(occGrid):
    return __findOccupancyGridBorders(occGrid, lambda w, h: (h, w))
  

def findHBorders(occGrid):
    return __findOccupancyGridBorders(occGrid, lambda w, h: (w, h))

def findMinMax(borders):
    n, x = 0, 0
    length = len(borders)
    for i in range(length):
        if borders[i][0] < borders[i][1]:
            n = i
            break
    for i in range(length):
        if borders[length - i - 1][0] < borders[length - i - 1][1]:
            x = length - i - 1
            break
    return n, x

def convertNamespaceID(tb_id, typeof):
    if isinstance(tb_id, typeof):
        return tb_id
    if typeof is int:
        return int(tb_id.strip()[-1])
    return 'tb3_'+str(tb_id)

def xand(a, b):
    return a and b or not (a or b)

def xor(a, b):
    return not xand(a, b)

def logg(loggerName, *args):
    if len(args) == 1:
        args = args[0]
    s = '%s:\n%s\n'%(loggerName, str(args))
    return s.replace('\n', '\n'+' '*10)

class PointGenerator:

    def __init__(self, occGrid):
        self.defaultPoint = (0, 0)
        self.wBorders = findWBorders(occGrid)
        self.hBorders = findHBorders(occGrid)
        self.wmin, self.wmax = findMinMax(self.hBorders) # yes, they are switched
        self.hmin, self.hmax = findMinMax(self.wBorders) # yes, they are switched
        assert self.wmin < self.wmax
        assert self.hmin < self.hmax
        self.dw = self.wmax - self.wmin
        self.dh = self.hmax - self.hmin

    def generateGoal(self, i):
        assert -1 < i < 2
        point = self.defaultPoint
        attempts = 0
        trueattempts = 0
        while attempts < 50:
            trueattempts += 1
            w = random.randint(self.wmin + (self.dw//2)*i, self.wmin + (self.dw//2)*(i+1))
            h = random.randint(self.hmin, self.hmax)
            hmin, hmax = self.hBorders[w]
            wmin, wmax = self.wBorders[h]
            if not wmin < w < wmax:
                pass
            elif hmin < h < hmax:
                point = (w, h)
                attempts = 50
            attempts += 1
        return point
 

# non-public 

def __findOccupancyGridBorders(occGrid, arrangeDim):
    _ = occGrid.info
    parallellN, orthagonalN = arrangeDim(_.width, _.height)
    borders = []
    for p in range(parallellN):
        oStart = __findEdge(occGrid, p, orthagonalN, lambda o: o, arrangeDim)
        oStop = __findEdge(occGrid, p, orthagonalN, lambda o: orthagonalN-o-1, arrangeDim)
        borders.append((oStart, oStop))
    return borders

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

    def pl(matrix, a='*', b='#'):
        return '\n'.join([''.join([b if item==1 else a for item in row]) for row in matrix])

    k, n = 40, 60
    padding = 3
    tykk = 2
    matrix = [
        [(1 if (padding-1<j<padding+tykk or n-padding-tykk-2<j<n-padding) and padding-2<i<k-padding-2 else 0) for j in range(n)] 
        for i in range(k)
        ]
    for i in range(padding, padding+tykk):
        for j in range(padding+tykk-1, n-padding-tykk):
            matrix[i][j] = 1
    print(pl(matrix))
    for _ in range(10):
        length = random.randint(1, 10)
        i0 = random.randint(4, len(matrix)-length-3)
        j = random.choice(list(range(4, len(matrix[0])-4, 4)))
        for d in range(length):
            matrix[i0+d][j] = 1
    occg = OccupancyGrid(data=[bit for row in matrix for bit in row])
    occg.info.width, occg.info.height = len(matrix[0]), len(matrix) 
    gen = PointGenerator(occg)
    strMatrix = pl(matrix, '+', '.')
    print(strMatrix)
    palet = [list(row) for row in strMatrix.strip().split('\n')]
    for _ in range(50):
        w, h = gen.generateGoal(0)
        palet[w][h] = 'A'
        w, h = gen.generateGoal(1)
        palet[w][h] = 'B'
    print()
    print('\n'.join([''.join(row) for row in palet]))






