import math as m

class MutableVector:

    def __init__(self, origin=(0, 0), end=(0, 0)):
        self.__x0, self.__y0 = origin
        self.__x, self.__y = origin
        self.__update()

    def __mul__(self, c):
        return self.__dx*c, self.__dy*c

    def fromOrigin(self, point: tuple[float, float]):
        self.__x0, self.__y0 = point
        self.__update()
        return self

    def toEnd(self, point: tuple[float, float]):
        self.__x, self.__y = point
        self.__update()
        return self

    def __update(self):
        self.__dx, self.__dy = self.__x - self.__x0, self.__y - self.__y0
        self.__len = m.sqrt(self.__dx**2 + self.__dy**2)

    def len(self):
        return self.__len

    def dx(self):
        return self.__dx

    def dy(self):
        return self.__dy


def findBlockage(origin: tuple[int, int], 
                 end: tuple[int, int], 
                 grid: list[list[int | bool]],
                 ) -> int:
    arrow = MutableVector(origin, end)
    step = 1/2
    for i in range(1, int(arrow.len()/step) + 1):
        i, j = arrow*step*i
        if grid[round(i)][round(j)] != 0:
            return i, j
    return None

if __name__ == '__main__':

    grid = [
        [0 for 0 in range(100)] for _ in range(100)
    ]

    def drawWall(p, q):
        i1, j1 = p
        i2, j2 = q
        






