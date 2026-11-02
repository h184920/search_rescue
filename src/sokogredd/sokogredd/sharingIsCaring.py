

class MutableVector:

    def __init__(self):
        self.__x0, self.__y0, self.__x, self.__y = 0,0,0,0
        self.__dx, self.__dy = 0,0
        self.__len = 0.0

    def __mul__(self, c):
        return self.__dx*c, self.__dy*c

    def setOrigin(self, point: tuple[float, float]):
        self.__x0, self.__y0 = point
        self.__update()
        return self

    def setEnd(self, point: tuple[float, float]):
        self.__x, self.__y = point
        self.__update()
        return self

    def __update(self):
        self.__dx, self.__dy = self.__x - self.__x0, self.__y - self.__y0
        self.__len = m.sqrt(self.__dx**2 + self.__dy**2)

    def len(self):
        return self.__len