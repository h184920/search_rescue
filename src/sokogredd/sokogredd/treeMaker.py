from sharingIsCaring import *

class TreeMaker(dict):

    def __init__(self, points: list[tuple[float, float]], grid: list[list[float]]):
        super().__init__(**{i: (points[i], None) for i in range(len(points))})
        self.__build(grid)

    def __build(self, grid):
        ids = self.__arrangement()
        parentID = 0
        while len(ids) > 0:
            childID = self.__popId(parentID, ids, grid)
            self[parentID][1] = childID
            parentID = childID

    def __arrangement(self):
        ids = [id for id in self.keys()[1:]]
        vector = MutableVector(origin=self[0][0])
        for lowest in range(len(ids) - 1):
            i = lowest
            while i > 0:
                A = self[ids[i + 1]][0]
                B = self[ids[i]][0]
                lenA = vector.toEnd(A).len()
                lenB = vector.toEnd(B).len()
                if (lenA >= lenB):
                    break
                self.__swap(i, i+1, ids)
                i += 1

    def __popId(self, parentID, ids, grid):
        parent = self[parentID][0]
        for i in range(len(ids)):
            childID = ids[i]
            if findBlockage(parent, self[childID][0], grid) is None:
                break
        ids.pop(i)

    def __swap(self, i, j, array):
        temp = array[i]
        array[i] = array[j]
        array[j] = temp













    
    


if __name__ == '__main__':
    pass