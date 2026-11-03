from sharingIsCaring import *

class TreeMaker(dict):

    def __init__(self, points: list[tuple[float, float]], grid: list[list[float]]):
        super().__init__(**{i: (points[i], None) for i in range(len(points))})
        self.__build(grid)

    def __build(self, grid):
        ids = self.__sortedIds()
        parentID = 0
        while len(ids) > 0:
            childID = self.__findAndPopChild(parentID, ids, grid)
            self[parentID][1] = childID
            parentID = childID

    def __sortedIds(self):
        ids = [id for id in self.keys()[1:]]
        withRootAsOrigin = MutableVector(origin=self[0][0])
        for nextUp in range(1, len(ids)):
            shuffleIndex = nextUp
            while shuffleIndex > 0:
                shufflePoint = self[ids[shuffleIndex]][0]
                comparisonPoint = self[ids[shuffleIndex - 1]][0]
                distanceToShuffle = withRootAsOrigin.toEnd(shufflePoint).len()
                distanceToComparison = withRootAsOrigin.toEnd(comparisonPoint).len()
                if (distanceToComparison <= distanceToShuffle):
                    break
                self.__swap(shuffleIndex-1, shuffleIndex, ids)
                shuffleIndex -= 1

    def __findAndPopChild(self, parentID, ids, grid):
        parent = self[parentID][0]
        for i in range(len(ids)):
            childID = ids[i]
            if findBlockage(parent, self[childID][0], grid) is None:
                break
        return ids.pop(i)

    def __swap(self, i, j, array):
        temp = array[i]
        array[i] = array[j]
        array[j] = temp













    
    


if __name__ == '__main__':
    pass