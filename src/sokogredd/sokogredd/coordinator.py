import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from detection.msg import Detection
from bug2_interfaces.srv import Bug2
from nav_msgs.msg import OccupancyGrid

from time import sleep

# tar ikke vare på punkt, men mulig
# bruker ikke feedback

class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        self.canRun = True
        self.grid = None
        self.fireEvent = None
        self.create_subscription(OccupancyGrid, '/map', self.receiveGrid)
        self.create_subscription(Detection, '/true_detection', self.detectionSubCallback)
        self.tbTable = {i : TbSpace(i, self, Bug2, 'bug2') for i in range(2)}

    def receiveGrid(self, ogMsg):
        self.grid = ogMsg

    def parseMap(self):
        while self.grid is None:
            sleep(0.5)
        # todo

    def spin(self):
        self.parseMap()
        while not self.isDead():
            rclpy.spin_once(self)
            # todo

    def detectionSubCallback(self, dMsg):
        pass

    def makeFeedcallback(self, tbID, point):
        return 

    def makeResponseCallback(self, tbID, future):
        pass

    def makeResultCallback(self, tbID, futureGH):
        pass
    
    def kill(self):
        self.canRun = False

    def isDead(self):
        return self.canRun

    def LOGG(self, obj):
        print(obj)

class TbSpace:

    def __init__(self, id, superduper, actionType, actionName):
        self.id = id
        self.busy = False
        self.client = ActionClient(superduper, actionType, actionName)
        self.pointGenerator = None

    def assignGoal(self, explicit=None):
        if explicit is None and self.pointGenerator is None:
            raise Exception('%s could not assign action a goal since %s had not been '+
            'provided a generator function and not explicit destination was provided'%self.__class__.__name__)

    def justStop(self):
        pass     


