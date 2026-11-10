import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point
from detection.msg import Detection
from bug2_interfaces.srv import Bug2
from nav_msgs.msg import OccupancyGrid

from time import sleep

import sharingIsCaring as sic

# note that
#   tar ikke vare på punkt, men mulig
#   bruker ikke feedback
# TODELO:
#   TODO logging
#   TODO startposisjoner

DISTANCE_THRESHOLD = 1.5

class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        self.canRun = True
        self.grid = None
        self.boundriesX, self.boundriesY = None
        self.fireEvent = None
        self.create_subscription(OccupancyGrid, '/map', self.receiveMap)
        self.create_subscription(Detection, '/true_detection', self.detectionCallback)
        self.tbTable = {i : TbSpace(i, self, Bug2, 'bug2') for i in range(2)}

    def spin(self):
        self.parseMap()
        while not self.isDead():
            rclpy.spin_once(self)
            if not self.fireEvent is None:
                self.dealWithThat()
            for tb in self.tbTable.values():
                if not tb.isWorking():
                    tb.startWorking()

    def kill(self): 
        self.canRun = False

    def isDead(self):
        return self.canRun

    def detectionCallback(self, dMsg):
        if dMsg.marker_id == 4:
            self.fireEvent = dMsg
            self.gridCopy = list(dMsg.data)

    def receiveMap(self, ogMsg):
        self.grid = ogMsg

    # msg .data, .info: .origion.position, .width, height, .resolution

    def parseMap(self):
        while self.grid is None:
            sleep(0.5)
        X, Y = self.grid.info.width, self.grid.info.height
        self.boundriesX = []
        self.boundriesY = []
        for x in range(X):
            bottom = top = False
            for y in range(Y):
                ybottom = X*(X-x-1) + y
                if bottom and self.grid.data[ybottom]
                if self.grid.data[ybottom] >= 1:
                    bottom = True
                    continue
            for y in range(Y):
                ytop = X*()








        

    def dealWithThat(self):
        finder = self.tbTable[self.fireEvent.tb_id]
        finder.startWorking(self.fireEvent.position)
        assistant = self.findAssistance(self.fireEvent.tb_id)
        assistant.startWorking(self.fireEvent.position)

    def findAssistance(self, tb_id):
        dist = 500.0
        x, y = self.fireEvent.position.x, self.fireEvent.position.y
        for id, tb in self.tbTable.items():
            if id == tb_id:
                continue
            tbx, tby = tb.latestPosition.x, tb.latestPosition.y
            tbDist = sic.distance(x, y, tbx, tby)
            if tbDist < dist:
                theChosenOne = tb
        return theChosenOne

    def LOGG(self, obj):
        print(obj)

class TbSpace:

    def __init__(self, id, node, actionType, actionName, startingPosition):
        self.latestPosition = startingPosition
        self.id = id
        self.goal_handle, self.goal = None, None
        self.client = ActionClient(node, actionType, actionName)
        self.generateGoal = None

    def setGoalGenerator(self, generator): 
        self.generateGoal = generator

    def startWorking(self, explicitGoal=None):
        if explicitGoal is None and self.generateGoal is None:
            raise Exception('%s could not assign action a goal since %s had not been '+
            'provided a generator function and not explicit destination was provided'%self.__class__.__name__)
        self.stopWorking()
        self.goal = explicitGoal
        if self.goal is None:
            self.goal = self.generateGoal()
        self.client.wait_for_server()
        future = self.client.send_goal_async(self.goal, self.__feedCallBack)
        future.add_done_callback(self.__responseCallback)

    def stopWorking(self):
        if self.goal_handle is None:
            self.goal = None
            return
        if not self.goal_handle.cancel_goal():
            raise Exception('navigator refused to cancel goal')
        self.goal_handle, self.goal = None, None

    def isWorking(self):
        return not self.goal is None 

    def __responseCallback(self, node, ghFuture):
        self.goal_handle = ghFuture.result()
        if not self.isWorking() or not self.goal_handle.accepted:
            return
        self.goal_handle.get_result_async().add_done_callback(self.__resultCallback)

    def __resultCallback(self, node, resultFuture):
        P = resultFuture.result().result.base_position
        distance = sic.distance(P.x, P.y, self.goal.x, self.goal.y)
        if distance > DISTANCE_THRESHOLD - DISTANCE_THRESHOLD*0.3:
            self.startWorking(self.goal)
        self.goalHandle, self.goal = None, None
        
        
    def __feedCallBack(self, node, feedbackMsg):
        self.latestPosition = feedbackMsg.current_position


