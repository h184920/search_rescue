import rclpy
import time
from sokogredd import utils as u
#import utils as u
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from detection.msg import Detection
from bug2_interfaces.action import Bug2
from nav_msgs.msg import OccupancyGrid


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
        self.__canRun = True
        self.__mapReceived = False
        self.__fireEvent = None
        self.create_subscription(OccupancyGrid, '/map', self.__receiveMapCallback, 10)
        self.create_subscription(Detection, '/true_detection', self.__detectionCallback, 10)
        self.__tbTable = [TbSpace(self, i, Bug2, 'bug2') for i in range(2)]

    def spin(self):
        self.__awaitMap()
        while not self.isDead():
            rclpy.spin_once(self)
            if not self.__fireEvent is None:
                self.__dealWithThat()
            for tb in self.__tbTable:
                if not tb.isWorking():
                    tb.startWorking()

    def isDead(self):
        return self.__canRun

    def kill(self): 
        self.__canRun = False

    def __awaitMap(self):
        rclpy.spin_once(self)
        while not self.__mapReceived:
            rclpy.spin_once(self)
            time.sleep(0.5)
            
    def __dealWithThat(self):
        finder = self.__tbTable[self.__fireEvent.tb_id]
        finder.startWorking(self.__fireEvent.position)
        assistant = self.__findAssistance(self.__fireEvent.tb_id)
        assistant.startWorking(self.__fireEvent.position)

    def __findAssistance(self, tb_id):
        dist = 500.0
        x, y = self.__fireEvent.position.x, self.__fireEvent.position.y
        for id in range(len(self.__tbTable)):
            tb = self.__tbTable[id]
            if id == tb_id:
                continue
            tbx, tby = tb.latestPosition.x, tb.latestPosition.y
            tbDist = u.distance(x, y, tbx, tby)
            if tbDist < dist:
                theChosenOne = tb
        return theChosenOne

    def defineGoalGenerator(self, i, ogMsg):
        def generator():
            h = random.randint(0, len(self.wBorders) - 1)
            wmin, wmax = self.wBorders[h]
            delta = (wmax - wmin)//len(self.tbTable)
            w = random.randint(wmin + i*delta, wmin + (i+1)*delta)
            hmin, hmax = self.hBorders[w]
            if not (hmin < h < hmax):
                h = random.choice((hmin, hmax))
            return u.mapToRealPoint(ogMsg, w, h)
        return generator

    def __receiveMapCallback(self, ogMsg):
        if self.__mapReceived:
            return
        if len(self.__tbTable) > 2:
            raise Exception('that would need an update')
        self.wBorders = u.findWBorders(ogMsg)
        self.hBorders = u.findHBorders(ogMsg)
        for id in range(len(self.__tbTable)):
            self.__tbTable[id].setGoalGenerator(self.defineGoalGenerator(id, ogMsg))
        self.canRun = True

    def __detectionCallback(self, dMsg):
        if dMsg.marker_id == 4:
            self.__fireEvent = dMsg
            self.gridCopy = list(dMsg.data)

class TbSpace:

    def __init__(self, node, id, actionType, actionName, startingPosition=Point(x=0.0, y=0.0, z=0.0)):
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
        distance = u.distance(P.x, P.y, self.goal.x, self.goal.y)
        if distance > DISTANCE_THRESHOLD - DISTANCE_THRESHOLD*0.3:
            self.startWorking(self.goal)
        self.goalHandle, self.goal = None, None
        
    def __feedCallBack(self, node, feedbackMsg):
        self.latestPosition = feedbackMsg.current_position

def main(args=None):
    rclpy.init(args=args)
    controller = Coordinator()
    rclpy.spin(controller)   
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()