import rclpy
import time
from sokogredd import utils as u
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from detection.msg import Detection
from bug2_interfaces.action import Bug2
from nav_msgs.msg import OccupancyGrid

# TODO:
#   logging, unntak
#   ta bedre hensyn til mulig ikke godkjent deteksjon

DISTANCE_THRESHOLD = 1.5

class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        self.__canRun = True
        self.__mapDigested = False
        self.__fireDuo = None
        self.create_subscription(OccupancyGrid, '/map', self.__receiveMapCallback)
        self.create_subscription(Detection, '/true_detection', self.__detectionCallback)
        self.__clientTable = [ClientHandler(self, i, Bug2, 'bug2') for i in range(2)]

    def spin(self):
        self.__awaitMap()
        while not self.isDead():
            rclpy.spin_once(self)
            if not (
                self.__fireDuo is None 
                or self.__fireDuo[0].isReady() 
                or self.__fireDuo[1].isReady()
            ):
                a, b = self.__fireDuo
                a.markFireComplete(), b.markFireComplete()
            for client in self.__clientTable:
                if client.isReady():
                    client.startWorking()

    def isDead(self):
        return self.__canRun

    def __awaitMap(self):
        rclpy.spin_once(self)
        while not self.__mapDigested:
            time.sleep(0.5)
            rclpy.spin_once(self)

    def __findAssistant(self, finderClient, position):
        dist = 500.0
        x, y = position.x, position.y
        for candidate in self.__clientTable:
            if candidate.id == finderClient.id:
                continue
            tbx, tby = candidate.latestPosition.x, candidate.latestPosition.y
            tbDist = u.distance(x, y, tbx, tby)
            if tbDist < dist:
                theChosenOne = finderClient
        return theChosenOne 

    def __defineGoalGenerator(self, i, ogMsg, wBorders, hBorders):
        def generator():
            h = random.randint(0, len(wBorders) - 1)
            wmin, wmax = wBorders[h]
            delta = (wmax - wmin)//len(self.tbTable)
            w = random.randint(wmin + i*delta, wmin + (i+1)*delta)
            hmin, hmax = hBorders[w]
            if not (hmin < h < hmax):
                h = random.choice((hmin, hmax))
            return u.mapToRealPoint(ogMsg, w, h)
        return generator

    def __receiveMapCallback(self, ogMsg):
        if self.__mapDigested:
            return
        if len(self.__clientTable) > 2:
            raise Exception('that would need an update')
        wBorders = u.findWBorders(ogMsg)
        hBorders = u.findHBorders(ogMsg)
        for id in range(len(self.__clientTable)):
            self.__clientTable[id].setGoalGenerator(self.__defineGoalGenerator(id, ogMsg, wBorders, hBorders))
        self.__mapDigested = True

    def __detectionCallback(self, dMsg):
        client = self.__clientTabel[dMsg.tb_id]
        if not dMsg.accepted:
            client.noteUnaccepted(dMsg.marker_id, dMsg.pose)
            return
        if dMsg.marker_id != 4:
            client.noteAccepted(dMsg.marker_id)
            return
        assistant = self.__findAssistant(client, dMsg.pose.position)
        client.assignBigFire(dMsg.pose)
        assistant.assignBigFire(dMsg.pose)
        self.__fireDuo = (client, assistant)


class ClientHandler:

    def __init__(self, node, id, actionType, actionName, startingPosition=Point(x=0.0, y=0.0, z=0.0)):
        self.latestPosition = startingPosition
        self.id = id 
        self.__createRandomPoint = None
        self.__locked = False
        self.__greenFlag = False
        self.__markerStack = list()
        self.__pivotMarkerPose = None
        self.__flameMarkerPose = None
        self.__goalHandle, self.__goal = None, None
        self.__client = ActionClient(node, actionType, actionName)

    def startWorking(self):
        self.__lock()
        self.__ensureCanceled()
        self.__greenFlag = False
        if not self.__flameMarkerPose is None:
            goal = self.__flameMarkerPose.position
        elif not self.__pivotMarkerPose is None:
            goal = self.__pivotMarkerPose.position
        elif len(self.__markerStack) > 0:
            self.__pivotMarkerPose = self.__markerStack[-1][0]
            goal = self.__pivotMarkerPose.position
        else:
            goal = self.__createRandomPoint()
        self.__goal = goal
        request = Bug2.Request(taregt_position=goal)
        self.__client.wait_for_server()
        future = self.__client.send_goal_async(request, self.__feedCallBack)
        future.add_done_callback(self.__responseCallback)
        self.__unlock()

    def isReady(self): 
        return self.__greenFlag

    def isByFire(self):
        return self.byFire

    def setGoalGenerator(self, generator): 
        self.__createRandomPoint = generator

    def assignBigFire(self, markerPose):
        self.__lock()
        self.__flameMarkerPose = markerPose
        self.__greenFlag = True
        self.__unlock()

    def markFireComlete(self):
        self.__lock()
        self.__flameMarkerPose = None
        self.__greenFlag = True
        self.__unlock()

    def noteUnaccepted(self, markerID, markerPose):
        self.__lock()
        self.__markerStack.append((markerID, markerPose))
        self.__pivotMarkerPose = None
        self.__greenFlag = True
        self.__unlock()

    def noteAccepted(self, markerID):
        self.__lock()
        if self.__markerStack[-1][0] == markerID:
            self.__pivotMarkerPose = None
        for i in range(len(self.__markerStack)):
            if self.__markerStack[i][0] == markerID:
                self.__markerStack.pop(i)
                break
        self.__unlock()

    def __ensureCanceled(self):
        if not self.__locked:
            raise Exception('attempt to cancel goal outside locked section')
        if self.__goalHandle is None:
            return 
        self.__goalHandle.cancel_goal_async()
        self.__goalHandle = None
        
    def __responseCallback(self, node, ghFuture):
        self.__lock()
        self.__goalHandle = ghFuture.result()
        if not self.__goalHandle.accepted:
            self.__goalHandle, self.__goal = None, None
            self.__greenFlag = True
            return
        self.__goalHandle.get_result_async().add_done_callback(self.__resultCallback)
        self.__unlock()

    def __resultCallback(self, node, resultFuture):
        p = resultFuture.result().result.base_position
        self.__lock()
        distance = u.distance(p.x, p.y, self.__goal.position.x, self.__goal.position.y)
        self.__goalHandle, self.__goal = None, None
        if not self.__flameMarkerPose is None and distance < DISTANCE_THRESHOLD:
            self.__greenFlag = False
            return
        if not self.__pivotMarkerPose is None:
            self.__markerStack.pop()
        self.__greenFlag = True
        self.__unlock()
        
    def __feedCallBack(self, node, feedbackMsg):
        self.latestPosition = feedbackMsg.current_position

    def __lock(self):
        while self.__isLocked:
            time.sleep(0.01)
        self.__isLocked = True

    def __unlock(self):
        self.__isLocked = False

def main(args=None):
    rclpy.init(args=args)
    controller = Coordinator()
    rclpy.spin(controller)   
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()