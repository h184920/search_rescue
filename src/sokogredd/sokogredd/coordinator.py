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

DISTANCE_THRESHOLD = 1.5

class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        self.__name = 'coordinator'
        self.__canRun = True
        self.__mapDigested = False
        self.__fireDuo = None
        self.create_subscription(OccupancyGrid, '/map', self.__receiveMapCallback, 10)
        self.create_subscription(Detection, '/true_detection', self.__detectionCallback, 10)
        self.__clientTable = [ClientHandler(self, i, Bug2, 'bug2') for i in range(2)]
        self.logg('INITIALIZED')

    def logg(self, obj):
        u.logg(self.__name, obj)

    def spin(self):
        self.__awaitMap()
        self.logg('MAP HAS BEEN PARSED. STARTING')
        while not self.__isDead():
            rclpy.spin_once(self)
            if not (self.__fireDuo is None 
                    or self.__fireDuo[0].isReady() 
                    or self.__fireDuo[1].isReady()):
                self.logg('big fire determined complete')
                a, b = self.__fireDuo
                a.markFireComplete(), b.markFireComplete()
                self.__fireDuo = None
            for client in self.__clientTable:
                if not client.isReady():
                    continue
                self.logg('client_%s was ready for new task. Starting client_'%(client.id))
                client.startWorking()
            time.sleep(1.0)
        self.logg('IS DEAD')

    def kill(self):
        self.__canRun = False
        for client in self.__clientTable:
            client.kill()
        
    def __awaitMap(self):
        rclpy.spin_once(self)
        while not self.__mapDigested:
            time.sleep(1.0)
            self.logg('..awaiting map')
            rclpy.spin_once(self)

    def __isDead(self):
        return self.__canRun

    def __receiveMapCallback(self, ogMsg):
        if self.__mapDigested:
            return
        if len(self.__clientTable) > 2:
            raise Exception('that would need an update')
        wBorders = u.findWBorders(ogMsg)
        hBorders = u.findHBorders(ogMsg)
        self.logg('map digested with len(wBorders)=%s, len(hBorders)=%s\n'+
                  'samples [1000000::]: \n %s\n%s'%(
                      len(wBorders), len(hBorders), wBorders[1000000::], hBorders[1000000::])
                )
        for id in range(len(self.__clientTable)):
            self.__clientTable[id].setGoalGenerator(
                self.__defineGoalGenerator(id, ogMsg, wBorders, hBorders)
            )
        self.__mapDigested = True

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

    def __detectionCallback(self, dMsg):
        s = 'received new detection for marker with id %s. Detected by bot_%s at %s'%(
            dMsg.marker_id, dMsg.tb_id, dMsg.pose.position
            )
        client = self.__clientTabel[dMsg.tb_id]
        if not dMsg.accepted:
            self.logg(s+'\n ! detection not accepted by server')
            client.noteUnaccepted(dMsg.marker_id, dMsg.pose)
            return
        if dMsg.marker_id != 4:
            self.logg(s+'.. accepted by server')
            client.noteAccepted(dMsg.marker_id)
            return
        self.logg(s+'\n markerID=4')
        assistant = self.__findAssistant(client, dMsg.pose.position)
        client.assignBigFire(dMsg.pose)
        assistant.assignBigFire(dMsg.pose)
        self.__fireDuo = (client, assistant)
        self.logg('created duo (%s, %s)'%(client.id(), assistant.id()))

    def __findAssistant(self, finderClient, position):
        self.logg('finding assistance bot for %'%(finderClient.id()))
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


class ClientHandler:

    def __init__(self, node, id, actionType, actionName, startingPosition=Point(x=0.0, y=0.0, z=0.0)):
        self.latestPosition = startingPosition
        self.__id = id 
        self.__name = 'client_'+str(id)
        self.__createRandomPoint = None
        self.__locked = False
        self.__greenFlag = False
        self.__markerStack = list()
        self.__pivotMarkerPose = None
        self.__flameMarkerPose = None
        self.__goalHandle, self.__goal = None, None
        self.__client = ActionClient(node, actionType, actionName)
        self.logg('INITIALIZED')

    def locking(m):
        def method (fles, *args, **kwargs):
            fles.__lock()
            result = m(fles, *args, **kwargs)
            fles.__unlock()
            return result
        return method

    def id(self):
        return self.id

    def logg(self, obj):
        u.logg(self.__name, obj)

    @locking
    def kill(self):
        self.__ensureCanceled()

    @locking
    def startWorking(self):
        s='is to send new goal\nwill first cancel current goal if any'
        self.__ensureCanceled()
        self.__greenFlag = False
        if not self.__flameMarkerPose is None:
            goal = self.__flameMarkerPose.position
            s+='\nfire ongoing at detected position'
        elif not self.__pivotMarkerPose is None:
            goal = self.__pivotMarkerPose.position
            s+='\nattempting to capture unaccepted marker'
        elif len(self.__markerStack) > 0:
            self.__pivotMarkerPose = self.__markerStack[-1][0]
            goal = self.__pivotMarkerPose.position
            s+='\nhad undetected marker registered'
        else:
            goal = self.__createRandomPoint()
            s+='\nfree to search, generating random'
        self.logg(s+'\nRequesting action server\ngoal=(%s, %s)'%(goal.x, goal.y))
        self.__goal = goal
        request = Bug2.Request(taregt_position=goal)
        self.__client.wait_for_server()
        future = self.__client.send_goal_async(request, self.__feedCallBack)
        future.add_done_callback(self.__responseCallback)

    def isReady(self): 
        return self.__greenFlag

    def setGoalGenerator(self, generator): 
        self.__createRandomPoint = generator

    @locking
    def assignBigFire(self, markerPose):
        self.logg('is assigned responsibility of fire event\n '+
                  'pose.position=%s'%(markerPose.position))
        self.__flameMarkerPose = markerPose
        self.__greenFlag = True

    @locking
    def markFireComlete(self):
        self.logg('received notice of fire event completion')
        self.__flameMarkerPose = None
        self.__greenFlag = True

    @locking
    def noteUnaccepted(self, markerID, markerPose):
        self.logg(('received notice of unaccepted marker\nid=%s, pose.position='+
                  '%s')%(markerID, markerPose.position))
        self.__markerStack.append((markerID, markerPose))
        self.__pivotMarkerPose = None
        if self.__flameMarkerPose is None:
            self.__greenFlag = True

    @locking
    def noteAccepted(self, markerID):
        s='received notice of accepted marker\n id=%s'%(markerID)
        if self.__markerStack[-1][0] == markerID:
            self.__pivotMarkerPose = None
        for i in range(len(self.__markerStack)):
            if self.__markerStack[i][0] == markerID:
                self.logg(s+'\nsuccesfully found in marker stack')
                self.__markerStack.pop(i)
                break
        self.logg(s+'\n! but marker was not found in stack')

    def __ensureCanceled(self):
        if not self.__locked:
            raise Exception('attempt to cancel goal outside locked section')
        if self.__goalHandle is None:
            return 
        self.__goalHandle.cancel_goal_async()
        self.__goalHandle = None

    @locking
    def __responseCallback(self, node, ghFuture):
        s='response callback called'
        self.__goalHandle = ghFuture.result()
        if not self.__goalHandle.accepted:
            self.logg(s+'\n .. the request was not accepted')
            self.__goalHandle, self.__goal = None, None
            self.__greenFlag = True
            return
        self.logg(s+'\n .. the request was accepted')
        self.__goalHandle.get_result_async().add_done_callback(self.__resultCallback)

    @locking
    def __resultCallback(self, node, resultFuture):
        s='result callback called'
        p = resultFuture.result().result.base_position
        distance = u.distance(p.x, p.y, self.__goal.position.x, self.__goal.position.y)
        self.__goalHandle, self.__goal = None, None
        if not self.__flameMarkerPose is None and distance < DISTANCE_THRESHOLD:
            self.logg(s+'\n! Engaged in fire event\n .. and the computed distance from the goal '+
                      'was small enought for threshhold\n .. will wait')
            self.__greenFlag = False
            return
        if not self.__pivotMarkerPose is None:
            self.logg(s+'\nhas retried once, but without being notified about detection acceptance'+
                      '\ngiving up marker with id %s'%(self.__markerStack[-1][0]))
            self.__markerStack.pop()
        self.logg(s+'\nready for next task\ndistance from goal position was %s'%(distance))
        self.__greenFlag = True
        
    def __feedCallBack(self, node, feedbackMsg):
        self.latestPosition = feedbackMsg.current_position

    def __lock(self):
        while self.__isLocked:
            time.sleep(0.01)
        self.__isLocked =True

    def __unlock(self):
        self.__isLocked = False

def main(args=None):
    rclpy.init(args=args)
    controller = Coordinator()
    try:
        controller.spin()
    except KeyboardInterrupt:
        controller.kill()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()