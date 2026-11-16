
import rclpy
import time
from sokogredd import utils as u
import random
from uuid import UUID
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.action.server import CancelResponse
from geometry_msgs.msg import Point, Pose
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from detection.msg import Detection
from bug2_interfaces.action import Bug2
from interfjeses.msg import JustBool
from interfjeses.srv import GoalPose
from interfjeses.action import Teamup
from unique_identifier_msgs.msg import UUID


class ActionHandler(Node):

    def __init__(self):
        super().__init__('action_handler')
        self.declare_parameter('namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.__markerStack = list()
        self.__firePose, self.__fireSent = None, False
        self.__currentGoalHandle, self.__validGoalID = None, 0
        self.__latestPosition = Point(x=0.0, y=0.0, z=0.0)
        self.create_subscription(JustBool, '/run_flag', self.__flagCB, 10)
        self.create_subscription(Detection, '/new_detection', self.__detectionCB, 10)
        self.__goalClient = self.create_client(GoalPose, '/goal_assignment')
        self.__teamupClient = ActionClient(self, Teamup, 'teamup')
        self.__bug2Client = ActionClient(self, Bug2, 'bug2')
        self.logg('Initialized')

    def __flagCB(self, fMsg):
        if fMsg.green:
            self.__logg('received start signal.. starting')
            self.__startRunning()
            return
        self.__logg('recevied stop signal.. stopping')
        self.__stopRunning()

    def __detectionCB(self, dMsg):
        logg = 'received detection message'
        if not dMsg.accepted:
            logg += ' as unaccepted by server'
            self.__markerStack.append(dMsg)
        elif dMsg.marker_id == 4:
            logg += ' as accepted marker with ID=4 !!'
            self.__firePose = dMsg.pose
            self.__fireSent = False
        else: 
            logg += ' as accepted by server'
            for i in range(len(self.__markerStack)):
                if self.__markerStack[i].marker_id == dMsg.marker_id:
                    self.__markerStack.pop(i)
                    logg += '\n  removed it from stack'
                else:
                    logg += '\n  not to be found in stack'
            self.__logg(logg)
            return
        self.__logg(logg)
        self.__sendGoal()

    def __randomGoalCB(self, futurePose):
        self.__logg('response from coordinator received')
        self.__sendGoal(randomPose=futurePose.result().goal_pose)

    def __bug2ResultCB(self, future):
        logg = ' reached result callback for goal'
        target = future.result()
        if target.wasCanceled:
            self.__logg(logg+', but the goal had been canceled')
            return
        if self.__fireSent:
            self.__logg(logg + '. Supposed to be a big fire.. sending teamup action request')
            futureTeamupHandle = self.__teamupClient.send_goal_async(
                Teamup.Goal(), self.__teamFeedbackCB
                )
            futureTeamupHandle.add_done_callback(self.__teamupResponseCB)
            return 
        if len(self.__markerStack) > 0:
            logg += ' reached previously undetected marker. Popping from stack. Hopefully accepted by server'
            self.__markerStack.pop()
        self.__logg(logg)
        self.__sendGoal()

    def __teamupResultCB(self, void):
        self.__logg('fire event over. Coordinator said so')
        self.__firePose = None
        self.__fireSent = False
        self.__sendGoal()

    def __defineBug2ResponseCB(goalHandleID):
        def responseCB(oneself, futureResponse):
            logg = 'response arrived form goal request for goal(id=%s)'%goalHandleID
            goalHandle = futureResponse.result()
            if not oneself.__receiveHandle(oneself, goalHandle, goalHandleID):
                oneself.__logg(logg + 'was no longer valid. Aborting')
                return
            oneself.__logg(logg + 'was still valid. Proceeding')
            futureResult = goalHandle.get_result_async()
            futureResult.add_done_callback(oneself.__bug2ResultCB)
        return responseCB

    def __teamupResponseCB(self, futureTeamup):
        logg = 'received teamup response from coordinator'
        teamupHandle = futureTeamup.result()
        if not teamupHandle.accepted:
            self.__logg(logg + 'wierdly not accepted\n ..aborting then')
            self.__firePose = None
            self.__fireSent = False
            self.__sendGoal()
        self.__logg(logg + '\n  proceeding')
        futureVoid = self.__teamupClient.get_result_async()
        futureVoid.add_done_callback(self.__teamupResultCB)

    def __bug2FeedbackCB(self, msg):
        self.__latestPosition = msg.feedback.current_position

    def __teamFeedbackCB(self, msg):
        return

    def __cancelCB(self, future):
        cancelResponse = future.result()
        if cancelResponse == CancelResponse.REJECT:
            raise Exception('request to cancel rejected')

    def __sendGoal(self, randomPose=None):
        if not self.__isRunning():
            return
        logg = 'about to send new goal-request'
        if self.__firePose is not None:
            logg += ' with marker4 event ongoing'
            if self.__fireSent:
                self.__logg(logg + '\n returning, since a request has already been sent')
                return
            self.__logg(logg + '\n sending request to bug2 for changing course')
            self.__fireSent = True
            goal = self.__firePose.position
        elif len(self.__markerStack) > 0:
            logg += '\n has unaccepted markers pending, handling that'
            goal = self.__markerStack[-1].pose.position
        elif randomPose is not None:
            logg += '\n next stop, the random goal previously generated'
            goal = randomPose.position
        else:
            futureResult = self.__goalClient.call_async(
                GoalPose.Request(tb_id=self.namespace)
                )
            futureResult.add_done_callback(self.__randomGoalCB)
            self.__logg(logg + '\n will first request a random destination')
            return
        self.__logg(logg)
        id = self.__validGoalID + 1
        self.__setGoalTo(id=id)
        futureGoalHandle = self.__bug2Client.send_goal_async(
            Bug2.Goal(target_position=goal),
            self.__bug2FeedbackCB)
        futureGoalHandle.add_done_callback(self.__defineBug2ResponseCB(id))

    def __receiveHandle(self, goalHandle, id):
        if not goalHandle.accepted:
            return False
        if not id == self.__validGoalID:
            self.__cancelGoal(goalHandle)
            return False
        self.__currentGoalHandle = goalHandle
        self.__validGoalID = 0
        return True

    def __setGoalTo(self, id=0):
        if self.__currentGoalHandle is not None:
            self.__cancelGoal(self.__currentGoalHandle)
            self.__currentGoalHandle = None
        self.__validGoalID = id

    def __cancelGoal(self, goalHandle):
        future = goalHandle.cancel_goal_async()
        future.add_done_callback(self.__cancelCB)

    def __startRunning(self):
        self.__logg('starting execution')
        if self.__isRunning():
            return
        self.__validGoalID = 0
        self.__sendGoal()

    def __stopRunning(self):
        self.__logg('stopping execution')
        if not self.__isRunning():
            return
        self.__setGoalTo(id=-1)

    def __isRunning(self):
        return self.__validGoalID > -1

    def __logg(self, obj):
        self.get_logger().info(u.logg(
            self.namespace+'.action_handler', obj)
            )

    # Single callback group. Not needed.

    # def locking(m):
    #     def method(fles, *args, **kwargs):
    #         fles.__lock()
    #         result = m(fles, *args, **kwargs)
    #         fles.__unlock()
    #         return result
    #     return method

    # def __lock(self):
    #     while self.__isLocked:
    #         time.sleep(0.01)
    #     self.__isLocked =True

    # def __unlock(self):
    #     self.__isLocked = False

def main(args=None):
    rclpy.init(args=args)
    controller = ActionHandler()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


            
    