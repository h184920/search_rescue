
import rclpy
import time
import random
import math
import rclpy.action.client as act
from sokogredd import utils as u
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
from rclpy.logging import LoggingSeverity

FIRE_DISTANCE_THRESHHOLD = 1.0

LOGGLEVEL = LoggingSeverity.INFO

class ActionHandler(Node):

    def __init__(self):
        super().__init__('action_handler')
        self.declare_parameter('namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.unacceptedMarkers = list()
        self.greenFlag = False
        self.firePose, self.fireSent = None, False
        self.currentGoal, self.requestCount = None, 0
        self.latestPosition = Point(x=0.0, y=0.0, z=0.0)
        self.create_subscription(JustBool, '/run_flag', self.flagCallback, 10)
        self.create_subscription(Detection, 'new_detection', self.detectionCallback, 10)
        self.randomTargetSrv = self.create_client(GoalPose, 'goal_assignment')
        self.teamupActCli = ActionClient(self, Teamup, 'teamup')
        self.bug2Action = ActionClient(self, Bug2, 'bug2')
        self.waitForServer(self.randomTargetSrv.wait_for_service, 'goal_assignment')
        self.waitForServer(self.teamupActCli.wait_for_server, 'teamup')
        self.waitForServer(self.bug2Action.wait_for_server, 'bug2')
        self.logg('Initialized\n'.upper()*5)
        self.get_logger().set_level(LOGGLEVEL)

    def updateGoal(self, targetPose=None, skipStack=False):
        if self.currentGoal is not None:
            self.cancelGoal()
            return
        if not self.greenFlag:
            return
        logg = 'about to send new goal-request'
        if self.firePose is not None:
            logg += ' with marker4 event ongoing'
            if self.fireSent:
                self.logg(logg + '\n returning, since a request has already been sent')
                return
            self.logg(logg + '\n sending request to bug2 for changing course')
            self.fireSent = True
            goal = self.firePose.position
        elif len(self.unacceptedMarkers) > 0 and not skipStack:
            logg += '\n has unaccepted markers pending, handling that'
            goal = self.unacceptedMarkers[-1].pose.position
        elif targetPose is not None:
            logg += '\n going for the random target'
            goal = targetPose.position
        else:
            self.randomTargetSrv.call_async(
                GoalPose.Request(tb_id=self.namespace)).add_done_callback(
                    self.randomTargetCallback)
            self.logg(logg + '\n will first request a random destination')
            return
        self.logg(logg)
        self.requestCount += 1
        self.bug2Action.send_goal_async(
            Bug2.Goal(target_position=goal), self.bug2FeedbackCallback).add_done_callback(self.bug2ResponseCallback)

    def cancelGoalCallback(self, _):
        # bug2 should always cancel the goal
        self.updateGoal()

    def cancelGoal(self, goalHandle=None):
        if goalHandle is None:
            goalHandle = self.currentGoal
            self.currentGoal = None
        goalHandle.cancel_goal_async().add_done_callback(self.cancelGoalCallback)

    def bug2ResponseCallback(self, fGoalHandle):
        s = 'received response from coordinator'
        outdated = self.requestCount > 1
        assert self.requestCount > 0
        self.requestCount -= 1
        goalHandle = fGoalHandle.result()
        if not goalHandle.accepted:
            s+='goal state was not accepted'
            if not outdated:
                self.logg(s + ',\n but it was the most recent sent. Sending new goal')
                self.updateGoal()
                return
            self.logg(s)
            return
        if outdated or not self.greenFlag:
            self.logg('\n   goal was outdated.. or self does not have green flag. Canceling')
            self.cancelGoal(goalHandle)
            return
        self.logg(s+ ',\n   proceeding to wait for result')
        self.currentGoal = goalHandle
        goalHandle.get_result_async().add_done_callback(self.bug2ResultCB)
        

    def randomTargetCallback(self, futureTargetPose):
        response = futureTargetPose.result()
        self.logg('received response: {}'.format(response))
        self.updateGoal(targetPose=response.goal_pose)

    def bug2ResultCB(self, futureResult):
        logg = ' reached result callback for goal'
        _ = futureResult.result()
        if _.status in (act.GoalStatus.STATUS_CANCELED, act.GoalStatus.STATUS_CANCELING):
            self.logg(logg + '\n goal in status canceling or canceled, trusting other callback to restart')
            return
        stackSize = len(self.unacceptedMarkers)
        target=None
        if self.fireSent:
            logg += '. Arrived by aruco-marker 4'
            p = _.result.base_position
            p0 = self.firePose.position
            if math.sqrt((p0.x - p.x)**2 + (p0.y - p.y)**2) < FIRE_DISTANCE_THRESHHOLD:
                self.logg('sending request for teamup with other robot')
                self.teamupActCli.send_goal_async(
                    Teamup.Goal(), self.teamFeedbackCallback).add_done_callback(
                        self.teamupResponseCallback)
                return
            self.logg('not close enough to marker, resending request')
            target = self.firePose
        elif stackSize > 1:
            logg += 'goal should be previously undetected marker. Moving it to stack bottom.'
            self.unacceptedMarkers.insert(0, self.unacceptedMarkers.pop())
        self.logg(logg)
        self.updateGoal(targetPose=target, skipStack=stackSize==1)

    def detectionCallback(self, dMsg):
        logg = 'received detection message'
        if not dMsg.accepted:
            logg += ' as unaccepted by server'
            self.unacceptedMarkers.append(dMsg)
        else: 
            logg += ' accepted by server'
            for i in range(len(self.unacceptedMarkers)):
                if self.unacceptedMarkers[i].marker_id == dMsg.marker_id:
                    self.unacceptedMarkers.pop(i)
                    logg += '\n removed it from stack'
            self.logg(logg)
            if dMsg.marker_id == 4:
                logg += ' marker was marker 4'
                self.firePose = dMsg.pose
                self.fireSent = False
        self.logg(logg)
        self.updateGoal()

    def teamupResponseCallback(self, futureTeamup):
        logg = 'received teamup response from coordinator'
        teamupHandle = futureTeamup.result()
        if not teamupHandle.accepted:
            self.logg(logg + '.\n   Request for teamup was unaccepted by coordinator\n'.upper()*3)
            self.firePose = None
            self.fireSent = False
            self.updateGoal()
        self.logg(logg + '\n  teamed up')
        futureVoid = self.teamupActCli.get_result_async()
        futureVoid.add_done_callback(self.teamupResultCallback)

    def teamFeedbackCallback(self, msg):
        return

    def teamupResultCallback(self, futureVoid):
        s = 'teamup result arrived'
        _ = futureVoid.result()
        if _.status == act.GoalStatus.STATUS_SUCCEEDED:
            s += '\nmarker 4 successfully handled'
            self.firePose = None
            self.fireSent = False
        elif not _.status == act.GoalStatus.STATUS_SUCCEEDED:
            s = '\nunsuccessful. Trying again'
        self.logg(s)
        self.updateGoal()

    def bug2FeedbackCallback(self, msg):
        if msg.feedback.current_position is None:
            return
        self.latestPosition = msg.feedback.current_position

    def flagCallback(self, fMsg):
        if fMsg.green and self.greenFlag or not (fMsg.green or self.greenFlag):
            return
        self.greenFlag = fMsg.green
        self.updateGoal()

    def waitForServer(self, callable, name):
        while not callable(timeout_sec=0.5):
            self.logg('waiting for %s'%(name))
        self.logg('%s is ready'%(name))

    def logg(self, obj):
        self.get_logger().info(u.logg(
            self.namespace+'.action_handler', str(obj).upper()))

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


            
    