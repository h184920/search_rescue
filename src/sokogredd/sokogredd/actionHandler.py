import rclpy
import rclpy.action.client as act
import time
import math
import random
import sokogredd.utils as u
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from detection.msg import Detection
from interfjeses.msg import JustBool
from interfjeses.srv import GoalPose
from bug2_interfaces.action import Bug2
from interfjeses.action import Teamup

FIRE_DISTANCE_THRESHHOLD = 1.5
LOGGLEVEL = LoggingSeverity.INFO
 
class ActionHandler(Node):

    def __init__(self):
        super().__init__('action_handler')
        self.declare_parameter('namespace')
        self.id = self.get_parameter('namespace').get_parameter_value().string_value
        self.uncanceled, self.sendCount = None, 0
        self.greenFlag = False
        self.lastSentPose = None
        self.markerStack = list()
        self.aruco4Pose = None
        self.latestPosition = Point(x=0.0, y=0.0, z=0.0)
        self.create_subscription(JustBool, '/run_flag', self.flagCB, 10)
        self.create_subscription(Detection, 'new_detection', self.detectionCB, 10)
        self.randomTargetSrv = self.create_client(GoalPose, '/goal_assignment')
        self.teamupActCli = act.ActionClient(self, Teamup, '/teamup')
        self.bug2Action = act.ActionClient(self, Bug2, 'navigate')
        self.helpTimer = self.create_timer(15.0, self.restartUnactiveCB)
        self.waitForServer(self.randomTargetSrv.wait_for_service, '/goal_assignment')
        self.waitForServer(self.teamupActCli.wait_for_server, '/teamup')
        self.waitForServer(self.bug2Action.wait_for_server, 'navigate')
        self.INFO('Initialized\n'.upper()*5)
        self.get_logger().set_level(LOGGLEVEL)

    # =============CALLBACKS=============================

    # ==== EXTERNAL UPDATES

    def restartUnactiveCB(self):
        if not self.greenFlag or self.isActing():
            return
        self.INFO('had to restart')
        if self.hasUnemptyStack():
            self.headFor(self.getTopMarkerPose())
            return
        self.requestRandomTarget()
        
    def flagCB(self, fMsg):
        self.TOPP('flaggCB')
        if fMsg.green is self.greenFlag:
            return
        self.greenFlag = fMsg.green
        if not fMsg.green:
            if self.isActing():
                self.cancelAction()
            return
        if self.hasAruco4Pose():
            self.headFor(self.aruco4Pose)
        elif self.hasUnemptyStack():
            self.headFor(self.getTopMarkerPose())
        else:
            self.requestRandomTarget()

    def detectionCB(self, dMsg):
        self.TOPP('detectionCB')
        if not dMsg.accepted:
            self.INFO('unaccepted marker registered')
            self.markerStack.append(dMsg)
            if self.hasAruco4Pose():
                self.INFO('will prioritize aruco4 (still ongoing)')
                return
            self.headFor(self.getTopMarkerPose())
        self.removeIfExists(dMsg.marker_id)
        if dMsg.marker_id == 4:
            self.INFO('aruco marker nr 4 !')
            self.aruco4Pose = dMsg.pose
            self.headForAruco4()
            return
        self.INFO('marker was accepted')

    def randomTargetCB(self, ftp):
        self.TOPP('randomTargetCB')
        if self.hasAruco4Pose() or self.hasUnemptyStack():
            return
        self.headFor(ftp.result().goal_pose)

    # ==== NAVIAGTION

    def navResponseCB(self, fgh):
        self.TOPP('navResponseCB')
        outdated, accepted = self.receiveNavResponse(fgh)
        if accepted:
            if outdated:
                self.INFO('a more recent target had been sent, this is outdated')
                self.cancelAction()
                return
            self.INFO('new target ACCEPTED')
            return
        if not outdated:
            self.INFO('up to date target UNACCEPTED')
            time.sleep(4.0)
            self.headFor(self.lastSentPose)
            return
        self.INFO('outdated target was UNACCEPTED')

    def navResultCB(self, fr):
        self.TOPP('navResultCB')
        _ = fr.result()
        self.INFO('target resulted')
        if _.status in (act.GoalStatus.STATUS_CANCELED, act.GoalStatus.STATUS_CANCELING):
            return
        elif self.hasAruco4Pose():
            p, q = _.result.base_position, self.aruco4Pose.position
            distanceFromMarker = u.distance(p.x, p.y, q.x, q.y)
            if distanceFromMarker < FIRE_DISTANCE_THRESHHOLD:
                self.INFO('arrived with arucomarker 4')
                self.requestForTeamup()
            else:
                self.INFO('distance to aruco4 not acceptable')
                x, y = self.aruco4Pose.position.x, self.aruco4Pose.position.y
                o = self.aruco4Pose.orientation
                newx, newy = x + math.cos(o), y + math.sin(o) # vector length = 1
                self.headFor(Point(x=newx, y=newy))
        elif len(self.markerStack) > 1:
            self.INFO('will head for other notherunaccepted marker, where had multiple')
            self.markerStack.insert(0, self.markerStack.pop())
            self.headFor(self.getTopMarkerPose())
        else:
            self.INFO('will request for new random point by default')
            self.requestRandomTarget()

    def navFeedbackCB(self, msg):
        if msg.feedback.current_position is None:
            return
        self.latestPosition = msg.feedback.current_position

    def navCancelCB(self, _):
        pass

    # ==== TEAMUP

    def teamupResponseCB(self, ftu):
        self.TOPP('teamupResponseCB')
        gh = ftu.result()
        if not gh.accepted:
            self.INFO('teamup not accepted, leaving teamup')
            self.leaveTeamup()
            return
        self.INFO('teamup accepted')
        gh.get_result_async(
            ).add_done_callback(self.teamupResultCB)

    def teamupResultCB(self, _):
        _ = _.result()
        if _.status != act.GoalStatus.STATUS_SUCCEEDED:
            self.INFO('teamup unsuccessful, sending new request')
            self.requestForTeamup()
            return
        self.INFO('teamup resulted')
        self.leaveTeamup()

    # =============PROCEDURES============================

    def headFor(self, pose, remember=True):
        if isinstance(pose, Point):
            pose = Pose(position=pose)
        if self.isActing():
            self.cancelAction()
        self.sendCount += 1
        if remember:
            self.lastSentPose = pose
        self.bug2Action.send_goal_async(
            Bug2.Goal(target_position=pose.position), self.navFeedbackCB
                ).add_done_callback(self.navResponseCB)
        gx, gy = pose.position.x, pose.position.y
        if self.hasAruco4Pose() and u.equalPoints(pose.position, self.aruco4Pose.position):
            targetStr = 'arocu4'
        elif self.hasUnemptyStack() and u.equalPoints(pose.position, self.getTopMarkerPose().position):
            targetStr = 'unaccepted marker'
        else:
            targetStr = 'other'
        self.INFO('new goal sent: ' + targetStr)

    def cancelAction(self):
        self.INFO('canceling current goal.. expect new sent unless not')
        self.uncanceled.cancel_goal_async()
        self.uncanceled = None

    def receiveNavResponse(self, futureGH):
        gHandle = futureGH.result()
        self.sendCount -= 1
        if gHandle.accepted:
            self.uncanceled = gHandle
            self.uncanceled.get_result_async().add_done_callback(self.navResultCB)
        return self.sendCount > 0, gHandle.accepted

    def removeIfExists(self, marker_id):
        for i in range(len(self.markerStack)):
            if self.markerStack[i].marker_id == marker_id:
                self.INFO('poped accepted marker from stack')
                self.markerStack.pop(i)

    def headForAruco4(self):
        self.INFO('sedning request to got to aruco marker 4')
        self.headFor(self.aruco4Pose)

    def getTopMarkerPose(self):
        return self.markerStack[-1].pose

    def requestRandomTarget(self):
        self.randomTargetSrv.call_async(GoalPose.Request(tb_id=self.id)
            ).add_done_callback(self.randomTargetCB)

    def requestForTeamup(self):
        self.INFO('requesting teamup')
        self.teamupActCli.send_goal_async(
            Teamup.Goal()).add_done_callback(
                self.teamupResponseCB)

    def leaveTeamup(self):
        self.INFO('leaving teamup')
        self.aruco4Pose = None
        if self.hasUnemptyStack():
            self.headFor(self.getTopMarkerPose())
        else:
            self.requestRandomTarget()

    def createRoamingPoint(self, point, maxR=2):
        x, y = point.x, point.y
        r = random.randint(1, round(maxR*10))/10
        dx, dy = random.randint(-r*10, r*10)/10, random.randint(-r*10, r*10)/10
        return Point(x=x+dx, y=y+dy)

    # =============CHECKS============================

    def isActing(self):
        return self.uncanceled is not None

    def canAct(self):
        return self.greenFlag

    def hasAruco4Pose(self):
        return self.aruco4Pose is not None

    def hasUnemptyStack(self):
        return len(self.markerStack) > 0

    # --------------------------------------------------

    def waitForServer(self, callable, name):
        while not callable(timeout_sec=0.5):
            self.INFO('waiting for %s'%(name))
        self.INFO('%s is ready'%(name))

    def TOPP(self, methodname):
        self.get_logger().info(methodname)

    def INFO(self, obj):
        self.get_logger().info(u.logg(
            self.id, '  ' + str(obj)))


def main(args=None):
    rclpy.init(args=args)
    controller = ActionHandler()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()


            
    