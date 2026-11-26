import rclpy
import rclpy.action as act
import bug2_navigation.global_utils as gu
import time
import math as m
import random
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from interfjeses.msg import JustBool
from std_srvs.srv import SetBool
from bug2_interfaces.srv import GoToPoint
from bug2_interfaces.action import Bug2

GOTORANGE = 1.5
DANGERZONE = GOTORANGE*0.4
LINE_THICKNESS = 1.0
LASAR_SPREAD = 15

class bug2_controller(Node):

    def __init__(self ):
        super().__init__("bug2")
        self.declare_parameter('namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.logglevel = 1
        self.x = self.y = None
        self.lastCheckedOrient = self.lastCheckedPos = self.orient = None
        self.passed_guard = False
        self.reentrantCBG = ReentrantCallbackGroup()
        self.stateTimer = self.create_timer(10.0, self.checkState, self.reentrantCBG)
        self.breakSignaler = self.create_publisher(JustBool, 'at_once', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scanner_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.mainLooper = self.create_timer(0.1, self.main_loop)
        self.clientCBG = MutuallyExclusiveCallbackGroup()
        self.goto_client = self.create_client(GoToPoint, 'go_switch', callback_group=self.clientCBG)
        self.wallfollow_client = self.create_client(SetBool, 'wall_switch', callback_group=self.clientCBG)
        self.lastCheckedPosition = None
        self.spinPub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lastEnabler = self.enable_gotopoint
        self.acceptCount = 0
        self.lockA = threading.Lock()
        self.lockB = threading.Lock()
        self.actionServer = act.ActionServer(self, Bug2, 'navigate', execute_callback=self.lead_callback, goal_callback=self.goalCB, cancel_callback=self.cancelCB, callback_group=self.reentrantCBG)
        while not self.goto_client.wait_for_service(timeout_sec=1.0):
            self.logg2("waiting for gotopoints server to initialize")
        self.logg2("..connection established")
        while not self.wallfollow_client.wait_for_service(timeout_sec=1.0):
            self.logg2("waiting for wallfollowers server to initialize")
        self.reset()
        self.logg2("..connection established")
        self.logg2("initialized bug2 controller\n".upper()*5)

    # callbacks

    def checkState(self):
        if self.orient is None or self.guardOn():
            return
        if self.lastCheckedOrient is None:
            self.lastCheckedOrient = self.orient
            self.lastCheckedPosition = Point(x=self.x, y=self.y)
            return
        if (abs(self.lastCheckedOrient - self.orient) < 0.3
                and abs(self.x - self.lastCheckedPosition.x) < 0.1
                and abs(self.y - self.lastCheckedPosition.y) < 0.1):
            self.logg1('spinning free\n'.upper()*10)
            for _ in range(50):
                self.spinPub.publish(makeTwist(lin=0.0, ang=m.pi))
                time.sleep(0.1)
        self.lastCheckedPosition = Point(x=self.x, y=self.y)
        self.lastCheckedOrient = self.orient

    def main_loop(self):
        if self.guardOn():
            return
        if not self.on_target():
            self.iterate()
            return
        self.stop_all_at_once()

    def cancelCB(self, _):
        return act.CancelResponse.ACCEPT

    def goalCB(self, _):
        if self.acceptCount > 2:
            return act.GoalResponse.REJECT
        with self.lockB:
            self.acceptCount += 1
        return act.GoalResponse.ACCEPT

    def lead_callback(self, goalHandle: act.server.ServerGoalHandle):
        with self.lockA:
            self.logg1('new action started')
            self.reset(target=goalHandle.request.target_position)
            while not (self.guardOn() or self.on_target() or goalHandle.is_cancel_requested):
                time.sleep(1.0)
                if not self.odomed or not self.scanned:
                    continue
                goalHandle.publish_feedback(Bug2.Feedback(current_position=Point(x=self.x, y=self.y)))
            result = Bug2.Result()
            if goalHandle.is_cancel_requested:
                goalHandle.canceled()
            else:
                goalHandle.succeed()
                result.base_position = Point(x=self.x, y=self.y)
            self.spin = True 
            self.reset()
            with self.lockB:
                self.acceptCount -= 1
            self.logg1('action ended')
            return result

    def odom_callback(self, msg):
        if self.guardOn():
            return
        self.passed_guard = True
        self.x, self.y = gu.extract_odom_position(msg)
        self.orient = gu.extract_odom_orientation(msg)
        self.offset = gu.compute_distance_and_offset(
                self.x, self.y, 
                self.orient, 
                self.target.x, self.target.y)[1]       
        self.odomed = True
        self.passed_guard = False

    def laser_callback(self, msg):
        if self.guardOn() or not self.odomed:
            return
        self.guard_passed = True
        self.front_lasar = msg.ranges[0]
        self.lasarL1 = msg.ranges[round(LASAR_SPREAD/2)]
        self.lasarL2 = msg.ranges[LASAR_SPREAD]
        self.lasarR1 = msg.ranges[360 - round(LASAR_SPREAD/2)]
        self.lasarR2 = msg.ranges[360 - LASAR_SPREAD]
        self.aligned_lasar = msg.ranges[(360 - round(m.degrees(self.offset))) % 360]
        self.scanned = True
        self.passed_guard = False

    # internal work

    def iterate(self):
        self.active_handler()
           
    def initial_handler(self):
        self.logg2("INITAL STATE:")
        if not (self.odomed and self.scanned):
            self.logg2("..still waiting for odom and scanner data")
            return
        if not self.is_hindered():
            self.logg2("starting in gotopoint")
            self.enable_gotopoint()
        else:
            self.logg2("starting in wallfollower")
            self.enable_wallfollower()

    def wallfollow(self):
        self.logg2(f"IN STATE WALLFOLLOWER. Count = {self.w_count}")
        if self.on_track() and not self.is_hindered() and self.request_stop_wallfollower():
            self.logg3(f"..position on line\n{'-'*20}switching state{'-'*20}")
            self.enable_gotopoint()
            return False
        self.logg3("..continuing in wallfollower")
        return True
   
    def gotopoint(self):
        self.logg2(f"IN STATE GOTOPOINT. Count = {self.g_count}")
        if self.is_hindered() and self.request_stop_gotopoint():
            self.logg3("-"*20+"switching state"+"-"*20)
            self.enable_wallfollower()
            return False
        self.logg3("..continuing in gotopoint")
        return True

    # enable / disable

    def stop_all_at_once(self):
        self.breakSignaler.publish(JustBool(green=False))

    def enable_gotopoint(self):
        self.logg3("sending request to enable gotopoint")
        response = self.send_and_receive(self.goto_client, GoToPoint.Request(move_switch=True, target_position=self.target))
        if response.success:
            self.lastEnabler = self.enable_gotopoint
            self.logg1("gotopoint enabled")
            self.active_handler = self.gotopoint
            self.g_count += 1
        else:
            self.logg3("gotopoint responded with false\n ..not enabled")
        return response.success

    def request_stop_gotopoint(self):
        self.logg3("sending request to disable gotopoint")
        response = self.send_and_receive(self.goto_client, GoToPoint.Request(move_switch=False, target_position=self.target))
        if response.success:
            self.logg3("gotopoint responded with true\n ..disabled")
        else:
            self.logg3("gotopoint responded with false\n ..not disabled\n SHOULD BE ON TARGET")
        return response.success

    def enable_wallfollower(self):
        self.logg3("sending request to enable wallfollower")
        response = self.send_and_receive(self.wallfollow_client, SetBool.Request(data=True))
        if response.success:
            self.lastEnabler = self.enable_wallfollower
            self.logg1("wallfollower enabled")
            self.set_line_from_current_position()
            self.active_handler = self.wallfollow
            self.w_count += 1
            time.sleep(0.5)
        else:
            self.logg3("wallfollower responded with false ????\n ..not enabled")
        return response.success
  
    def request_stop_wallfollower(self):
        self.logg3("sending request for disabling to wallfollower")
        response = self.send_and_receive(self.wallfollow_client, SetBool.Request(data=False))
        if response.success:
            self.logg3("wallfollower respponded with true\n ..disabled")
        else:
            self.logg3("wallfollower responded with false ????\n ..enabled")
        return response.success

    def send_and_receive(self, client, request):
        return client.call(request)

    # state updates

    def reset(self, target=None): 
        while self.passed_guard:
            self.logg3('passed guard is %s'%(self.passed_guard))
            time.sleep(0.1)
        self.in_reset = True
        self.odomed = self.scanned = False
        self.active_handler = self.initial_handler
        self.target = target
        self.stop_all_at_once()
        self.set_line_from_current_position()
        self.w_count = self.g_count = 0
        self.offset = None
        self.aligned_lasar = self.front_lasar = self.lasarL1 = self.lasarL2 = self.lasarR1 = self.lasarR2 = None
        self.in_reset = False
    
    def set_line_from_current_position(self):
        self.line_begin_x = self.x
        self.line_begin_y = self.y

    # checks

    def guardOn(self):
        return self.target is None or self.in_reset

    def is_hindered(self):
        return ((self.is_frontblocked(GOTORANGE) and self.is_pathblocked(GOTORANGE)
            or self.is_frontblocked(DANGERZONE) or self.is_pathblocked(DANGERZONE)))

    def is_pathblocked(self, range):
        self.logg3(f"checking for blockage along path for distance {range}")
        if (self.aligned_lasar < range):
            self.logg3("WALL DETECTED")
            return True
        self.logg3("path is free")
        return False

    def is_frontblocked(self, range):
        self.logg3(f"checking for blockage of front for distance {range}")
        if (self.front_lasar < range 
            or self.lasarL1 < range or self.lasarR1 < range
            or self.lasarL2 < range or self.lasarR2 < range):
            self.logg2("WALL DETECTED")
            return True
        self.logg3("front free")
        return False

    def on_target(self):
        if self.x is None or self.y is None:
            return False
        distance = vector_length(self.target.x - self.x, self.target.y - self.y)
        return self.approved_distance(distance)
    
    def on_track(self):
        self.logg3("checking position for alignment")
        px, py = delta(self.line_begin_x, self.x), delta(self.line_begin_y, self.y)
        tx, ty = delta(self.line_begin_x, self.target.x), delta(self.line_begin_y, self.target.y)
        p_len = vector_length(px, py)
        if p_len == 0.0:
            return True
        t_len = vector_length(tx, ty)
        if t_len == 0.0:
            angle = 0.0
            distance = p_len
        else:
            angle = m.acos((px*tx + py*ty)/(p_len*t_len))
            distance = m.sin(angle)*p_len
        return angle < m.pi/2 and self.approved_distance(distance)

    def approved_distance(self, distance):
        return distance < LINE_THICKNESS

    # just logging

    def logg3(self, obj, indent=5):
        if self.logglevel < 3:
            return
        self.get_logger().info(" "*indent + str(obj).replace("\n", " "*indent+"\n"))

    def logg2(self, obj, indent=5):
        if self.logglevel < 2:
            return
        self.get_logger().info(" "*indent + str(obj).replace("\n", " "*indent+"\n"))

    def logg1(self, obj, indent=5):
        if self.logglevel < 1:
            return
        self.get_logger().info(" "*indent + str(obj).replace("\n", " "*indent+"\n"))
        

# bug2 end ..

def delta(x1, x2):
    return x2 - x1

def vector_length(x, y):
    return m.sqrt(x**2 + y**2)

def vector_product(x1, y1, x2, y2):
    return x1*x2 + y1*y2

def makeTwist(lin=0.0, ang=0.0):
    twist = Twist()
    twist.linear.x = lin
    twist.angular.z = ang
    return twist

TARGET = None
TEST = False

def test_loop(instance: bug2_controller, executor: MultiThreadedExecutor):
    import re
    while True:
        target = (random.randint(-10, 10)/10, random.randint(-50, 50)/10)
        instance.logg1('\n\n\n new target set: %s'%(str(target)))
        time.sleep(3.0)
        instance.reset(target=Point(x=target[0], y=target[1]))
        count = 0
        while not instance.on_target():
            executor.spin_once()
            if count % 50 == 0:
                instance.logg1('\nTARGET: %s\n'%(str(target)))
                count = 0
            count += 1


def main(args=None):
    rclpy.init(args=args)
    controller = bug2_controller()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    if TEST:
        test_loop(controller, executor)
    else:
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()