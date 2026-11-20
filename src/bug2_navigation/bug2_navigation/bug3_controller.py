import rclpy
import rclpy.action as act
import bug2_navigation.global_utils as gu

import time
import math as m

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.logging import LoggingSeverity

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

from bug2_interfaces.action import Bug2
from bug2_interfaces.srv import GoToPoint
from interfjeses.msg import JustBool

from random import randint

from rclpy.timer import Timer


GOTORANGE = 1.5
DANGERZONE = GOTORANGE*0.4
LINE_THICKNESS = 0.75
LASAR_SPREAD = 15


class bug2_controller(Node):

    def __init__(self ):
        super().__init__("bug2")
        self.declare_parameter('namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.loggCount = 0
        self.loggMod = 50
        self.x = self.y = None
        self.passed_guard = False
        self.breakSignaler = self.create_publisher(JustBool, 'at_once', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scanner_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.mainLooper = self.create_timer(0.1, self.main_loop)
        self.clientCBG = MutuallyExclusiveCallbackGroup()
        self.goto_client = self.create_client(GoToPoint, 'go_to_point_bryter', callback_group=self.clientCBG)
        self.wallfollow_client = self.create_client(SetBool, 'wall_follower_bryter', callback_group=self.clientCBG)
        self.actionCBG = MutuallyExclusiveCallbackGroup()
        self.actionServer = act.ActionServer(self, Bug2, 'bug2', execute_callback=self.lead_callback, cancel_callback=self.cancel_callback, callback_group=self.actionCBG)
        while not self.goto_client.wait_for_service(timeout_sec=1.0):
            self.force_logg("waiting for gotopoints server to initialize")
        self.force_logg("..connection established")
        while not self.wallfollow_client.wait_for_service(timeout_sec=1.0):
            self.force_logg("waiting for wallfollowers server to initialize")
        self.reset()
        self.force_logg("..connection established")
        self.force_logg("initialized bug2 controller\n".upper()*5)
        self.get_logger().set_level(LoggingSeverity.FATAL)

    def main_loop(self):
        if self.guardOn():
            return
        if not self.on_target():
            self.iterate()
            return
        self.stop_all_at_once()
        time.sleep(1.0)

    # action callbacks

    def lead_callback(self, goalHandle: act.server.ServerGoalHandle):
        self.reset(target=goalHandle.request.target_position)
        was_canceled = False
        while not (self.guardOn() or was_canceled):
            if goalHandle.is_cancel_requested:
                was_canceled = True
                goalHandle.canceled()
                break
            time.sleep(1.0)
            if not self.odomed or not self.scanned:
                continue
            goalHandle.publish_feedback(Bug2.Feedback(current_position=Point(x=self.x, y=self.y)))
        result = Bug2.Result(was_canceled=was_canceled)
        if not was_canceled:
            goalHandle.succeed()
        result.base_position = Point(x=self.x, y=self.y)
        self.spin = True
        self.reset()
        return result

    def cancel_callback(self, cReq):
        self.force_logg('goal was canceled and callback called\n'*10)
        return act.CancelResponse.ACCEPT
    
    # subscription callbacks

    def odom_callback(self, msg):
        if self.guardOn():
            return
        self.passed_guard = True
        self.x, self.y = gu.extract_odom_position(msg)
        self.offset = gu.compute_distance_and_offset(
                self.x, self.y, 
                gu.extract_odom_orientation(msg), 
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
        self.logg("INITAL STATE:")
        if not (self.odomed and self.scanned):
            self.itrlog("..still waiting for odom and scanner data")
            return
        if not self.is_hindered():
            self.itrlog("starting in gotopoint")
            self.enable_gotopoint()
        else:
            self.itrlog("starting in gotopoint")
            self.enable_wallfollower()

    def wallfollow(self):
        self.logg(f"IN STATE WALLFOLLOWER. Count = {self.w_count}")
        if self.on_track() and not self.is_hindered() and self.request_stop_wallfollower():
            self.itrlog(f"..position on line\n{'-'*20}switching state{'-'*20}")
            self.enable_gotopoint()
            return False
        self.itrlog("..continuing in wallfollower")
        return True
   
    def gotopoint(self):
        self.logg(f"IN STATE GOTOPOINT. Count = {self.g_count}")
        if self.is_hindered() and self.request_stop_gotopoint():
            self.itrlog("-"*20+"switching state"+"-"*20)
            self.enable_wallfollower()
            return False
        self.itrlog("..continuing in gotopoint")
        return True

    # enable / disable

    def stop_all_at_once(self):
        self.breakSignaler.publish(JustBool(green=False))

    def enable_gotopoint(self):
        self.itrlog("sending request to enable gotopoint")
        response = self.send_and_receive(self.goto_client, GoToPoint.Request(move_switch=True, target_position=self.target))
        if response.success:
            self.itrlog("gotopoint responed with true\n ..gotopoint enabled")
            self.active_handler = self.gotopoint
            self.g_count += 1
        else:
            self.itrlog("gotopoint responded with false\n ..not enabled")
        return response.success

    def request_stop_gotopoint(self):
        self.itrlog("sending request to disable gotopoint")
        response = self.send_and_receive(self.goto_client, GoToPoint.Request(move_switch=False, target_position=self.target))
        if response.success:
            self.itrlog("gotopoint responded with true\n ..disabled")
        else:
            self.itrlog("gotopoint responded with false\n ..not disabled\n SHOULD BE ON TARGET")
        return response.success

    def enable_wallfollower(self):
        self.itrlog("sending request to enable wallfollower")
        response = self.send_and_receive(self.wallfollow_client, SetBool.Request(data=True))
        if response.success:
            self.itrlog("wallfollower responded with true\n ..enabled")
            self.set_line_from_current_position()
            self.active_handler = self.wallfollow
            self.w_count += 1
        else:
            self.itrlog("wallfollower responded with false ????\n ..not enabled")
        return response.success
  
    def request_stop_wallfollower(self):
        self.itrlog("sending request for disabling to wallfollower")
        response = self.send_and_receive(self.wallfollow_client, SetBool.Request(data=False))
        if response.success:
            self.itrlog("wallfollower respponded with true\n ..disabled")
        else:
            self.itrlog("wallfollower responded with false ????\n ..enabled")
        return response.success

    def send_and_receive(self, client, request):
        return client.call(request)

    # state updates

    def reset(self, target=None):
        while self.passed_guard:
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
        self.itrlog(f"checking for blockage along path for distance {range}")
        if (self.aligned_lasar < range):
            self.itrlog("WALL DETECTED")
            return True
        self.itrlog("path is free")
        return False

    def is_frontblocked(self, range):
        self.itrlog(f"checking for blockage of front for distance {range}")
        if (self.front_lasar < range 
            or self.lasarL1 < range or self.lasarR1 < range
            or self.lasarL2 < range or self.lasarR2 < range):
            self.logg("WALL DETECTED")
            return True
        self.itrlog("front free")
        return False

    def on_target(self):
        if self.x is None or self.y is None:
            return False
        distance = vector_length(self.target.x - self.x, self.target.y - self.y)
        return self.approved_distance(distance)
    
    def on_track(self):
        self.itrlog("checking position for alignment")
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

    def itrlog(self, message, indent=5):
        self.logg(" "*indent + str(message).replace("\n", " "*indent+"\n"))

    def logg(self, obj):
        self.loggCount += 1
        if not self.loggCount%self.loggMod == 0:
            return
        self.loggCount = 0
        self.get_logger().info(gu.logg(self.namespace+'/bug2', obj))

    def force_logg(self, obj):
        count = self.loggCount
        self.loggCount = self.loggMod -1
        self.logg(obj)
        self.loggCount = count
        

# bug2 end ..

def delta(x1, x2):
    return x2 - x1

def vector_length(x, y):
    return m.sqrt(x**2 + y**2)

def vector_product(x1, y1, x2, y2):
    return x1*x2 + y1*y2

TARGET = None


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    controller = bug2_controller()
    executor = MultiThreadedExecutor()
    controller.reset(TARGET)
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()