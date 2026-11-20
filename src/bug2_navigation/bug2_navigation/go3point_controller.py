import rclpy
import bug2_navigation.global_utils as gu
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from bug2_interfaces.srv import GoToPoint
from bug2_navigation.global_utils import logg
from interfjeses.msg import JustBool
from tf_transformations import euler_from_quaternion
from math import sin, cos, pi, sqrt, acos

TEST = False

LANDING_RANGE = 1.5
TARGET_FRACTION = 0.5
TARGET = (0.0, 0.0)

LINEAR = 0.8


class gotopoint_controller(Node):

    def __init__(self, target=TARGET, init_active=False):
        super().__init__("gotopoint")
        self.declare_parameter('namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active = init_active
        self.received_initial_message = False
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.server = self.create_service(GoToPoint, 'go_to_point_bryter', self.respond_to_switch)
        self.create_subscription(JustBool, 'at_once', self.receiveBreak, 10)
        self.x = self.y = self.orientation = self.dist = self.off = 0.0
        self.tX, self.tY = target
        self.loggMod = 50
        self.loggCount = self.loggMod
        self.force_logg(f"initialized gotopoint_controller with target as {self.tX, self.tY}\n".upper()*5)
        self.get_logger().set_level(LoggingSeverity.FATAL)

    def receiveBreak(self, _):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.5
        self.pub.publish(t)
        self.active = False

    def odom_callback(self, msg):
        self.x, self.y = gu.extract_odom_position(msg)
        self.orientation = gu.extract_odom_orientation(msg)
        self.compute_distance_and_offset()
        self.received_initial_message = True

    def publish_twist(self):
        if not (self.active and self.received_initial_message):
            return
        twist = Twist()
        if self.hasArrived():
            self.logg("Have arrived")
            self.pull_breaks(twist)
            return
        angular = - self.off
        if self.off < -pi/2 or pi/2 < self.off:
            self.logg(f"Orienting")
            twist.linear.x = 0.0
            pa = abs(angular)
            twist.angular.z = sin(angular/pa*min(pa, pi/2))*0.9
        else:
            self.logg("Moving")
            twist.linear.x = cos(angular) * LINEAR
            twist.angular.z = sin(angular) * LINEAR
        if self.dist < LANDING_RANGE: 
            scale = self.dist/LANDING_RANGE
            twist.linear.x *= scale
            twist.angular.z *= scale
        self.logg_state(twist)
        self.pub.publish(twist)

    def respond_to_switch(self, request, response):
        self.force_logg("gotopoint: received request: " + str(request.move_switch))
        for i in range(3):
            twist = Twist()
            twist.linear.x = twist.angular.z = 0.0
            self.pub.publish(twist)
        self.set_target(request.target_position.x, request.target_position.y)
        should_finish = self.hasArrived()
        if not should_finish:
            should_finish = request.move_switch
        self.active = should_finish
        response.success = (
            not (request.move_switch or self.active)
            or (request.move_switch and self.active))
        return response

    # helpers and routines

    def set_target(self, x, y):
        self.tX, self.tY = x, y
        self.compute_distance_and_offset()

    def hasArrived(self):
        return self.dist < TARGET_FRACTION*LANDING_RANGE

    def compute_distance_and_offset(self):
        self.dist, self.off = gu.compute_distance_and_offset(self.x, self.y, self.orientation, self.tX, self.tY)

    def enable(self):
        self.is_active = True

    def pull_breaks(self, twist=None):
        self.is_active = False
        if twist is None:
            twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def logg_state(self, tvist):
        self.logg(f" TARGET = {(self.tX, self.tY)}\n"+
            f" position = ({round(self.x, 2)}, {round(self.y, 2)})\n"+
            f" distance = {round(self.dist, 2)}\n" +
            f" orientation = {round(self.orientation, 3)}"+
            f" offset = {round(self.off, 2)}\n"+
            f" lin = {round(tvist.linear.x, 2)}, ang = {round(tvist.angular.z, 2)}\n")

    def logg(self, obj):
        self.loggCount += 1
        if not self.loggCount%self.loggMod == 0:
            return
        self.loggCount = 0
        self.get_logger().info(logg(self.namespace+'/gotopoint', obj))

    def force_logg(self, obj):
        count = self.loggCount
        self.loggCount = self.loggMod -1
        self.logg(obj)
        self.loggCount = count

# gotopoint_controller end ..

def test_loop(instance: gotopoint_controller):
    from random import randint
    import re
    while True:
        while not instance.hasArrived():
            rclpy.spin_once(instance)
        instance.pull_breaks()
        new_target = input("\nfor random: ENTER\nfor manual: <float>, <float>\nto quit: none of the above\ntarget = ")
        if new_target == '':
            target = (randint(-50, 0)/10, randint(-50, 0)/10)
        else:
            match = re.fullmatch(" *-?(\d).(\d)* *, *-?\d.\d* *", new_target)
            if not match is None:
                xstring, ystring = new_target.split(",")
                target = (float(xstring.strip()), float(ystring.strip()))
                print(f"target position accepted as {target}")
            else: return
        instance.set_target(*target)
        instance.enable()

 
def main(args=None):
    rclpy.init(args=args)
    gotopoint = gotopoint_controller(init_active=TEST)
    if TEST:
        print("running playground")
        test_loop(gotopoint)
    else:
        rclpy.spin(gotopoint)
    gotopoint.destroy_node()
    rclpy.shutdown()