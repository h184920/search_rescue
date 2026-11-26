import rclpy
import bug2_navigation.global_utils as gu
import math as m
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
if __name__ != '__main__':
    from interfjeses.msg import JustBool
from std_srvs.srv import SetBool

import time

TEST = False
LOGGING = 0

scale = 1.25

momentum = 1.0
L1 = scale*momentum * 1.34
L2 = scale*momentum * 1.53
L3 = scale*momentum * 1.8
L4 = scale*momentum * 1.8

sharpness = 0.8
STRAIGHT_SCALAR = scale*sharpness * 0.75
INNER_SWING_LIN = scale*sharpness * 0.45
INNER_SWING_ANG = scale*sharpness * 0.75
OUTER_SWING_LIN = scale*sharpness * 0.35
OUTER_SWING_ANG = scale*sharpness * (-1) * 0.95

MARGIN = 0.4
ANGLE_SMALL = 25
ANGLE_LARGE = 30

ang1, ang2, ang3, ang4 = 345, 295, 270, 245

class WallfollowerController(Node):

    def __init__(self, active=TEST):
        super().__init__("wallfollower")
        self.declare_parameter('namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.logg2(self.namespace)
        self.active = active
        self.sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.3, self.publish_twist)
        self.server = self.create_service(SetBool, 'wall_switch', self.respond_to_switch)
        self.create_subscription(JustBool, 'at_once', self.receiveBreak, 10)
        self.L1A = self.L1B = self.L2 = self.L3 = self.L4 = 3.5
        self.frontParallel = self.backParallel = self.antenna = 3.5 
        self.ang1, self.ang2, self.ang3, self.ang4 = ang1, ang2, ang3, ang4
        self.rightwall = 1
        self.has_known_wall = False
        self.alignment = 0.0
        self.frontPAng = self.backPAng = 0.0
        self.logg2("initialized wallfollower_controller\n".upper()*5)

    def receiveBreak(self, _):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.5
        self.pub.publish(t)
        self.active = False

    def laser_callback(self, msg):
        self.L1A = msg.ranges[(360 - self.ang1)]
        self.L1B = msg.ranges[self.ang1]
        self.L2 = msg.ranges[self.ang2]
        self.L3 = msg.ranges[self.ang3]
        self.L4 = msg.ranges[self.ang4]
        self.alignment = self.wallAlignment()
        self.frontPAng = (359 - round(m.degrees(self.alignment))%359)
        self.backPAng = 180 - round(m.degrees(self.alignment))
        self.frontParallel = msg.ranges[self.frontPAng]
        self.backParallel = msg.ranges[self.backPAng]
        if self.active:
            frot = rotate(self.frontPAng, -self.rightwall*90)
            brot = rotate(self.backPAng, self.rightwall*90)
            self.logg2('fronangle, frot = %s, %s'%(self.frontPAng, frot))
            self.logg2('backangle, brot = %s, %s'%(self.backPAng, brot))
            assert abs(rotate(self.frontPAng, 180) - self.backPAng) < 3
            assert abs(frot - brot) < 3

    def respond_to_switch(self, request, response):
        self.logg1("wallfollower: recevied request " + str(request.data))
        for i in range(3):
            twist = Twist()
            twist.linear.x = twist.angular.z = 0.0
            self.pub.publish(twist)
        if self.active and not request.data and min(self.L1A, self.L1B) < L1*MARGIN:
            response.success = False
        else:
            self.has_known_wall = False
            self.active = request.data
            response.success = True
        if self.active:
            self.pickSide()
        return response

    def publish_twist(self):
        if not self.active:
            return
        twist = Twist()
        l1, l2, l3, l4 = self.L1B < L1, self.L2 < L2, self.L3 < L3, self.L4 < L4
        if (self.L1A < L1 or l1) and not (l2 or l3 or l4) or (max(self.L1A, self.L1B) < L1*MARGIN):
            self.logg3("getting unstuck")
            self.linear(0.0, twist)
            self.angular(INNER_SWING_ANG, twist) 
        elif (self.L1A < L1 or l1) and self.L1A < self.L1B and not self.has_known_wall:
            self.logg3("SWITCHED SIDE")
            self.switch_side()
            self.linear(0.0, twist)
            self.angular(0.0, twist)
        elif ((l3 or l4) and (not l2 or self.alignment > m.radians(ANGLE_LARGE))):
            self.logg3("rounding edge")
            self.has_known_wall = True
            self.linear(OUTER_SWING_LIN, twist)
            self.angular(OUTER_SWING_ANG, twist)
        elif l1 and not self.aligned_enough(-m.radians(ANGLE_SMALL)):
            self.logg3("swinging")
            self.has_known_wall = True
            self.linear(INNER_SWING_LIN*max(0.0, self.L1B - MARGIN*L1)/L1, twist)
            self.angular(INNER_SWING_ANG, twist)
        elif l2 and l3: 
            self.logg3("wallfollowing")
            self.has_known_wall = True
            o = rotation(self.L3 - (self.L3 - L3*MARGIN)*abs(self.L3 - L3*MARGIN)/(L3 - L3*MARGIN), self.L2, m.radians(abs(self.ang2 -self.ang3)))
            self.linear(self.default() * m.cos(o) * ((m.pi/2 - abs(o))/m.pi*2), twist)
            self.angular(self.default() * m.sin(-o), twist)
        elif not (l1 or l2 or l3 or l4) and self.has_known_wall:
            self.logg3("reconnection with wall")
            self.linear(OUTER_SWING_LIN, twist)
            self.angular(OUTER_SWING_ANG*0.5, twist)
        else:
            self.logg3("free")
            self.linear(self.default(), twist)
            self.angular(0.0, twist)
        self.logg3(f"LASAR: L1,   L2,   L3,   L4\n{' '*7}{round(self.L1B, 2)}, {round(self.L2, 2)}, {round(self.L3, 2)}, {round(self.L4, 2)}")
        self.logg3(f'CMD:', round(twist.linear.x, 2), round(twist.angular.z, 2))
        self.pub.publish(twist)

    def default(self):
        return STRAIGHT_SCALAR if self.L1B > L1 else STRAIGHT_SCALAR*(max(0.1, self.L1B - L1*MARGIN)/(L1 - MARGIN*L1))

    def wallAlignment(self):
        if self.L3 > 3.5 or self.L2 > 3.5:
            return 0.0 
        return rotation(self.L3, self.L2, m.radians(abs(self.ang2 - self.ang3)))

    def aligned_enough(self, min_disalignment):
        O32 = rotation(self.L3, self.L2, m.radians(abs(self.ang2 - self.ang3)))
        O31 = rotation(self.L3, self.L1B, m.radians(abs(self.ang1 - self.ang3)))
        aligned = round(O32, 1) == round(O31, 1)
        if aligned:
            self.logg2("is aligned enough for", round(m.degrees(O32), 2))
        return aligned and abs(O32) < abs(min_disalignment)

    def switch_side(self):
        self.ang1 = flip_index(self.ang1)
        self.ang2 = flip_index(self.ang2)  
        self.ang3 = flip_index(self.ang3)  
        self.ang4 = flip_index(self.ang4)
        self.rightwall = (-1)*self.rightwall
        self.has_known_wall = False

    def pickSide(self):
        if (not bool(self.alignment) or self.frontParallel >= self.backParallel or self.L1A <= self.L1B
            or not (self.frontParallel < 0.5 or self.backParallel < 0.5)):
            return
        self.logg1('switching side\n'.upper()*10)
        known = self.has_known_wall
        self.switch_side()
        self.has_known_wall = known
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.5*self.rightwall
        self.pub.publish(t)
        time.sleep(1.5)

    def linear(self, lin, twist):
        twist.linear.x = lin

    def angular(self, ang, twist):
        twist.angular.z = self.rightwall*ang  

    def logg3(self, *args):
        if LOGGING < 3:
            return
        self.logg1(*args)

    def logg2(self, *args):
        if LOGGING < 2:
            return
        self.logg1(*args)

    def logg1(self, *args):
        if LOGGING < 1:
            return
        self.get_logger().info(gu.logg(self.namespace+'/wallfollower', *args))
          
def flip_index(ang):
    return (360 - ang) % 360

def rotate(deg, deltaDeg):
    return (360 + deg + deltaDeg)%360

def rotation(orthagonal, diagonal, angle):
    orthagonal = round(orthagonal, 1)
    diagonal = round(diagonal, 1)
    return m.atan((diagonal*m.cos(angle) - orthagonal)/(diagonal*m.sin(angle)))

def main(args=None):
    rclpy.init(args=args)
    wallfollower = WallfollowerController(active=TEST)
    try:
        rclpy.spin(wallfollower)
    except KeyboardInterrupt:
        pass
    wallfollower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    deg = 1
    for deg in range(360):
        print('%s |+ %s = %s  |- %s = %s'%(deg, 90, rotate(deg, 90), 90, rotate(deg, -90)))

