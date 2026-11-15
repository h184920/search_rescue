import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data as fin

from std_srvs.srv import SetBool

import math as m

TEST = True

scale = 1.0

momentum = 0.45
L1 = scale*momentum * 1.24
L2 = scale*momentum * 1.53
L3 = scale*momentum * 1.8
L4 = scale*momentum * 1.8

sharpness = 0.7
STRAIGHT_SCALAR = scale*sharpness * 0.65
INNER_SWING_LIN = scale*sharpness * 0.45
INNER_SWING_ANG = scale*sharpness * 0.8
OUTER_SWING_LIN = scale*sharpness * 0.05
OUTER_SWING_ANG = scale*sharpness * (-1) * 1.0

MARGIN = 0.75
ANGLE_SMALL = 30
ANGLE_LARGE = 30
wallfollower_controller
ang1, ang2, ang3, ang4 = 345, 295, 270, 245

class WallfollowerController(Node):

    def __init__(self, active=False):
        super().__init__("wallfollower")
        self.sub = self.create_subscription(LaserScan, "scan", self.laser_callback, fin)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.active = active
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.server = self.create_service(SetBool, "wall_follower_bryter", self.respond_to_switch)
        self.L0 = self.L1 = self.L2 = self.L3 = self.L4 = 3.5
        self.ang1, self.ang2, self.ang3, self.ang4 = ang1, ang2, ang3, ang4
        self.rightwall = 1
        self.has_known_wall = False
        print("initialized wallfollower_controller")

    def laser_callback(self, msg):
        self.L0 = msg.ranges[(360 - self.ang1)]
        self.L1 = msg.ranges[self.ang1]
        self.L2 = msg.ranges[self.ang2]
        self.L3 = msg.ranges[self.ang3]
        self.L4 = msg.ranges[self.ang4]

    def respond_to_switch(self, request, response):
        print("wallfollower: recevied request " + str(request.data))
        for i in range(3):
            twist = Twist()
            twist.linear.x = twist.angular.z = 0.0
            self.pub.publish(twist)
        if self.active and not request.data and min(self.L0, self.L1) < L1*MARGIN:
            response.success = False
        else:
            self.has_known_wall = False
            self.active = request.data
            response.success = True
        return response

    def publish_twist(self):
        if not self.active:
            return
        twist = Twist()
        l1, l2, l3, l4 = self.L1 < L1, self.L2 < L2, self.L3 < L3, self.L4 < L4
        if (self.L0 < L1 or l1) and not (l2 or l3 or l4) or (max(self.L0, self.L1) < L1*MARGIN):
            print("getting unstuck")
            self.linear(0.0, twist)
            self.angular(INNER_SWING_ANG, twist) 
            self.get_logger().info("Problemet er i nr 1")

        elif (self.L0 < L1 or l1) and self.L0 < self.L1 and not self.has_known_wall:
            print("SWITCHED SIDE")
            self.switch_side()
            self.linear(0.0, twist)
            self.angular(0.0, twist)
            self.get_logger().info("Problemet er i nr 2")
        elif ((l3 or l4) and (not l2 or rotation(self.L3, self.L2, m.radians(abs(self.ang2 - self.ang3))) > m.radians(ANGLE_LARGE))):
            print("rounding edge")
            #self.get_logger().info(f"L1={L1}, L2={L2}, L3={L3}, L4={L4}")
            self.has_known_wall = True
            self.linear(OUTER_SWING_LIN, twist)
            self.angular(OUTER_SWING_ANG, twist)

        elif l1 and not self.aligned_enough(-m.radians(ANGLE_SMALL)):
            print("swinging")
            self.has_known_wall = True
            self.linear(INNER_SWING_LIN*max(0.0, self.L1 - MARGIN*L1)/L1, twist)
            self.angular(INNER_SWING_ANG, twist)
            self.get_logger().info("Problemet er i nr 4")

        elif l2 and l3: 
            print("wallfollowing")
            self.has_known_wall = True
            o = rotation(self.L3 - (self.L3 - L3*MARGIN)*abs(self.L3 - L3*MARGIN)/(L3 - L3*MARGIN), self.L2, m.radians(abs(self.ang2 -self.ang3)))
            self.linear(self.default() * m.cos(o) * ((m.pi/2 - abs(o))/m.pi*2), twist)
            self.angular(self.default() * m.sin(-o), twist)

        elif not (l1 or l2 or l3 or l4) and self.has_known_wall:
            print("reconnection with wall")
            self.linear(OUTER_SWING_LIN, twist)
            self.angular(OUTER_SWING_ANG*0.5, twist)
        else:
            print("free")
            self.linear(self.default(), twist)
            self.angular(0.0, twist)
        print(f"LASAR: L1,   L2,   L3,   L4\n{' '*7}{round(self.L1, 2)}, {round(self.L2, 2)}, {round(self.L3, 2)}, {round(self.L4, 2)}")
        print(f'CMD:', round(twist.linear.x, 2), round(twist.angular.z, 2), end="\n\n")
        self.pub.publish(twist)

    def default(self):
        return STRAIGHT_SCALAR if self.L1 > L1 else STRAIGHT_SCALAR*(max(0.1, self.L1 - L1*MARGIN)/(L1 - MARGIN*L1))

    def aligned_enough(self, min_disalignment):
        O32 = rotation(self.L3, self.L2, m.radians(abs(self.ang2 - self.ang3)))
        O31 = rotation(self.L3, self.L1, m.radians(abs(self.ang1 - self.ang3)))
        aligned = round(O32, 1) == round(O31, 1)
        if aligned:
            print("is aligned enough for", round(m.degrees(O32), 2), end="\n\n")
        return aligned and abs(O32) < abs(min_disalignment)

    def switch_side(self):
        self.ang1 = flip_index(self.ang1)
        self.ang2 = flip_index(self.ang2)  
        self.ang3 = flip_index(self.ang3)  
        self.ang4 = flip_index(self.ang4)
        self.rightwall = (-1)*self.rightwall
        self.has_known_wall = False

    def linear(self, lin, twist):
        twist.linear.x = lin

    def angular(self, ang, twist):
        twist.angular.z = self.rightwall*ang  
          
def flip_index(ang):
    return (360 - ang) % 360

def rotation(orthagonal, diagonal, angle):
    orthagonal = round(orthagonal, 1)
    diagonal = round(diagonal, 1)
    return m.atan((diagonal*m.cos(angle) - orthagonal)/(diagonal*m.sin(angle) + 0.05))

def main(args=None):
    print(f"running wallfollower: active={TEST}")
    rclpy.init(args=args)
    wallfollower = WallfollowerController(active=TEST)
    rclpy.spin(wallfollower)
    wallfollower.destroy_node()
    rclpy.shutdown()