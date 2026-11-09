import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from bug2_interfaces.srv import GoToPoint
from std_srvs.srv import SetBool
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point
import math

import time 
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bug2_interfaces.action import Bug2


class bug2_controller(Node):
    def __init__(self) -> None:
        super().__init__("bug2_controller")

        self.tilstand = None #starter vell alltid med go_to_point 

        self.goalX = None
        self.goalY = None 
        self.aktivtMaal = False
        self.actionServer = ActionServer(self, Bug2, "bug2", execute_callback=self.execute_callback, callback_group=ReentrantCallbackGroup())

        self.startX = None
        self.startY = None
        self.position = None

        self.distanceFront = 5.0
        self.distanceLastLine = 0.0

        self.GTPclient = self.create_client(GoToPoint, "go_to_point_bryter")
        self.WFclient = self.create_client(SetBool, "wall_follower_bryter")

        while not self.WFclient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Venter på WallFollower service ....")

        while not self.GTPclient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Venter på GoToPoint service ...")

        self.subOdom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.subLaser = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)

        #Creates a timer definition -> during rclpy.spin() the function self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.controll_loop) 

    def execute_callback(self, goal_handle):
        maal = goal_handle.request.target_position
        self.get_logger().info(f"Nye målet er : ({maal.x},{maal.y}) ")

        #nullstill alt slik at contrpller_loop kan starte fresh 
        self.goalX = maal.x
        self.goalY = maal.y 
        self.startX = None
        self.startY = None
        self.distanceLastLine = 0.0
        self.tilstand = None
        self.aktivtMaal = True 

        feedbavk = Bug2.Feedback()
        while (rclpy.ok() and self.tilstand != "ferdig"):
            if goal_handle.is_cancel_requested:
                self.set_go_to_point(False)
                self.set_wall_follower(False)
                self.aktivtMaal = False
                self.tilstand = None
                goal_handle.canceled()
                return Bug2.Result()

            if self.position is not None:
                feedbavk.current_position = self.position
                goal_handle.publish_feedback(feedbavk)
            time.sleep(0.5)

        self.aktivtMaal = False
        goal_handle.succeed()
        result = Bug2.Result()
        result.base_position = self.position
        return result

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]


    def laser_callback(self, msg):
        self.distanceFront = msg.ranges[0]


    def avstand_maal(self):
        dx = self.goalX - self.position.x 
        dy = self.goalY - self.position.y

        return math.sqrt((dx**2) + (dy**2))


    def avstand_linje(self):
        #finnes sikkert en måte som er 100 ganger bedre å bruke så se etter det om det er mye tid igjen
        teller = abs(((self.goalX-self.startX)*(self.startY-self.position.y)) - ((self.startX-self.position.x)*(self.goalY-self.startY)))
        nevner = math.sqrt((self.goalX-self.startX)**2+(self.goalY-self.startY)**2)
        return teller/nevner


    def set_wall_follower(self, aktiv):
        req = SetBool.Request()
        req.data = aktiv
        self.WFclient.call_async(req)


    def set_go_to_point(self, aktiv):
        req = GoToPoint.Request()
        req.move_switch = aktiv
        punkt = Point()
        punkt.x = self.goalX
        punkt.y = self.goalY
        req.target_position = punkt
        self.GTPclient.call_async(req)


    def endre_tilstand(self, ny_tilstand):
        if self.tilstand == ny_tilstand:
            pass
        elif ny_tilstand == "ferdig":
            self.set_go_to_point(False)
            self.set_wall_follower(False)
            self.get_logger().info("Ferdig oppgave løst")
        elif self.tilstand == "go_to_point" and ny_tilstand == "wall_follower":
            self.set_go_to_point(False)
            self.set_wall_follower(True)
            self.get_logger().info("Bytter over til wall_follower")
        else:
            self.set_wall_follower(False)
            self.set_go_to_point(True)
            self.get_logger().info("Bytter over til go_to_point")

        self.tilstand = ny_tilstand
     

    def controll_loop(self):

        if self.position is None or not self.aktivtMaal:
            return

        if self.startX is None:
            self.startX = self.position.x
            self.startY = self.position.y
            self.get_logger().info(f"Starter på posisjon: ({self.startX},{self.startY}) ")
            self.endre_tilstand("go_to_point")
            return

        if self.avstand_maal() <= 0.1: 
            self.endre_tilstand("ferdig")
            return

        if self.tilstand == "go_to_point":
            if self.distanceFront < 0.5:
                self.distanceLastLine = self.avstand_maal()
                self.endre_tilstand("wall_follower")

        elif self.tilstand == "wall_follower":
            if self.avstand_linje() < 0.1 and self.avstand_maal() < self.distanceLastLine:
                if ((self.distanceLastLine - self.avstand_maal()) > 0.5):
                    self.endre_tilstand("go_to_point")



def main(args=None):

    rclpy.init(args=args)

    controller = bug2_controller()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.spin()

    controller.destroy_node()

    rclpy.shutdown()
  



if __name__ == '__main__':
    main()
