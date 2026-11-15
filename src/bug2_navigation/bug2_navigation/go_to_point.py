import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from bug2_interfaces.srv import GoToPoint
import math


class go_to_point(Node):
    def __init__(self) -> None:
        super().__init__("go_to_point")

        self.goal_x = None
        self.goal_y = None

        self.sub = self.create_subscription(Odometry, "odom", self.clbk_odom, 10)

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.aktiv = False
        self.service = self.create_service(GoToPoint, "go_to_point_bryter", self.bytt_til_WF)


        #Creates a timer definition -> during rclpy.spin() the function self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 
    


    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]


    def bytt_til_WF(self, request, response):
        self.aktiv = request.move_switch
        self.goal_x = request.target_position.x
        self.goal_y = request.target_position.y
        response.success = True
        return response

    def timer_callback(self):
        if self.position is None:
            return 
        
        if not self.aktiv:
            return

        vel_msg = Twist()

        dx = self.goal_x -self.position.x
        dy = self.goal_y - self.position.y

        avstand = math.sqrt((dx**2)+(dy**2)) #avstand til mål
        onsket_yaw = math.atan2(dy, dx) #ønsket yaw grad
        d_yaw = onsket_yaw - self.yaw #delta yaw
        yaw_off_target = math.atan2(math.sin(d_yaw), math.cos(d_yaw)) #hvor mye offcorse den er fra target

        if avstand < 0.15:
            pass 
            

        elif abs(yaw_off_target) > 0.3: #tweak for mer smoothnes i at den ikke skal måtte overskinge så mye 
            if yaw_off_target > 0:
                vel_msg.angular.z = 0.5
            else:
                vel_msg.angular.z = -0.5
        else: 
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 1.0 * yaw_off_target # for å garrantere at den er float 

        self.pub.publish(vel_msg)

    

def main(args=None):
    rclpy.init(args=args)

    controller = go_to_point()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()