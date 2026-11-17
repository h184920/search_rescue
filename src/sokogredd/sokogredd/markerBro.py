import math 
import rclpy
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers
from detection.msg import Detection

class markerBro(Node):
    def __init__(self):
        super().__init__('markerBro')
        self.declare_parameter("namespace")
        self.tb_id = self.get_parameter('namespace').get_parameter_value().string_value
        self.x = None
        self.y = None
        self.yaw = None
        self.turn = 0.0
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(ArucoMarkers, "aruco_markers", self.aruco_callback, 10)
        self.pub = self.create_publisher(Detection, "/xdetection", 10)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q =  msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.turn = abs(msg.twist.twist.angular.z)

    def aruco_callback(self, msg):
        if self.yaw is None:
            return
        for markerId, pose in zip(msg.marker_ids, msg.poses):
            frem = pose.position.z 
            venstre = -pose.position.x
            
            if frem < 1.5: #for å ikke spamme terminalen like mye 
                self.get_logger().info(f"framme= {frem}, venstre={venstre}, yaw={math.degrees(self.yaw)}. Robot={self.x}, {self.y}")
            
            if frem > 2.0:
                continue
            
            detec = Detection()
            detec.tb_id = self.tb_id
            detec.marker_id = markerId
            detec.pose.position.x = self.x + frem*math.cos(self.yaw)- venstre*math.sin(self.yaw)
            detec.pose.position.y = self.y + frem*math.sin(self.yaw) + venstre*math.cos(self.yaw)
            detec.accepted = False 
            self.pub.publish(detec)
            self.get_logger().info(f"tb3_{self.tb_id} såg markøren:{markerId}, på punktet ({detec.pose.position.x}, {detec.pose.position.y})")
            
def main(args=None):
    rclpy.init(args=args)
    controller = markerBro()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()