import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ros2_aruco_interfaces.msg import ArucoMarkers



class superController(Node):
    def __init__(self) -> None:
        super().__init__('superController')

        #TODO subscribe til begge namespacene sine camera og posisjon.
        

        #TODO Subscribe to both robots’ detectors

        self.create_subscription(ArucoMarkers, '/tb3_0/aruco_markers', self.tb3_0_callback, 10)
        self.create_subscription(ArucoMarkers, '/tb3_1/aruco_markers', self.tb3_1_callback, 10)

        def tb3_0_callback(self, msg):
            self.marker_callback(msg, 'tb3_0')

        def tb3_1_callback(self, msg):
            self.marker_callback(msg, 'tb3_1')

        #def marker_callback(self, msg, robot):


def main(args=None):
    rclpy.init(args=args)
    controller = superController()
    rclpy.spin(controller)
    controller.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()