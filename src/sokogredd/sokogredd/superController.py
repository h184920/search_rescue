import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class superController(Node):
    def __init__(self) -> None:
        super().__init__('superController')

        #TODO subscribe til begge namespacene sine camera og posisjon.
        


#TODO lag callbacks


def main(args=None):
    rclpy.init(args=args)
    controller = superController()
    rclpy.spin(controller)
    controller.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()