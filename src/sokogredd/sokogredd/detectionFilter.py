import math
import rclpy
import utils as u
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from detection.msg import Detection
from multi_robot_scoring.scoring_interfaces import SetMarkerPosition

# TODO: 
#   endre fra busy wait strategi for poeng-registrering
#   ta hensyn til mulig ikke godkjent deteksjon

class DetectionFilter(Node):

    def __init__(self):
        super().__init__('detection_filter')
        self.__shouldSpin = False
        self.__detections = set()   
        self.__detectionBuffer = list()
        self.create_subscription(Detection, '/detection', self.__subscription_callback)
        self.__filteredMarkersPub = self.create_publisher(Detection, '/true_detection', 10)
        self.__points_client = self.create_client(SetMarkerPosition, '/set_marker_position')

    def spin(self):
        while not self.isFinished():
            rclpy.spin_once(self)
            self.__check_buffer()

    def isFinished(self):
        return not self.__shouldSpin or len(self.__detections) >= 5

    def __subscription_callback(self, detectionMsg):
        if detectionMsg.marker_id in self.__detections:
            return
        self.__detectionBuffer.append(detectionMsg)

    def __check_buffer(self):
        while len(self.__detectionBuffer) != 0:
            detectedPoint = self.__detectionBuffer.pop()
            future = self.__points_client.call_async(SetMarkerPosition(
                marker_id=detectedPoint.marker_id, 
                marker_position=detectedPoint.pose.position
            ))
            while not future.done():
                continue
            detectedPoint.handled = future.result().accepted
            self.__filteredMarkersPub.publish(detectedPoint)
            if detectedPoint.handled:
                self.__detections.add(detectedPoint.marker_id)
