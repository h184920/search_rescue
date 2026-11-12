import math

import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from detection.msg import Detection

from scoring_interfaces.srv import SetMarkerPosition

# todo: endre vekk fra busy wait strategi for poeng-registrering
# ta hensyn til mulig ikke godkjent deteksjon

class DetectionFilter(Node):

    def __init__(self):
        super().__init__('detection_filter')
        self.complete = False
        self.checked = set()   
        self.detectionBuffer = list()
        self.create_subscription(Detection, '/detection', self.subscription_callback, 10)
        self.filteredMarkersPub = self.create_publisher(Detection, '/true_detection', 10)
        self.points_client = self.create_client(SetMarkerPosition, '/set_marker_position')

    def subscription_callback(self, detectionMsg):
        if detectionMsg.marker_id in self.checked:
            return
        self.detectionBuffer.append(detectionMsg)

    def check_buffer(self):
        while len(self.detectionBuffer) != 0:
            detectedPoint = self.detectionBuffer.pop()
            future = self.points_client.call_async(SetMarkerPosition(
                marker_id=detectedPoint.marker_id, 
                marker_position=detectedPoint.position
            ))
            while not future.done():
                continue
            if not future.result().accepted:
                self.LOGG('det som ikke skulle skjedde skjedde desverre siden saant.. \n server godkjente ikke punktenen')
                return
            self.checked.add(detectedPoint.marker_id)
            self.filteredMarkersPub.publish(detectedPoint)
            
    def spin(self):
        while not self.isComplete():
            rclpy.spin_once(self)
            self.check_buffer()

    def isComplete(self):
        return self.complete or len(self.checked) >= 5

    def setComplete(self, buulien):
        self.complete = buulien

    def LOGG(self, obj):
        print(obj)


def main(args=None):
    rclpy.init(args=args)
    controller = DetectionFilter()
    rclpy.spin(controller)   
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

            
            
            
            

             


    
    

