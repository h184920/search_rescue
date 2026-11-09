import math

import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from detection.msg import Detection

from multi_robot_scoring.scoring_interfaces import SetMarkerPosition

# todo: endre vekk fra busy wait strategi for poeng-registrering
# ta hensyn til mulig ikke godkjent deteksjon

class Leder(Node):

    def __init__(self):
        super().__init__('leder_controller')
        self.complete = False
        self.checked = set()
        self.detectionBuffer = list()
        self.create_subscription(Detection, '/aruco_detections', self.subscription_callback, callback_group=ReentrantCallbackGroup)
        self.filteredMarkersPub = self.create_publisher(Detection, '/filtered_detections', 10)
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
        while not self.complete:
            rclpy.spin_once(self)
            self.check_buffer()

    def LOGG(self, obj):
        print(obj)




            
            
            
            

             


    
    

