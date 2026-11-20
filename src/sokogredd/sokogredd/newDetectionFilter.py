import math
import rclpy
from sokogredd import utils as u
import time
from rclpy.node import Node
from detection.msg import Detection
from scoring_interfaces.srv import SetMarkerPosition
from rclpy.logging import LoggingSeverity
from bug2_navigation import global_utils as gu

from rclpy.logging import LoggingSeverity

LOGGLEVEL = LoggingSeverity.INFO

class DetectionFilter(Node):
    def __init__(self):
        super().__init__('detectionFilter')
        self.name = 'detection_filter'
        self.detectionIDs = set()
        self.create_subscription(Detection, '/detection', self.subscriptionCallback, 10)
        self.filteredMarkersPub = self.create_publisher(Detection, '/true_detection', 10)
        self.points_client = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.points_client.wait_for_service(timeout_sec=0.5):
            self.logg('waiting for %s'%('scoring server'))
        self.logg('connection established')
        self.logg('Initialized\n'*5)
        self.get_logger().set_level(LOGGLEVEL)
 
    def subscriptionCallback(self, detectionMsg):
        mId = detectionMsg.marker_id
        if mId in self.detectionIDs or not (-1 < mId < 6):
            return
        self.points_client.call_async(SetMarkerPosition.Request(
                marker_id=detectionMsg.marker_id, 
                marker_position=detectionMsg.pose.position
            )).add_done_callback(self.scoreCallback(detectionMsg))

    def scoreCallback(self, detection):
        def callback(futurePoints):
            s = 'detection of marker %s evaluated by server'%(detection.marker_id)
            detection.accepted = futurePoints.result().accepted
            self.filteredMarkersPub.publish(detection)
            ds = '\n  detection not accepted'
            if detection.accepted:
                ds = '\n  detection accepted'
                self.detectionIDs.add(detection.marker_id)
            self.logg(s+ds)
        return callback

    def logg(self, obj):
        self.get_logger().info(u.logg(self.name, obj))

def main(args=None):
    rclpy.init(args=args)
    controller = DetectionFilter()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()