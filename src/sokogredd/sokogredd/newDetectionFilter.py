import math
import rclpy
from sokogredd import utils as u
import time
from rclpy.node import Node
from detection.msg import Detection
from scoring_interfaces.srv import SetMarkerPosition

class DetectionFilter(Node):

    def __init__(self):
        super().__init__('detectionFilter')
        self.__name = 'detection_filter'
        self.__detectionIDs = set()
        self.create_subscription(Detection,  '/detection', self.__subscription_callback, 10)
        self.__filteredMarkersPub = self.create_publisher(Detection, '/true_detection', 10)
        self.__points_client = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.__points_client.wait_for_service():
            self.get_logger().info("Venter på poengserver ....")
        self.logg('Initialized\n'*5)
 
    def __subscription_callback(self, detectionMsg):
        s='Detection received'
        if detectionMsg.marker_id in self.__detectionIDs:
            self.logg(s+'\nmarkeer has already been accepted.. so pass')
            return
        self.logg(s+'\nsending it to server')
        self.__points_client.call_async(SetMarkerPosition.Request(
                marker_id=detectionMsg.marker_id, 
                marker_position=detectionMsg.pose.position
            )).add_done_callback(self.__scoreCB(detectionMsg))

    def __scoreCB(self, detection):
        def callback(_, futurePoints):
            s = 'detection of marker %s evaluated by server'%(detection.marker_id)
            detection.accepted = futurePoints.result().accepted
            self.__filteredMarkersPub.publish(detection)
            ds = '\n  detection not accepted'
            if detection.accepted:
                ds = '\n  detection accepted'
                self.__detectionIDs.add(detection.marker_id)
            self.logg(s+ds)
        return callback

    def logg(self, obj):
        self.get_logger().info(u.logg(self.__name, obj))

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