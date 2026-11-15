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
        self.__detections = set()  
        self.__wasKilled = False 
        self.__detectionBuffer = list()
        self.create_subscription(Detection, '/detection', self.__subscription_callback, 10)
        self.__filteredMarkersPub = self.create_publisher(Detection, '/true_detection', 10)
        self.__points_client = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.__points_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Venter på __points_client service ....")

        self.logg('INITIALIZED')

    def spin(self):
        self.logg('SPINNING')
        while not self.isDead():
            rclpy.spin_once(self)
            self.__check_buffer()
            #time.sleep(1.0)
        self.logg('IS DEAD')

    def logg(self, obj):
        u.logg(self.__name, obj)

    def isDead(self):
        return self.__wasKilled or len(self.__detections) >= 5

    def kill(self):
        self.__wasKilled = True
 
    def __subscription_callback(self, detectionMsg):
        s='DETECTION RECEIVED\n'
        if detectionMsg.marker_id in self.__detections:
            self.logg(s+' was already known and accepted marker.. PASS')
            return
        self.logg(s+' adding it to buffer')
        self.__detectionBuffer.append(detectionMsg)

    def __check_buffer(self):
        s=''
        while len(self.__detectionBuffer) != 0:
            s+='\nsending next detection to server'
            detectedPoint = self.__detectionBuffer.pop()
            future = self.__points_client.call_async(SetMarkerPosition.Request(
                marker_id=detectedPoint.marker_id, 
                marker_position=detectedPoint.pose.position
            ))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done():
                self.get_logger().info("timeout på /set_marker_posision")
                self.__detectionBuffer.append(detectedPoint)
                break
            detectedPoint.accepted = future.result().accepted
            self.__filteredMarkersPub.publish(detectedPoint)
            ds = '\n  detection not accepted'
            if detectedPoint.accepted:
                ds = '\n  detection accepted'
                self.__detections.add(detectedPoint.marker_id)
            s+=ds
        self.logg(s)

def main(args=None):
    rclpy.init(args=args)
    controller = DetectionFilter()
    try:
        controller.spin()
    except KeyboardInterrupt:
        controller.kill()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()