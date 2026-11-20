import rclpy
import time

from rclpy.node import Node
from std_srvs.srv import Empty

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Server(Node):

    def __init__(self):
        super().__init__('server')
        self.serv = self.create_service(Empty, 'lock', self.deadlockSlowResponse)
        self.get_logger().info('initialized server')

    def deadlock(self, request, response):
        self.get_logger().info('recevied request')
        return response

    def deadlockSlowResponse(self, req, res):
        self.get_logger().info('received request for slow response')
        self.get_logger().info('will respond in..')
        for i in range(5):
            print(5-i, end='', flush=True)
            time.sleep(1.0)
        return res

def main():
    try:
        rclpy.init(args=None)
        node = Server()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
