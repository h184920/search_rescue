import rclpy

from rclpy.node import Node
from std_srvs.srv import Empty

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Client(Node):

    def __init__(self):
        super().__init__('client')
        self.serviceCBG = MutuallyExclusiveCallbackGroup()
        self.timerCBG = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(Empty, 'lock', callback_group=self.serviceCBG)
        self.create_timer(5.0, self.timerCB, callback_group=self.timerCBG)
        self.get_logger().info('initialized client')

    def timerCB(self):
        self.get_logger().info('timer entered')
        _ = self.cli.call(Empty.Request())
        self.get_logger().info('through')

def main():
    try:
        rclpy.init()
        node = Client()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

