import rclpy
import time as t
import random as r

from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Point
from bug2_interfaces.action import Bug2


class CoordinatorMock(Node):

    def __init__(self, namespace):
        super().__init__('mocker')
        self.__namespace = namespace
        self.serv = ActionServer(Bug2, 'tb3_%s'%(self.__namespace), self.__actionCallb)





