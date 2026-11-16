import rclpy
import time
from sokogredd import utils as u
import random
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose
from detection.msg import Detection
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from interfjeses.msg import JustBool
from interfjeses.srv import GoalPose
from interfjeses.action import Teamup

DISTANCE_THRESHOLD = 1.5

TBIDS = ['tb3_0', 'tb3_1']

class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        self.__name = 'coordinator'
        self.__occGrid = None
        self.__wBorders, self.__hBorders = list(), list()
        self.__waitingActors = 0
        plis_funger = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.__receiveMapCB, plis_funger)
        self.__flag = self.create_publisher(JustBool, '/run_flag', 10)
        self.__pubTable = {id: self.create_publisher(Detection, '/'+id+'/new_detection', 10) for id in TBIDS}
        self.create_subscription(Detection, '/true_detection', self.__detectionCB, 10)
        self.__pointsService = self.create_service(GoalPose, '/goal_assignment', self.__assignGoalCB)
        self.__teamupTable = {id: ActionServer(self, Teamup, '/'+id+'/teamup', self.__teamupActionCB) for id in TBIDS}
        while not self.__pointsService.wait_for_server(timeout_sec=0.5):
            self.logg('waiting for pointService...')
        
        self.logg('pointsService is ready')
        self.logg('Initialized')

    def __waitForServer(self, server):
        while not server.wait_for_server(timeout_sec=0.5):
            self.__logg()

    def __receiveMapCB(self, ogMsg):
        self.__occGrid = ogMsg
        self.__wBorders = u.findWBorders(ogMsg)
        self.__hBorders = u.findHBorders(ogMsg)
        self.logg('map digested with len(wBorders), len(hBorders) = %s, %s'%(len(self.__wBorders), len(self.__hBorders)))
        self.__flag.publish(JustBool(green=True))

    def __detectionCB(self, dMsg):
        self.logg('marker detection received: %s'%(dMsg))
        self.__pubTable[dMsg.tb_id].publish(dMsg)
        if dMsg.accepted and dMsg.marker_id == 4:
            assist_id = random.choice([id for id in self.pubTable.keys() if id != dMsg.marker_id])
            self.__pubTable[assist_id].publish(dMsg)

    def __assignGoalCB(self, request, response):
        self.logg('received request for new destination from action handler %s'%(request.tb_id))
        if not isinstance(request.tb_id, str):
            raise TypeError('goal-request had invalid namespace id of type %s'%(type(request.tb_id)))
        self.logg(request.tb_id.split('_'))
        response.goal_pose = Pose(position=self.__generateGoal(int(request.tb_id.split('_')[-1].strip())))
        return response

    def __teamupActionCB(self, goal_handle):
        self.__waitingActors += 1
        self.logg('another firefighter teamed up. Count: %s'%(self.__waitingActors))
        while self.__waitingActors < 2:
            time.sleep(1.0)
        self.logg('big fire handled. Releasing bots')
        goal_handle.succeed()
        self.__waitingActors = 0
        return Teamup.Response()

    def __generateGoal(self, idInt):
        h = random.randint(10, len(self.__wBorders) - 10)
        wmin, wmax = self.__wBorders[h]
        delta = (wmax - wmin)//2
        w = random.randint(wmin + idInt*delta, wmin + (idInt+1)*delta)
        hmin, hmax = self.__hBorders[w] 
        if not (hmin < h < hmax):
            h = random.choice((hmin, hmax))
        point = u.mapToRealPoint(self.__occGrid, w, h)
        self.logg('generated point %s'%(point))
        return point

    def logg(self, obj):
        self.get_logger().info(u.logg(self.__name, obj))

def main(args=None):
    rclpy.init(args=args)
    controller = Coordinator()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()