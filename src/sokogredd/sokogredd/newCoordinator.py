import rclpy
import time
import sokogredd.utils as u
import random
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose
from detection.msg import Detection
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from interfjeses.msg import JustBool
from interfjeses.srv import GoalPose
from interfjeses.action import Teamup
from rclpy.logging import LoggingSeverity

DISTANCE_THRESHOLD = 1.5

TBIDS = ['tb3_0', 'tb3_1']

LOGGLEVEL = LoggingSeverity.INFO

class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        self.name = 'coordinator'
        self.occGrid = None
        self.wBorders, self.hBorders = list(), list()
        self.waitingActors = 0
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                        depth=5)
        self.create_subscription(OccupancyGrid, '/map', self.receiveMapCallback, qos_profile)
        self.create_subscription(Detection, '/true_detection', self.detectionCallback, 10)
        self.flagPublisher = self.create_publisher(JustBool, '/run_flag', 10)
        self.pubTable = {namespace: self.create_publisher(Detection, '/'+namespace+'/new_detection', 10) for namespace in TBIDS}
        for namespace in TBIDS: 
            self.logg(namespace)
            self.create_service(GoalPose, '/'+namespace+'/goal_assignment', self.assignGoalCallback)
            ActionServer(self, Teamup, '/'+namespace+'/teamup', self.teamupActionCallback)
        self.logg('Initialized\n'.upper()*5)
        self.get_logger().set_level(LOGGLEVEL)

    def receiveMapCallback(self, ogMsg):
        self.logg('recevied map\n'*10)
        self.occGrid = ogMsg
        self.wBorders = u.findWBorders(ogMsg)
        self.hBorders = u.findHBorders(ogMsg)
        self.logg(f'wBorders:\n{self.wBorders}\nhBorders:\n{self.hBorders}'.replace('),', '\n'))
        self.flagPublisher.publish(JustBool(green=True))

    def detectionCallback(self, dMsg):
        self.logg('marker detection received: %s'%(dMsg))
        self.pubTable[dMsg.tb_id].publish(dMsg)
        if dMsg.accepted and dMsg.marker_id == 4:
            assist_id = random.choice([id for id in self.pubTable.keys() if u.convertNamespaceID(id, int) != dMsg.marker_id])
            self.pubTable[assist_id].publish(dMsg)

    def assignGoalCallback(self, request, response):
        response.goal_pose = Pose(position=self.generateGoal(u.convertNamespaceID(request.tb_id, int)))
        return response

    def teamupActionCallback(self, goalHandle):
        self.waitingActors += 1
        self.logg('another firefighter teamed up. Count: %s'%(self.waitingActors))
        while self.waitingActors < 2:
            time.sleep(1.0)
        self.logg('big fire handled. Releasing bots')
        goalHandle.succeed()
        self.waitingActors = 0
        return Teamup.Response()

    def generateGoal(self, i):
        wmin, wmax = self.wBorders[1], self.wBorders[2]
        hmin, hmax = self.hBorders[1], self.hBorders[2]
        dW = (wmax - wmin)//2
        dH = (hmax - hmin)//2
        self.logg('%s, %s, %s, %s'%(wmin, wmax, hmin, hmax))
        w = random.randint(wmax - dW*(i+1), wmax - dW*i)
        h = random.randint(hmax - dH*(i+1), hmax - dH*i)
        self.logg('w, h = %s %s'%(w, h))
        point = u.mapToRealPoint(self.occGrid, w, h)
        self.logg('\n\nas real point: generated point %s'%(point)+'\n\n')
        return point

    def logg(self, *obj):
        self.get_logger().info(u.logg(self.name, *obj))

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