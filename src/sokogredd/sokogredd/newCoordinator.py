import rclpy
import time
import sokogredd.utils as u
import random
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, Point
from detection.msg import Detection
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from interfjeses.msg import JustBool
from interfjeses.srv import GoalPose
from interfjeses.action import Teamup

DISTANCE_THRESHOLD = 1.5

TBIDS = ['tb3_0', 'tb3_1']

EDGETUPLE = (-10, 10)

class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        self.name = 'coordinator'
        self.occGrid = None
        self.wBorders, self.hBorders = list(), list()
        self.wmin, self.wmax, self.hmin, self.hmax, self.dw, self.dh = 0, 0, 0, 0, 0, 0
        self.waitingActors, self.releasedWaiters = 0, 0
        self.markers = dict()
        self.startTime = time.time()
        self.aruco4OK = False
        self.defaultPoint = Point()
        self.teamuplock = threading.Lock()
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                        depth=5)
        self.create_timer(60*10.0, self.timesup)
        self.create_subscription(OccupancyGrid, '/map', self.receiveMapCallback, qos_profile)
        self.create_subscription(Detection, '/true_detection', self.detectionCallback, 10)
        self.flagPublisher = self.create_publisher(JustBool, '/run_flag', 10)
        self.pubTable = {namespace: self.create_publisher(Detection, '/'+namespace+'/new_detection', 10) for namespace in TBIDS}
        self.reEntrant = ReentrantCallbackGroup()
        self.goalAssigner = self.create_service(GoalPose, '/goal_assignment', self.assignGoalCBStratB, callback_group=self.reEntrant)
        self.teamupAction = ActionServer(self, Teamup, '/teamup', self.teamupActionCallback, callback_group=self.reEntrant)
        self.logg('Initialized\n'.upper()*5)

    def timesup(self):
        sinceEpochDelta = time.time() - self.startTime
        mins = round(sinceEpochDelta//60)
        secs = round(sinceEpochDelta%60)
        mstr = '%s:%s'%(mins, secs)
        self.logg('TIMES UP AT %s\n%s'%(mstr, str(self.markers).replace(',', '\n')))
        self.flagPublisher.publish(JustBool(green=False))
        return
        

    def receiveMapCallback(self, ogMsg):
        self.logg('recevied map')
        self.occGrid = ogMsg
        self.wBorders = u.findWBorders(ogMsg)
        self.hBorders = u.findHBorders(ogMsg)
        self.wmin, self.wmax = u.findMinMax(self.hBorders) # yes, they are switched
        self.hmin, self.hmax = u.findMinMax(self.wBorders) # yes, they are switched
        self.logg
        assert self.wmin < self.wmax
        assert self.hmin < self.hmax
        self.dw = self.wmax - self.wmin
        self.dh = self.hmax - self.hmin
        self.flagPublisher.publish(JustBool(green=True))
        self.logg('published flag')

    def detectionCallback(self, dMsg):
        self.logg('marker detection received: %s'%(dMsg))
        self.pubTable[dMsg.tb_id].publish(dMsg)
        if not dMsg.accepted:
            return
        sinceEpochDelta = time.time() - self.startTime
        mins = round(sinceEpochDelta//60)
        secs = round(sinceEpochDelta%60)
        mstr = '%s:%s'%(mins, secs)
        self.markers[dMsg.marker_id] = mstr
        self.logg('\n%s: [%s]\n'%(dMsg.marker_id, mstr))
        if self.testDone():
            return
        if dMsg.marker_id == 4:
            assist_id = random.choice([id for id in self.pubTable.keys() if u.convertNamespaceID(id, str) != dMsg.tb_id])
            self.pubTable[assist_id].publish(dMsg)

    def teamupActionCallback(self, goalHandle):
        with self.teamuplock:
            self.waitingActors += 1
        self.logg(str(self.waitingActors) + ' guys have teamed up')
        self.aruco4OK = self.waitingActors >= 2
        while not self.aruco4OK:
            time.sleep(0.5)
        self.testDone()
        goalHandle.succeed()
        self.logg('big fire handled. Releasing bot')
        return Teamup.Result()

    def assignGoalCBStratA(self, request, response):
        response.goal_pose = Pose(position=self.generateGoal(u.convertNamespaceID(request.tb_id, int)))
        return response

    def assignGoalCBStratB(self, request, response):
        i = u.convertNamespaceID(request.tb_id, int)
        if i == 0:
            p = self.generateGoal(random.choice((0, 1)))
            p.x, p.y = p.x*0.5, p.y*0.5
            response.goal_pose = Pose(position=p)
            return response
        else:
            response.goal_pose = Pose(position=self.sendToEdge())
        return response

    def sendToEdge(self):
        x = float((-1)**random.randint(0, 1)*10)
        y = float((-1)**random.randint(0, 1)*10)
        return Point(x=x, y=y)

    def generateGoal(self, i):
        point = self.defaultPoint
        w = random.randint(self.wmin + (self.dw//2)*i, self.wmin + (self.dw//2)*(i+1))
        hlimin, hlimax = self.hBorders[w]
        h = random.randint(hlimin, hlimax)
        point = u.mapToRealPoint(self.occGrid, w, h)
        self.logg('point generated: %s, %s'%(round(point.x, 2), round(point.y, 2)))
        return point
    
    def testDone(self):
        if len(self.markers) >= 5 and self.aruco4OK:
            self.logg('all done\n'.upper()*20 + str(self.markers).replace(',', '\n'))
            self.flagPublisher.publish(JustBool(green=False))
            return True
        return False

    def logg(self, *obj):
        self.get_logger().info(u.logg(self.name, *obj))

def main(args=None):
    try:
        rclpy.init(args=args)
        executor = MultiThreadedExecutor()
        controller = Coordinator()
        executor.add_node(controller)
        executor.spin()
    except KeyboardInterrupt:
        rclpy.shutdown()
        pass

if __name__ == '__main__':
    main()