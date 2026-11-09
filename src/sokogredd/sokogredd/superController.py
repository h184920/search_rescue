"""
superController: styrer begge robotene (tb3_0 og tb3_1).

Hvor stegene ligger:
  __init__        -> 1 (abonner på kartet), 3, publishere til 4/6/7, timer for main loop
  map_callback    -> 1
  del_ut_punkter  -> main loop, 2, 4, 6d, 7
  marker_callback -> 5a, 5b, 5c, 6a, 6b, 6c
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers
from scoring_interfaces.srv import SetMarkerPosition

from sokogredd import ViewPoints
from sokogredd.treeMaker import TreeMaker

ROBOTER = ('tb3_0', 'tb3_1')
BIG_FIRE_ID = 4
HUMAN_ID = 2
ANTALL_MARKORER = 5
CELLE_STORRELSE = 10    # samme tall som ViewPoints bruker i ned_skaler_grid
FRAMME_AVSTAND = 0.3    # meter: roboten regnes som framme ved et utsiktspunkt
FLAMME_AVSTAND = 1.5    # meter: steg 6d, litt under 2 m for å ha margin


def avstand(a, b):
    """Avstanden i meter mellom to punkter (x, y)."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def to_map(tf, p):
    """Gjør punktet p (relativt til kameraet) om til kartkoordinater med transformen tf.
    Roterer punktet med kvaternionen i tf og legger til forflytningen i tf."""
    q, t = tf.transform.rotation, tf.transform.translation
    tx = 2 * (q.y * p.z - q.z * p.y)
    ty = 2 * (q.z * p.x - q.x * p.z)
    tz = 2 * (q.x * p.y - q.y * p.x)
    x = p.x + q.w * tx + (q.y * tz - q.z * ty)
    y = p.y + q.w * ty + (q.z * tx - q.x * tz)
    z = p.z + q.w * tz + (q.x * ty - q.y * tx)
    return x + t.x, y + t.y, z + t.z


class superController(Node):
    def __init__(self) -> None:
        super().__init__('superController')

        # Tilstand for ruta (steg 1, 2 og 4)
        self.tree = None                                  # TreeMaker: id -> (punkt, neste_id)
        self.neste_id = None                              # bokmerke: neste punkt i kjeden
        self.posisjon = {}                                # robot -> (x, y) i meter
        self.maal = {robot: None for robot in ROBOTER}    # robot -> nåværende mål (x, y)

        # Tilstand for markørene (steg 5 og main loop)
        self.rapportert = set()                           # ID-er poengnoden har godtatt
        self.venter_svar = set()                          # ID-er som er sendt, men ikke besvart
        self.ferdig = False                               # True når steg 7 er gjort

        # Tilstand for den store flammen (steg 6)
        self.big_fire_funnet = False
        self.flamme_punkt = None                          # (x, y) i meter
        self.flamme_ferdig = False                        # True når 6d er oppfylt
        self.avbrutt = {}                                 # målene robotene hadde før flammen

        # TF: brukes i steg 5a
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 1. Read the blueprint map (/map) ...
        #    map_server sender kartet bare én gang, derfor TRANSIENT_LOCAL
        map_qos = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        # 3. Subscribe to both robots' ArUco detectors and odometry.
        self.create_subscription(ArucoMarkers, '/tb3_0/aruco_markers', self.tb3_0_callback, 10)
        self.create_subscription(ArucoMarkers, '/tb3_1/aruco_markers', self.tb3_1_callback, 10)
        self.create_subscription(Odometry, '/tb3_0/odom', self.tb3_0_odom_callback, 10)
        self.create_subscription(Odometry, '/tb3_1/odom', self.tb3_1_odom_callback, 10)

        # Publishere for mål (steg 4 og 6b): Bug2-noden til hver robot lytter på /tb3_X/goal
        self.tb3_0_goal_pub = self.create_publisher(PointStamped, '/tb3_0/goal', 10)
        self.tb3_1_goal_pub = self.create_publisher(PointStamped, '/tb3_1/goal', 10)

        # Publishere for fart (steg 7): null fart stopper roboten
        self.tb3_0_cmd_pub = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)
        self.tb3_1_cmd_pub = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)

        # Klient for poengnoden (steg 5b)
        self.scoring_client = self.create_client(SetMarkerPosition, '/set_marker_position')

        # Main loop (until all 5 markers are reported)
        #    En ROS-node har ingen while-løkke: timeren kjører del_ut_punkter to ganger i sekundet
        self.create_timer(0.5, self.del_ut_punkter)

    # =====================================================================
    # Setup
    # =====================================================================

    def map_callback(self, msg):
        # 1. Read the blueprint map (/map) and get the viewpoints, (from treeMaker.py) that see the walls.
        #    Merk: ViewPoints.py lager punktene, treeMaker.py bestemmer rekkefølgen.
        self.grid = ViewPoints.kart_til_grid(msg, CELLE_STORRELSE)       # kartet -> rutenett
        self.celle_m = msg.info.resolution * CELLE_STORRELSE              # størrelsen på én rute i meter
        self.origo = (msg.info.origin.position.x, msg.info.origin.position.y)  # hjørnet (0, 0) i meter
        punkter = ViewPoints.planlegg_punkter(self.grid)                  # TODO: navnet på deres funksjon
        self.tree = TreeMaker(punkter, self.grid)                         # rekkefølgen punktene skal besøkes i
        self.neste_id = 0                                                 # start i punkt 0
        self.get_logger().info(f'Rute laget med {len(punkter)} punkter')

    def til_kart(self, punkt):
        """Utsiktspunkt (rad, kolonne) i rutenettet -> (x, y) i meter."""
        rad, kol = punkt
        x = self.origo[0] + (kol + 0.5) * self.celle_m
        y = self.origo[1] + (rad + 0.5) * self.celle_m
        return x, y

    # =====================================================================
    # Main loop
    # =====================================================================

    def del_ut_punkter(self):
        # Main loop (until all 5 markers are reported)
        if self.ferdig or self.tree is None:
            return

        # End
        # 7. Stop both robots.
        if len(self.rapportert) == ANTALL_MARKORER:
            self.stopp_robotene()
            return

        # 6. If ID 4 (big fire):
        #    d. When both robots are within 2 m: continue with the remaining viewpoints.
        if self.flamme_punkt is not None and not self.flamme_ferdig:
            begge_framme = True
            for robot in ROBOTER:
                if robot not in self.posisjon or avstand(self.posisjon[robot], self.flamme_punkt) > FLAMME_AVSTAND:
                    begge_framme = False
            if begge_framme:
                self.flamme_ferdig = True
                self.get_logger().info('Begge robotene er ved den store flammen')
                # Fortsett mot utsiktspunktet hver robot var på vei til, eller ta et nytt
                for robot in ROBOTER:
                    if self.avbrutt.get(robot) is not None:
                        self.send_goal(robot, *self.avbrutt[robot])
                    else:
                        self.maal[robot] = None
            return   # ruta står på pause til begge er ved flammen

        # 2. Split the viewpoints between tb3_0 and tb3_1.
        #    Merk: punktene deles ut ett og ett, i TreeMaker-rekkefølgen, til den roboten som er ledig.
        # 4. Each robot drives to its next viewpoint using Bug2.
        #    Merk: superController sender målet, Bug2-noden til roboten kjører dit.
        for robot in ROBOTER:
            if robot not in self.posisjon or self.neste_id is None:
                continue
            ledig = self.maal[robot] is None or avstand(self.posisjon[robot], self.maal[robot]) < FRAMME_AVSTAND
            if ledig:
                punkt, neste = self.tree[self.neste_id]    # les neste punkt fra TreeMaker
                self.neste_id = neste                      # flytt bokmerket
                x, y = self.til_kart(punkt)                # rutenett -> meter
                self.send_goal(robot, x, y)                # send målet til roboten

    def send_goal(self, robot, x, y):
        """Sender målet (x, y) i kartet til Bug2-noden til roboten (steg 4 og 6)."""
        goal = PointStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.point.x = float(x)
        goal.point.y = float(y)
        if robot == 'tb3_0':
            self.tb3_0_goal_pub.publish(goal)
        else:
            self.tb3_1_goal_pub.publish(goal)
        self.maal[robot] = (x, y)

    def stopp_robotene(self):
        # 7. Stop both robots.
        self.tb3_0_cmd_pub.publish(Twist())   # Twist() = null fart
        self.tb3_1_cmd_pub.publish(Twist())
        self.ferdig = True
        self.get_logger().info('Alle markørene er rapportert. Robotene stoppes.')

    # =====================================================================
    # 3. Subscribe to both robots' ArUco detectors and odometry. (callbackene)
    # =====================================================================

    def tb3_0_odom_callback(self, msg):
        p = msg.pose.pose.position
        self.posisjon['tb3_0'] = (p.x, p.y)   # odom = kartkoordinater i kursets oppsett

    def tb3_1_odom_callback(self, msg):
        p = msg.pose.pose.position
        self.posisjon['tb3_1'] = (p.x, p.y)

    def tb3_0_callback(self, msg):
        self.marker_callback(msg, 'tb3_0')

    def tb3_1_callback(self, msg):
        self.marker_callback(msg, 'tb3_1')

    # =====================================================================
    # 5. When a robot detects a marker:
    # =====================================================================

    def marker_callback(self, msg, robot):
        # 5. When a robot detects a marker:
        #    a. Transform the marker position from the camera frame to the map frame (TF).
        #       Henter hvor kameraet står og hvilken vei det peker i kartet akkurat nå
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', f'{robot}/camera_rgb_optical_frame', Time())
        except TransformException as e:
            self.get_logger().warn(str(e))
            return

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            x, y, _ = to_map(tf, pose.position)   # markørens posisjon: kamera -> kart

            #    b. If the ID is not reported yet: send ID + position to the scoring service.
            self.rapporter(marker_id, x, y)

            #    c. Classify: ID 4 = big fire, ID 2 = human, ID 0/1/3 = small fire.
            if marker_id == BIG_FIRE_ID:
                type_markor = 'big fire'
            elif marker_id == HUMAN_ID:
                type_markor = 'human'
            else:
                type_markor = 'small fire'

            # 6. If ID 4 (big fire):
            if marker_id == BIG_FIRE_ID and not self.big_fire_funnet:
                self.big_fire_funnet = True
                self.flamme_punkt = (x, y)
                self.avbrutt = dict(self.maal)   # husk utsiktspunktene robotene var på vei til
                self.get_logger().info(f'{robot} fant {type_markor} på ({x:.2f}, {y:.2f})')

                #    a. The detecting robot drives to within 1.5 m of the marker and stops.
                #       Merk: stoppen ved 1.5 m gjøres av Bug2-noden (stoppavstand).
                self.send_goal(robot, x, y)

                #    b. Publish the marker position to the other robot.
                other = 'tb3_1' if robot == 'tb3_0' else 'tb3_0'
                self.send_goal(other, x, y)

                #    c. The other robot drives to the marker position using Bug2 and stops within 1.5 m.
                #       Merk: dette gjør Bug2-noden til den andre roboten når den får målet over.

    def rapporter(self, marker_id, x, y):
        # 5b. If the ID is not reported yet: send ID + position to the scoring service.
        if marker_id in self.rapportert or marker_id in self.venter_svar:
            return                                        # allerede rapportert eller sendt
        if not self.scoring_client.service_is_ready():
            return                                        # poengnoden kjører ikke ennå
        req = SetMarkerPosition.Request()
        req.marker_id = int(marker_id)
        req.marker_position.x = float(x)
        req.marker_position.y = float(y)
        req.marker_position.z = 0.0                       # markørene står på gulvet i kartet
        self.venter_svar.add(marker_id)
        future = self.scoring_client.call_async(req)
        future.add_done_callback(lambda f, mid=marker_id: self.rapport_svar(f, mid))

    def rapport_svar(self, future, marker_id):
        """Svaret fra poengnoden: godtatt, eller avvist (da prøver vi igjen neste gang den sees)."""
        self.venter_svar.discard(marker_id)
        svar = future.result()
        if svar is not None and svar.accepted:
            self.rapportert.add(marker_id)
            self.get_logger().info(f'ID {marker_id} godtatt ({len(self.rapportert)}/{ANTALL_MARKORER})')
        else:
            self.get_logger().warn(f'ID {marker_id} ble ikke godtatt, prøver igjen')


def main(args=None):
    rclpy.init(args=args)
    controller = superController()
    rclpy.spin(controller)   # holder noden i gang og kjører callbackene og timeren
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()