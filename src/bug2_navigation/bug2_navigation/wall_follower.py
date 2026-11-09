import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

class wall_follower(Node):
    def __init__(self) -> None:
        super().__init__("wall_follower")

        #for service 
        self.aktiv = False 
        self.service = self.create_service(SetBool, "wall_follower_bryter", self.bytt_til_GTP)


        self.snu_punkt = 0.5 #hvor nær vi skal være veggen før vi snur
        self.fant_vegg = False
        self.vegg_margin = 1.5


        self.sub = self.create_subscription(LaserScan, "/scan", self.clbk_laser, 10)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        #base 
        self.lidar_diagonal = 10
        self.lidar_right = 10
        self.lidar_front = 10

        #Creates a timer definition -> during rclpy.spin() the function self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def clbk_laser(self, msg):
        self.lidar_front = msg.ranges[0]
        self.lidar_right = msg.ranges[270]
        self.lidar_diagonal = msg.ranges[315] 

    def bytt_til_GTP(self, request, response):
        self.aktiv = request.data
        response.success = True 
        response.message = f"Wall follower er: {'Aktiv' if self.aktiv else 'Pauset'}"
        return response


    def timer_callback(self):
        if not self.aktiv:
            return 

        vel_msg = Twist()

        if self.lidar_front < self.snu_punkt:
            self.fant_vegg = True
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5

        #følg veggen 
        elif self.lidar_right < self.vegg_margin * self.snu_punkt: #tweak men børr være ca bra
            self.fant_vegg = True 

            vinkel_feil = self.lidar_right * (self.vegg_margin - 0.1) - self.lidar_diagonal # om 0 så skal vi følge veggen / er parralell
            avstand_igjen = self.snu_punkt - self.lidar_right

            vel_msg.angular.z = max(-0.5, min(0.5, (vinkel_feil + avstand_igjen * 0.8))) #insane om fungerer, men kult om går 
            vel_msg.linear.x = 0.2

        #kjør bue rundt vegg
        elif self.fant_vegg:

            vel_msg.linear.x = 0.5
            vel_msg.angular.z = -0.05

        else: 
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0.0

        

        self.pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)

    controller = wall_follower()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()