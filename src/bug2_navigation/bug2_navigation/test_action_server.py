import rclpy
from rclpy.node import Node 
from rclpy.action import ActionClient
from geometry_msgs.msg import Point 
from bug2_interfaces.action import Bug2 

class test_action_server(Node):
    def __init__(self) -> None:
        super().__init__("test_action_server")

        #Her jonas vil det være naturlig å mate inn de ulike punktene 
        self.declare_parameter("goal_x", -3.5)
        self.declare_parameter("goal_y", 10.0)

        self.action_client = ActionClient(self, Bug2, "bug2")

    def send_maal(self):
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)

        goal = Bug2.Goal()

        goal.target_position = Point(x=goal_x, y=goal_y, z=0.0)

        self.get_logger().info("Nå må me vente på at mr bug2 action server skal svare oss....")
        self.action_client.wait_for_server()

        self.get_logger().info(f"Vi har fått kontakt, sender punktet: ({goal_x},{goal_y})")
        future = self.action_client.send_goal_async(goal, self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info(f"Målpunktet ikke godkjent")
            return

        self.get_logger().info("Målpunktet ble akkseptert.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        punkt = feedback_msg.feedback.current_position 
        self.get_logger().info(f"Vi er nå på posisjon: ({punkt.x},{punkt.y})")

    def result_callback(self, future):
        punkt = future.result().result.base_position
        self.get_logger().info(f"Ferdig, me endte på: ({punkt.x},{punkt.y})")
        rclpy.shutdown()
    


def main(args=None):
    rclpy.init(args=args)

    controller = test_action_server()
    controller.send_maal()

    rclpy.spin(controller)



if __name__ == '__main__':
    main()