import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty  # Import the Empty service
from turtlesim.srv import TeleportAbsolute
import time

LIMITE_IZQUIERDO = 0.5
LIMITE_DERECHO = 10.5
LIMITE_SUPERIOR = 10.5
LIMITE_INFERIOR = 0.5

class MotionController(Node):

    def __init__(self):
        super().__init__('motion_controller')

        for i in range(5, 0, -1):
            print('Application starts in:', i)
            time.sleep(1)
        print('Initiating MotionController')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        self.teleport_turtle(5.0, 5.0, 0.0)

        # Clear the screen
        self.clear_screen()

        self.timer_ = self.create_timer(0.1, self.mover_tortuga)
        self.distancia = 0.05
        self.es_perform_spiral = True
        self.current_pose = None

    def teleport_turtle(self, x, y, theta):
        self.cli = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')
        self.req = TeleportAbsolute.Request()
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

    def clear_screen(self):
        self.clear_cli = self.create_client(Empty, 'clear')
        while not self.clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear service not available, waiting again...')
        self.clear_req = Empty.Request()
        self.clear_future = self.clear_cli.call_async(self.clear_req)
        rclpy.spin_until_future_complete(self, self.clear_future)

    def pose_callback(self, msg):
        self.current_pose = msg

    def mover_tortuga(self):
        vel_msg = Twist()

        if self.current_pose is None:
            return

        x = self.current_pose.x
        y = self.current_pose.y

        if self.es_perform_spiral:
            vel_msg.linear.x = self.distancia
            vel_msg.angular.z = 1.0
            self.distancia += 0.01
            self.get_logger().info("Drawing spiral")
        else:
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 0.0
            self.get_logger().info("Going straight")

        self.publisher_.publish(vel_msg)

        if self.es_perform_spiral:
            if x < LIMITE_IZQUIERDO or x > LIMITE_DERECHO or y > LIMITE_SUPERIOR or y < LIMITE_INFERIOR:
                self.get_logger().info("Going straight")
                self.es_perform_spiral = False

def main(args=None):
    rclpy.init(args=args)

    motion_controller = MotionController()

    rclpy.spin(motion_controller)

    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
