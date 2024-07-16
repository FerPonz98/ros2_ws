import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

LIMITS = {
    'left': 1.0,
    'right': 10.0,
    'top': 10.0,
    'bottom': 1.0
}

class MotionController(Node):

    def __init__(self):
        super().__init__('topics_practice_b')
        self.start_countdown()
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_timer(0.1, self.move_turtle)
        self.distance = 0.05
        self.in_spiral = True
        self.current_pose = None

    def start_countdown(self):
        for i in range(5, 0, -1):
            print(f'Application starts in: {i}')
            time.sleep(1)
        print('Initiating MotionController')

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_turtle(self):
        if self.current_pose is None:
            return

        vel_msg = Twist()
        x, y = self.current_pose.x, self.current_pose.y

        if self.is_near_wall(x, y):
            vel_msg.angular.z = 5.0
            vel_msg.linear.x = 3.8
            self.get_logger().info("Avoiding walls")
            self.in_spiral = False
        elif self.in_spiral:
            vel_msg.linear.x = self.distance
            vel_msg.angular.z = 2.0
            self.distance += 0.05
            self.get_logger().info("Drawing spiral")
            if x < LIMITS['left'] or x > LIMITS['right'] or y > LIMITS['top'] or y < LIMITS['bottom']:
                self.get_logger().info("Going straight")
                self.in_spiral = False
        else:
            vel_msg.linear.x = 4.0
            self.get_logger().info("Going straight")
        self.publisher_.publish(vel_msg)


    def is_near_wall(self, x, y):
        return (x - 1 < LIMITS['left'] or x + 1 > LIMITS['right'] or y - 1 < LIMITS['bottom'] or y + 1 > LIMITS['top'])

def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
