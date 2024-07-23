import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from turtlesim.msg import Pose

class TurtleServicePractice(Node):
    def __init__(self):
        super().__init__('service_practice')
        self.client = self.create_client(SetPen, '/turtle1/set_pen')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.req = SetPen.Request()

    def pose_callback(self, msg):
        if msg.x > 5.5:
            self.change_color(255, 0, 0)  # Red
            self.get_logger().info('Color: Red')
        else:
            self.change_color(0, 255, 0)  # Green
            self.get_logger().info('Color: Green')

    def change_color(self, r, g, b):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req.r = r
        self.req.g = g
        self.req.b = b
        self.req.width = 3
        self.req.off = 0
        future = self.client.call_async(self.req)
        future.add_done_callback(self.callback_color_change)

    def callback_color_change(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service call succeeded: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleServicePractice()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
