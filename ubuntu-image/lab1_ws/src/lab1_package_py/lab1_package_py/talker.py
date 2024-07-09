import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('talker')

        self.declare_parameter('v', 1.0)
        self.declare_parameter('d', 2.0)

        self.v = self.get_parameter('v').get_parameter_value().double_value
        self.d = self.get_parameter('d').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.v
        msg.drive.steering_angle = self.d

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: speed = {self.v}, steering angle = {self.d}')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()