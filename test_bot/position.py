import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('position_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, "/odom", self.pos_fun, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def pos_fun(self,data):
        
        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y
        pos_z = data.pose.pose.orientation.z
        
        msg = 'X: {:3f} Y: {:3f} Z: {:3f}'.format(pos_x, pos_y, pos_z)
        self.get_logger().info(msg)

    def timer_callback(self):
        a = 1    


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