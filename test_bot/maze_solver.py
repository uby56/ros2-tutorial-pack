import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import os

class Video_get(Node):
    def __init__(self):
        super().__init__('video_subscriber')

        self.subscriber = self.create_subscription(Image, '/camera1/image_raw',self.video_feed, 10)
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.maze_solving)
        self.i = 0

    def maze_solving(self):
        msg = Twist()
        
        msg.linear.x = 0.5

        self.publisher_.publish(msg)

    def video_feed(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.out.write(frame)

        cv2.imshow("output", frame)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args = args)
    image_subscriber = Video_get()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
