import rclpy
from rclpy.node import Node
import sys
import math

from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry


class robot_go_to_goal(Node):

    def __init__(self):
        super().__init__('Go_to_goal_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, "/odom", self.pos_fun, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.go_to_goal_fun)
        self.robot_pos = Point()
        self.goal_pos = Point()
        self.angle_e= 0.0 
        self.dist_e = 0.0 
        self.vel_msg = Twist()
        self.angle_to_turn = 0.0

    def pos_fun(self,data):
        
        self.robot_pos.x = data.pose.pose.position.x
        self.robot_pos.y = data.pose.pose.position.y
        
        quat = data.pose.pose.orientation
        
        (roll, pitch, yaw) = self.euler_from_quat(quat.x, quat.y, quat.z, quat.w)
        self.robot_pos.z = yaw
        

    def go_to_goal_fun(self):
        self.goal_pos.x = float(sys.argv[1])
        self.goal_pos.y = float(sys.argv[2])
        self.angle_offset = float(sys.argv[3])
        
        self.dist_e = math.sqrt(math.pow((self.goal_pos.x - self.robot_pos.x),2) + math.pow((self.goal_pos.x - self.robot_pos.x),2))
        
        self.angle_e = math.atan2((self.goal_pos.y - self.robot_pos.y) , (self.goal_pos.x - self.robot_pos.x)) + self.angle_offset

        

        self.angle_to_turn = self.angle_e - self.robot_pos.z

        if self.dist_e <= 0.1:
            self.get_logger().info("Stop")
            self.vel_msg.linear.x= 0.0
            self.vel_msg.angular.z = 0.0

        elif abs(self.angle_to_turn) > 0.2:
            self.get_logger().info("Turn")
            self.vel_msg.angular.z = self.angle_to_turn
            self.vel_msg.linear.x = 0.0
        
        else:
            self.get_logger().info("Go")
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.x = self.dist_e
            
        
        msg = 'Distance: {:3f} Angle: {:3f}'.format(self.dist_e, self.angle_to_turn)
        self.get_logger().info(msg)

        self.publisher_.publish(self.vel_msg)
    
    def euler_from_quat(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)

    go_to_goal = robot_go_to_goal()

    rclpy.spin(go_to_goal)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_go_to_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()