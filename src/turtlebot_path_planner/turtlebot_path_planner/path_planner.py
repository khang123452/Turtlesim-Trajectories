import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        # Declare a parameter for trajectory
        self.declare_parameter('trajectory', 'circle')
        self.trajectory = self.get_parameter('trajectory').get_parameter_value().string_value
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0
        self.angle = 0.0
        self.side_length = 2.0
        self.state = 0
        self.time_started = self.get_clock().now()
        self.total_angle_turned = 0.0
        self.circle_completed = False

    def timer_callback(self):
        msg = Twist()

        if self.trajectory == 'circle':
            msg.linear.x = 1.0
            msg.angular.z = 1.0
        elif self.trajectory == 'square':
            self.square_trajectory(msg)
        elif self.trajectory == 'infi':
            self.infinity_trajectory(msg)
        elif self.trajectory == 'star':
            self.star_trajectory(msg)
        
        self.publisher.publish(msg)
        self.time += 0.1

    def square_trajectory(self, msg):
        if self.state == 0:
            msg.linear.x = 1.0
            msg.angular.z = 0.0
            self.angle += msg.linear.x * 0.1
            if self.angle >= self.side_length:
                self.state = 1
                self.angle = 0.0
        elif self.state == 1:
            msg.linear.x = 0.0
            msg.angular.z = math.pi / 2
            self.angle += msg.angular.z * 0.1
            if self.angle >= math.pi / 2:
                self.state = 0
                self.angle = 0.0

    def infinity_trajectory(self, msg):
        if not self.circle_completed:
            msg.linear.x = 1.0
            msg.angular.z = 1.0
            delta_angle = msg.angular.z * 0.1
            self.total_angle_turned += delta_angle
            if self.total_angle_turned >= 2 * math.pi:
                self.circle_completed = True
                self.total_angle_turned = 0.0
                self.get_logger().info("First circle completed!")
        else:
            msg.linear.x = 1.0
            msg.angular.z = -1.0
            delta_angle = msg.angular.z * 0.1
            self.total_angle_turned += delta_angle
            if self.total_angle_turned <= -2 * math.pi:
                self.circle_completed = False
                self.total_angle_turned = 0.0
                self.get_logger().info("Second circle completed!")

    import math

    def star_trajectory(self, msg):
        time_elapsed = (self.get_clock().now() - self.time_started).nanoseconds / 1e9
        segment_duration = 1.0 
        turn_duration = 1.0  
        total_star_duration = 5 * (segment_duration + turn_duration)

        if time_elapsed < total_star_duration:
            phase_time = time_elapsed % (segment_duration + turn_duration)
            if phase_time < segment_duration:
                msg.linear.x = 1.0  
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = (1.2 * math.pi)  
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0 



def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the node and get the trajectory parameter
    path_planner = PathPlanner()
    
    rclpy.spin(path_planner)

    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
