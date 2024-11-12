from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

class FrameListener(Node):
    
    def __init__(self):
        super().__init__('robot_frame_listener')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 1)
        
        self.state = 'forward'
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.duration = 2  
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        msg = Twist()
        
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        if self.state == 'forward':
            msg.linear.x = 0.5
            if elapsed_time > self.duration:
                self.state = 'u_turn'
                self.start_time = current_time

        elif self.state == 'u_turn':
            msg.angular.z = 0.1
            if elapsed_time > self.duration:
                self.state = 'backward'
                self.start_time = current_time

        elif self.state == 'backward':
            msg.linear.x = -0.5
            if elapsed_time > self.duration:
                self.state = 'u_turn_back'
                self.start_time = current_time

        elif self.state == 'u_turn_back':
            msg.angular.z = 0.1
            if elapsed_time > self.duration:
                self.state = 'forward'
                self.start_time = current_time

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()