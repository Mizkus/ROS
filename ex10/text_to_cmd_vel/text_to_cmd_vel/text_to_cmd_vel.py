import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TextToCmdVel(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.subscriber = self.create_subscription(String, 'cmd_text', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('text_to_cmd_vel node started.')

    def listener_callback(self, msg):
        command = msg.data
        twist = Twist()

        if command == 'turn_right':
            twist.angular.z = -1.5
            twist.linear.x = 0.0
        elif command == 'turn_left':
            twist.angular.z = 1.5
            twist.linear.x = 0.0
        elif command == 'move_forward':
            twist.linear.x = 1.0
            twist.angular.z = 0.0
        elif command == 'move_backward':
            twist.linear.x = -1.0
            twist.angular.z = 0.0
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        self.publisher.publish(twist)
        self.get_logger().info(f'Publishing: linear.x: {twist.linear.x}, angular.z: {twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
