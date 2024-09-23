import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import argparse

from action_turtle_message.action import MessageTurtleCommands


class MessageTurtleCommandsClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'MessageTurtleCommands')

    def send_goal(self, s, angle):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.odom))


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Send goal to TurtleSim Action server.')
    parser.add_argument('s', type=int, help='The distance value to send as goal.')
    parser.add_argument('angle', type=int, help='The angle value to send as goal.')
    parsed_args = parser.parse_args()

    action_client = MessageTurtleCommandsClient()    

    action_client.send_goal(parsed_args.s, parsed_args.angle)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()