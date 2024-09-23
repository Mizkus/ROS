import rclpy
import time
from math import sqrt, pi
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_turtle_message.action import MessageTurtleCommands
from multiprocessing import Process, Queue

class PoseSubscriber(Node):

    def __init__(self, pose_queue):
        super().__init__('pose_subscriber')
        self.pose_queue = pose_queue
        self._pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        if not self.pose_queue.full():
            self.pose_queue.put(msg)
        else:
            self.pose_queue.get()
            self.pose_queue.put(msg)

def start_pose_listener(pose_queue):
    rclpy.init()
    pose_subscriber = PoseSubscriber(pose_queue)
    rclpy.spin(pose_subscriber)
    rclpy.shutdown()

class MessageTurtleCommandsActionServer(Node):

    def __init__(self, pose_queue):
        super().__init__('MessageTurtleCommands_action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'MessageTurtleCommands',
            self.execute_callback)
        self._cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_queue = pose_queue

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0

        curr_pose = self.pose_queue.get()

        params = goal_handle.request
        self.get_logger().info(f"s: {params.s}, angle: {params.angle}")

        twist_msg = Twist()
        twist_msg.linear.x = float(params.s)
        twist_msg.angular.z = float(params.angle) * pi / 180


        self._cmd_vel_publisher.publish(twist_msg)
        time.sleep(1)

        current_pose = self.pose_queue.get()


        odom_distance = sqrt((curr_pose.x - current_pose.x) ** 2 +
                             (curr_pose.y - current_pose.y) ** 2)

        feedback_msg.odom = int(odom_distance)
        self.get_logger().info(f'Odometry distance: {feedback_msg.odom}')

        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = MessageTurtleCommands.Result()
        result.result = True

        return result

def main(args=None):
    pose_queue = Queue(maxsize=1)
    
    pose_listener_process = Process(target=start_pose_listener, args=(pose_queue,))
    pose_listener_process.start()

    time.sleep(1)

    rclpy.init(args=args)

    action_server = MessageTurtleCommandsActionServer(pose_queue)

    rclpy.spin(action_server)

    pose_listener_process.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
