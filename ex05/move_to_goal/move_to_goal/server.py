import rclpy
import time
from rclpy.node import Node
from move_message.srv import MoveToGoal
from geometry_msgs.msg import Twist
from multiprocessing import Process, Queue
from turtlesim.msg import Pose
import math

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

class MoveToGoalServer(Node):
    def __init__(self, pose_queue):
        super().__init__('service')
        self.srv = self.create_service(MoveToGoal, 'MoveToGoal', self.execute_callback)
        self._cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_queue = pose_queue

    def check_input(self, coords):
        if coords.x < 0 or coords.x > 11 or \
           coords.y < 0 or coords.y > 11:
            return False
        return True
           

    def execute_callback(self, request, response):

        if not self.check_input(request):
            response.result = False
            return response

        time.sleep(1)
        pose = self.pose_queue.get()
        x = pose.x
        y = pose.y
        theta = pose.theta

        self.get_logger().info(f"x: {request.x}, y: {request.y}, theta: {request.theta}")

        twist_msg = Twist()
        twist_msg.angular.z = -theta

        self._cmd_vel_publisher.publish(twist_msg)
        time.sleep(2)

        twist_msg.angular.z = 0.0

        s_x = request.x - x
        s_y = request.y - y


        twist_msg.linear.x = s_x
        self._cmd_vel_publisher.publish(twist_msg)
        time.sleep(2)

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = s_y
        self._cmd_vel_publisher.publish(twist_msg)
        time.sleep(2)

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = request.theta
        self._cmd_vel_publisher.publish(twist_msg)
        time.sleep(2)


        pose = self.pose_queue.get()
        
        response.x = pose.x
        response.y = pose.y
        response.theta = pose.theta

        response.result = True
        return response

def main(args=None):

    pose_queue = Queue(maxsize=1)
    
    pose_listener_process = Process(target=start_pose_listener, args=(pose_queue,))
    pose_listener_process.start()

    time.sleep(1)

    rclpy.init(args=args)
    node = MoveToGoalServer(pose_queue)
    rclpy.spin(node)
    pose_listener_process.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
