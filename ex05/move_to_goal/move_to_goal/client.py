import sys
import rclpy
from rclpy.node import Node
from move_message.srv import MoveToGoal

class MoveToGoalClient(Node):
    def __init__(self):
        super().__init__('client')
        self.client = self.create_client(MoveToGoal, 'MoveToGoal')

    def send_request(self, x, y, theta):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, exiting...')
            return

        request = MoveToGoal.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().result:
            self.get_logger().info(f'Result: :)\n x: {future.result().x}, y: {future.result().y}, theta: {future.result().theta}')
        else:
            self.get_logger().info(f'Reuslt: :(')


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal client <x> <y> <theta>")
        return

    x= sys.argv[1]
    y = sys.argv[2]
    theta = sys.argv[3]

    client = MoveToGoalClient()
    client.send_request(float(x), float(y), float(theta))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
