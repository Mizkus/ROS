import sys
import rclpy
from rclpy.node import Node
from full_name_pkg.srv import FullNameSumService

class FullNameClient(Node):
    def __init__(self):
        super().__init__('client_name')
        self.client = self.create_client(FullNameSumService, 'SummFullName')

    def send_request(self, last_name, name, first_name):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, exiting...')
            return

        request = FullNameSumService.Request()
        request.last_name = last_name
        request.name = name
        request.first_name = first_name

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Full Name: {future.result().full_name}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run service_full_name client_name <last_name> <name> <first_name>")
        return

    last_name = sys.argv[1]
    name = sys.argv[2]
    first_name = sys.argv[3]

    client = FullNameClient()
    client.send_request(last_name, name, first_name)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
