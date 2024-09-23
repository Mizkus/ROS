import rclpy
from rclpy.node import Node
from full_name_pkg.srv import FullNameSumService

class FullNameServiceNode(Node):
    def __init__(self):
        super().__init__('service')
        self.srv = self.create_service(FullNameSumService, 'SummFullName', self.sum_full_name_callback)

    def sum_full_name_callback(self, request, response):
        response.full_name = f"{request.last_name} {request.name} {request.first_name}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FullNameServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
