import rclpy
from rclpy.node import Node
from simcharts_interfaces.srv import GetStaticObstacles, GetDynamicObstacles


class StaticObstacleClient(Node):
    def __init__(self):
        super().__init__('simcharts__static_obstacle_client')
        self.cli = self.create_client(GetStaticObstacles, 'simcharts__get_static_obstacles')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self):
        req = GetStaticObstacles.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Result of get_static_obstacles: %s' % response)
            return response.static_obstacles
        self.get_logger().info('Exception while calling service: %r' % future.exception())
        return

class DynamicObstacleClient(Node):
    def __init__(self):
        super().__init__('simcharts__dynamic_obstacle_client')
        self.cli = self.create_client(GetDynamicObstacles, 'simcharts__get_dynamic_obstacles')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self):
        req = GetDynamicObstacles.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Result of get_dynamic_obstacles: %s' % response)
            return response.dynamic_obstacles
        self.get_logger().info('Exception while calling service: %r' % future.exception())
        return

def main():
    rclpy.init()
    # soc = StaticObstacleClient()
    doc = DynamicObstacleClient()
    dyn_obst_polygons = doc.send_request()
    print(f"\n\nland_list: {dyn_obst_polygons}\n\n")
    doc.destroy_node()

if __name__ == '__main__':
    main()