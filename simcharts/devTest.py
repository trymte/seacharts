import rclpy
from rclpy.node import Node
import numpy as np
from simcharts_interfaces.srv import GetStaticObstacles, GetDynamicObstacles, DrawPath
from simcharts_interfaces.msg import Path


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

class DrawPathClient(Node):
    def __init__(self):
        super().__init__('simcharts__draw_path_client')
        self.cli = self.create_client(DrawPath, 'simcharts__draw_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self, id=None):
        req = DrawPath.Request()
        req.path = self.generatePath()
        req.id = str(np.random.randint(0, 100000)) if id == None else id
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Result of get_dynamic_obstacles: %s' % response)
            return response
        self.get_logger().info('Exception while calling service: %r' % future.exception())
        return
    
    def generatePath(self):
        size = [900.0, 506.20]
        origin = [569042.27, 7034900.20]     
        path_x = []
        path_y = []
        path_psi = []

        # Generate a path within the origin and size
    
        prev_x = 0
        prev_y = 0
        for i in range(10):
            rand_x = np.random.randint(prev_x, size[0]-(9-i))
            rand_y = np.random.randint(prev_y, size[1]-(9-i))
            path_x.append(origin[0] + rand_x)
            path_y.append(origin[1] + rand_y)
            path_psi.append(0.0)
            prev_x = rand_x
            prev_y = rand_y
        path = Path()
        path.x = path_x
        path.y = path_y
        path.psi = path_psi
        return path

def main():
    rclpy.init()
    # soc = StaticObstacleClient()
    # doc = DynamicObstacleClient()
    id = str(np.random.randint(0, 100000))
    draw = DrawPathClient()
    ret = draw.send_request(id)
    print(f"\n\nland_list: {ret}\n\n")
    draw.destroy_node()

    # Wait for user to press enter
    input("Press enter to continue...")
    
    draw2 = DrawPathClient()
    ret = draw2.send_request(id)
    print(f"\n\nland_list: {ret}\n\n")
    draw.destroy_node()

if __name__ == '__main__':
    main()