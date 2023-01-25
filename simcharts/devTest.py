#!/usr/bin/env conda run -n simcharts_env
import rclpy
from rclpy.node import Node
import numpy as np
from simcharts.utils.helper import *
from simcharts_interfaces.srv import GetStaticObstacles, GetDynamicObstacles, DrawPath, DrawTrajectory, AddVesselToLocalTraffic, CleanPlot, RemoveVesselFromLocalTraffic
from simcharts_interfaces.msg import Path, Trajectory, Vessel


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
        for i in range(100):
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

class DrawTrajectoryClient(Node):
    def __init__(self):
        super().__init__('simcharts__draw_trajectory_client')
        self.cli = self.create_client(DrawTrajectory, 'simcharts__draw_trajectory')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self, id=None):
        req = DrawTrajectory.Request()
        req.trajectory = self.generateTrajectory()
        req.id = str(np.random.randint(0, 100000)) if id == None else id
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Result of get_dynamic_obstacles: %s' % response)
            return response
        self.get_logger().info('Exception while calling service: %r' % future.exception())
        return
    
    def generateTrajectory(self):
        size = [900.0, 506.20]
        origin = [569042.27, 7034900.20]     
        traj_x = []
        traj_y = []
        traj_psi = []
        traj_t = []

        # Generate a path within the origin and size
    
        prev_x = 0
        prev_y = 0
        prev_psi = 0.0
        prev_t = 0.0
        len = 250
        for i in range(len):
            rand_x = np.random.randint(prev_x, size[0]-(len-1-i))
            rand_y = np.random.randint(prev_y, size[1]-(len-1-i))
            rand_psi = float(np.random.randint(0, 360))
            rand_t = float(prev_t + 0.5 + np.random.randint(0,100)/100)
            if i == 0:
                rand_t = 0.0
            traj_x.append(origin[0] + rand_x)
            traj_y.append(origin[1] + rand_y)
            traj_psi.append(rand_psi)
            traj_t.append(rand_t)
            prev_x = rand_x
            prev_y = rand_y
            prev_t = rand_t
        traj = Trajectory()
        traj.x = traj_x
        traj.y = traj_y
        traj.psi = traj_psi
        traj.t = traj_t
        return traj

class AddVesselClient(Node):
    def __init__(self):
        super().__init__('simcharts__add_vessel_client')
        self.cli = self.create_client(AddVesselToLocalTraffic, 'simcharts__add_vessel')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self, id=None):
        req = AddVesselToLocalTraffic.Request()
        req.vessel = self.generateVessel(id)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Result of get_dynamic_obstacles: %s' % response)
            return response
        self.get_logger().info('Exception while calling service: %r' % future.exception())
        return
    
    def generateVessel(self, id=None):
        vessel = Vessel()
        vessel.id = np.random.randint(0, 100000) if id == None else id
        vessel.timestamp = str(getTimeStamp())
        vessel.x = 569042.27 + 20
        vessel.y = 7034900.20 + 30
        vessel.sog = 0.0
        vessel.cog = 0.0
        vessel.heading = 0.0
        vessel.length = 55.0
        vessel.scale = vessel.length / 80.0 # 80 is the length of the default vessel in simcharts
        vessel.rot = 0.0
        vessel.name = f"Vessel {vessel.id}"
        vessel.shiptype = "vessel"
        vessel.vesselsimtype = "ghost"
        return vessel

class CleanPlotCliet(Node):
    def __init__(self):
        super().__init__('simcharts__clean_plot_client')
        self.cli = self.create_client(CleanPlot, 'simcharts__clean_plot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self, id=None):
        req = CleanPlot.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Result of get_dynamic_obstacles: %s' % response)
            return response
        self.get_logger().info('Exception while calling service: %r' % future.exception())
        return

class RemoveVesselFromLocalTrafficCliet(Node):
    def __init__(self):
        super().__init__('simcharts__remove_vessel_client')
        self.cli = self.create_client(RemoveVesselFromLocalTraffic, 'simcharts__remove_vessel')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self, id=None):
        req = RemoveVesselFromLocalTraffic.Request()
        req.id = id
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Result of get_dynamic_obstacles: %s' % response)
            return response
        self.get_logger().info('Exception while calling service: %r' % future.exception())
        return


def main():
    rclpy.init()
    # soc = StaticObstacleClient()
    # doc = DynamicObstacleClient()
    # id = str(np.random.randint(0, 100000))


    # Wait for user to press enter
    # input("Press enter to continue...")
    

    # id = str(np.random.randint(0, 100000))
    id = 1

    add_vessel_cli = AddVesselClient()
    ret = add_vessel_cli.send_request(id)

    add_vessel_cli.get_logger().info("Vessel added")
    
    # Wait for user to press enter
    input("Press enter to continue...")

    draw = DrawPathClient()
    ret = draw.send_request(id)
    draw.destroy_node()

    # Wait for user to press enter
    input("Press enter to continue...")

    clean = CleanPlotCliet()
    clean.send_request()
    clean.destroy_node()
    
    # Wait for user to press enter
    input("Press enter to continue...")

    remove = RemoveVesselFromLocalTrafficCliet()
    remove.send_request(id)
    remove.destroy_node()

    # draw_traj = DrawTrajectoryClient()
    # ret = draw_traj.send_request(id)
    # draw_traj.destroy_node()
    # draw.destroy_node()

if __name__ == '__main__':
    main()