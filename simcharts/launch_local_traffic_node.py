#!/usr/bin/env conda activate simcharts_env
import rclpy
import threading
import simcharts.utils as utils
from simcharts.enc import ENC
from simcharts.nodes import LocalTrafficNode
from simcharts_aisforwarder.nodes import AISpublisher

def main():
    rclpy.init()
    config = utils.config.SeaChartsConfig(utils.paths.config)
    publish_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

    local_traffic_node = LocalTrafficNode(config.settings['enc'], cli_args=None)
    ais_forwarder = AISpublisher(publish_callback_group)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ais_forwarder)
    executor.add_node(local_traffic_node)

    try:
        while True:
            executor.spin_once(timeout_sec=0.01)
    except KeyboardInterrupt:
        local_traffic_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()