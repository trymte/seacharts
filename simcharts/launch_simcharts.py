#!/usr/bin/env conda run -n simcharts_env
import rclpy
from simcharts.enc import ENC
import simcharts.utils as utils

def spinMultipleNodes(executor):
    executor.spin()

def main():
    # BUG: Plotted paths do not show up before the user moves the canvas
    rclpy.init()
    config = utils.config.SeaChartsConfig(utils.paths.config)
    
    enc_node = ENC(config, border=True, cli_args=None)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(enc_node)
    enc_node.start_sim(executor)

    enc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()