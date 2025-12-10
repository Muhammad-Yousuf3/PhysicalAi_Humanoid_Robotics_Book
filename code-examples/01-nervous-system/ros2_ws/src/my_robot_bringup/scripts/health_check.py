#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import sys

class HealthCheckNode(Node):
    def __init__(self):
        super().__init__('health_check_node')
        self.get_logger().info('Starting ROS 2 health check...')

    def run_check(self):
        self.check_ros2_environment()
        self.check_ros2_topics()
        self.check_ros2_nodes()
        self.get_logger().info('ROS 2 health check complete.')

    def check_ros2_environment(self):
        self.get_logger().info('--- Checking ROS 2 Environment ---')
        try:
            ros_distro = subprocess.check_output(['printenv', 'ROS_DISTRO']).decode().strip()
            self.get_logger().info(f'ROS_DISTRO: {ros_distro}')
            if 'humble' not in ros_distro.lower():
                self.get_logger().warn('Warning: ROS_DISTRO is not Humble. Expected Humble.')
        except Exception:
            self.get_logger().error('Error: ROS_DISTRO environment variable not set. Please source your ROS 2 setup script.')
            sys.exit(1)
        self.get_logger().info('ROS 2 environment check passed.')

    def check_ros2_topics(self):
        self.get_logger().info('--- Checking ROS 2 Topics ---')
        try:
            topics_output = subprocess.check_output(['ros2', 'topic', 'list']).decode().strip()
            self.get_logger().info('Active topics:')
            for topic in topics_output.split('\n'):
                self.get_logger().info(f'- {topic}')
            if not topics_output:
                self.get_logger().warn('No active ROS 2 topics found.')
        except Exception as e:
            self.get_logger().error(f'Error listing ROS 2 topics: {e}')
            sys.exit(1)
        self.get_logger().info('ROS 2 topics check passed.')

    def check_ros2_nodes(self):
        self.get_logger().info('--- Checking ROS 2 Nodes ---')
        try:
            nodes_output = subprocess.check_output(['ros2', 'node', 'list']).decode().strip()
            self.get_logger().info('Active nodes:')
            for node in nodes_output.split('\n'):
                self.get_logger().info(f'- {node}')
            if not nodes_output:
                self.get_logger().warn('No active ROS 2 nodes found.')
        except Exception as e:
            self.get_logger().error(f'Error listing ROS 2 nodes: {e}')
            sys.exit(1)
        self.get_logger().info('ROS 2 nodes check passed.')

def main(args=None):
    rclpy.init(args=args)
    health_check_node = HealthCheckNode()
    health_check_node.run_check()
    health_check_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
