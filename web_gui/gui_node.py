#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import os
import signal

class GuiBridgeNode(Node):
    def __init__(self):
        super().__init__('gui_bridge_node')
        
        self.mapping_sub = self.create_subscription(Empty, '/gui/trigger_mapping', self.mapping_callback, 10)
        self.orch_sub = self.create_subscription(Empty, '/gui/trigger_orchestration', self.orch_callback, 10)
        self.estop_sub = self.create_subscription(Empty, '/gui/estop', self.estop_callback, 10)
        
        self.mapping_process = None
        self.orch_process = None
        
        self.get_logger().info("GUI Bridge Node started. Listening for web commands...")

    def mapping_callback(self, msg):
        if self.mapping_process is None or self.mapping_process.poll() is not None:
            self.get_logger().info("Starting Behavior Tree Mapping Node...")
            # Use preexec_fn to run in a new process group so we can kill all its children easily
            self.mapping_process = subprocess.Popen(
                ["ros2", "run", "robot_Behavior", "robot_behavior"],
                preexec_fn=os.setsid
            )
        else:
            self.get_logger().warn("Mapping is already running!")

    def orch_callback(self, msg):
        if self.orch_process is None or self.orch_process.poll() is not None:
            self.get_logger().info("Starting Map Operations (Orchestration) Node...")
            self.orch_process = subprocess.Popen(
                ["ros2", "run", "robot_navigation", "map_operation_node"],
                preexec_fn=os.setsid
            )
    def estop_callback(self, msg):
        self.get_logger().error("E-STOP TRIGGERED: Force killing behavior and map operations...")
        
        os.system("pkill -f robot_behavior")
        os.system("pkill -f map_operation_node")
        
        self.mapping_process = None
        self.orch_process = None
        
        self.get_logger().info("E-Stop process kill commands sent.")

def main(args=None):
    rclpy.init(args=args)
    node = GuiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GUI Bridge Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
