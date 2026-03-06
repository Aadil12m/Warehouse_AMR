import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import transforms3d

class AmclInitPosePublisher(Node):
    def __init__(self):
        super().__init__('amcl_init_pose_publisher')
        
        # Declare parameters (force them to be floats to avoid silent crashes)
        self.declare_parameter('x', -2.0)
        self.declare_parameter('y', -0.5)
        self.declare_parameter('theta', 0.0)
        self.declare_parameter('cov', 0.25)
        

        # Match RViz's QoS profile exactly
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile)
        
        # Wait a few seconds for AMCL to fully wake up before firing
        self.timer = self.create_timer(3.0, self.send_pose)

    def send_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        
        # Grab the exact simulation time
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # CRITICAL: Force typecast to float. If an int slips in here, the node will silently fail.
        msg.pose.pose.position.x = float(self.get_parameter('x').value)
        msg.pose.pose.position.y = float(self.get_parameter('y').value)
        
        theta = float(self.get_parameter('theta').value)
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        msg.pose.pose.orientation.w = float(quat[0])
        msg.pose.pose.orientation.x = float(quat[1])
        msg.pose.pose.orientation.y = float(quat[2])
        msg.pose.pose.orientation.z = float(quat[3])
        
        cov = float(self.get_parameter('cov').value)
        covariance = np.zeros(36)
        covariance[0] = cov
        covariance[7] = cov
        covariance[35] = cov
        msg.pose.covariance = covariance.tolist()
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Published initial pose: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")
        
        # Cancel the timer so it only successfully publishes once
        self.timer.cancel()

def main():
    rclpy.init()
    node = AmclInitPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()