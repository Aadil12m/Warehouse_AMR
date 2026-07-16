import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class battery_dummy(Node):
    def __init__(self):
        super().__init__('battery_dummy')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(BatteryState, '/battery_level', qos_profile)
        
        # Wait a few seconds for AMCL to fully wake up before firing
        self.timer = self.create_timer(3.0, self.send_pose)

    def send_pose(self):
        msg = BatteryState()
        msg.percentage=1.0
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = battery_dummy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()