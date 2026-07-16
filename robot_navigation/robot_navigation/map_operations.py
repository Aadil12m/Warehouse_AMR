import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray,Pose
import cv2
import numpy as np
import yaml

class map_operation(Node):
    def __init__(self):
        super().__init__("map_operation_Node")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_ = self.create_publisher(PoseArray, 'rack_poses', 10)
        self.timer_ = self.create_timer(1.0, self.pub_callback)
        self.has_published = False

    def pub_callback(self):
        map_path = '/home/aadil/AMR_ws/src/Warehouse_AMR/robot_gazebo/maps/my_new_map'
        
        # Load the YAML file to get dynamic map metadata
        try:
            with open(f'{map_path}.yaml', 'r') as file:
                map_data = yaml.safe_load(file)
                resolution = map_data['resolution']
                origin_x = map_data['origin'][0]
                origin_y = map_data['origin'][1]
        except Exception as e:
            # We don't error out because the map might not be saved yet!
            self.get_logger().info("Waiting for Explore node to save the map...")
            return
        
        map_img = cv2.imread(f'{map_path}.pgm', cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            self.get_logger().error("YAML found but could not load .pgm image!")
            return
        
        coords = []
        _, thresh = cv2.threshold(map_img, 240, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            
            if area < 20 or area > 500:
                continue
                
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Dynamically calculate the real-world coordinates!
                map_height = map_img.shape[0]
                world_x = (cX * resolution) + origin_x
                world_y = ((map_height - cY) * resolution) + origin_y
                
                # Filter: only accept detections inside the rack grid area
                # The 9 racks are in the box X:[-1.3, 1.3] Y:[-1.3, 1.3]
                # This excludes the hexagon decorations outside that area
                if world_x < -1.3 or world_x > 1.3 or world_y < -1.3 or world_y > 1.3:
                    self.get_logger().warn(f"REJECTED: area={area:.1f} world=({world_x:.2f}, {world_y:.2f}) - outside rack grid")
                    continue
                
                # Offset: rack radius=0.15m, robot half-width=0.15m (0.5x0.3 body)
                # 0.55m offset gives clearance from rack edge
                waypoint_x = world_x
                waypoint_y = world_y - 0.55
                
                coords.append([waypoint_x, waypoint_y])
                
                # Print the coordinates to the terminal so the user can see them!
                self.get_logger().info(f"Rack Pixel ({cX},{cY}) -> World ({world_x:.2f}, {world_y:.2f}). Waypoint: ({waypoint_x:.2f}, {waypoint_y:.2f})")

        Poses = PoseArray()
        Poses.header.frame_id = "map"
        
        for i in range(len(coords)):
            rack_waypoint = Pose()
            rack_waypoint.position.x = float(coords[i][0])
            rack_waypoint.position.y = float(coords[i][1])
            
            # Orientation: yaw = 90 degrees (pi/2) to face +Y (the rack)
            # This makes the front of the robot (camera) face the rack
            rack_waypoint.orientation.z = 0.7071068
            rack_waypoint.orientation.w = 0.7071068
            
            Poses.poses.append(rack_waypoint)

        if not self.has_published:
            self.pub_.publish(Poses)
            self.get_logger().info(f"Published {len(coords)} rack poses. Shutting down detection timer.")
            self.has_published = True
            
            # Optional: destroy the timer so we don't keep running OpenCV
            self.timer_.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = map_operation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
