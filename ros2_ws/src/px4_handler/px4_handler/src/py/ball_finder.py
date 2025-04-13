#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from vision_msgs.msg import Detection2DArray
import numpy as np
import time


class Maneuver(Node):
    def __init__(self):
        super().__init__('drone_movement_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Ros pubsubs
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/custom_trajectory', 10)
        self.detection_subscriber = self.create_subscription(Detection2DArray, '/detections', self.ball_detection_callback, 10)
        # Global state for current coords
        self.current_coords = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0

        # Some helper class variables
        self.SPIN_TIMEOUT = 0.05
        self.SPEED = 4.0
        self.CORNER_BREAK = 1.0
        self.something_detected = False
        self.goto_rescue = False
        self.rescue_mode = True

        # For follower mode
        # self.direction_pub = self.create_publisher(String, '/direction_commands', 10)  # New topic for direction commands
        # self.direction_subscriber = self.create_subscription(String, '/direction_commands', self.direction_callback, 10)  # Subscription to direction commands

    
    def vehicle_local_position_callback(self, msg):
        self.current_coords = np.array([msg.x, msg.y, msg.z])
        self.current_yaw = msg.heading
    
    def ball_detection_callback(self, msg):
        # Process the received message (msg) here
        try:
            for detection in msg.detections:
                for hypothesis in detection.results:
                    if hypothesis.hypothesis.class_id == "32.0" and self.something_detected == False:  # Check for class_id 32
                        self.get_logger().info("Object with class_id 32 detected!")
                        self.something_detected = True
                        return
        except Exception:
            return


    def publish_trajectory(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.trajectory_pub.publish(msg)

    
    
    def move_in_direction_infinitely(self, direction_vector, step_size=0.1):
        """
        Continuously move the drone in a given direction vector.
        
        Args:
            direction_vector (list or np.array): The [dx, dy, dz] direction.
            step_size (float): Step size for movement updates.
        """
        direction_vector = np.array(direction_vector)
        norm = np.linalg.norm(direction_vector)

        if norm == 0:
            self.get_logger().warn("Direction vector cannot be zero.")
            return
        
        unit_direction = direction_vector / norm

        self.get_logger().info(f"Starting infinite movement in direction {unit_direction}...")

        while rclpy.ok():
            self.current_coords += unit_direction * self.SPEED * step_size
            self.publish_trajectory(self.current_coords[0], self.current_coords[1], self.current_coords[2], self.current_yaw)

            rclpy.spin_once(self, timeout_sec=self.SPIN_TIMEOUT)
            time.sleep(step_size)

    def move_to_waypoint(self, target_coords, yaw, step_size=0.1, tolerance=0.2):
        """
        Takes target coordinates [x, y, z, yaw] and moves to that dir.

        Args:
            target_coords (list): Target coordinates [x, y, z]
            yaw (float): Target rotation
        """
        target_coords = np.array(target_coords) # To numpy array
        
        
        while np.linalg.norm(self.current_coords - target_coords) > tolerance: # Have we reached the goal?
            direction = target_coords - self.current_coords # Direction vector
            distance = np.linalg.norm(direction) # Distance to the target
            if self.something_detected and self.goto_rescue == False: # If we have detected something --> return
                return 
            if distance > 0: # If distance is bigger than zero
                step = direction / distance * self.SPEED * step_size 
                self.current_coords += step 

            self.publish_trajectory(self.current_coords[0], self.current_coords[1], self.current_coords[2], yaw)
            
            rclpy.spin_once(self, timeout_sec=0.0)
    
    def perform_motions(self, motions):
        for waypoint in motions:
            self.move_to_waypoint(waypoint[:3], waypoint[3])
            time.sleep(self.CORNER_BREAK)

            if self.something_detected: # We have detected something, break movements immediatelly.
                break
    
    def start_moving(self, waypoints):
        """
        Takes waypoints and moves to their setpoints one at a time.
        
        Args:
            waypoints (list): List of target setpoints [x, y, z, yaw].
        """
        time.sleep(3) # Don't start immediatelly
        rclpy.spin_once(self, timeout_sec=self.SPIN_TIMEOUT) # Check for callbacks
        x, y, z = self.current_coords[0], self.current_coords[1], self.current_coords[2] # Starting x, y, z
        yaw = self.current_yaw # Starting yaw

        self.perform_motions(waypoints)

        if self.something_detected:
            self.get_logger().info("Cat found, let's go to its rescue.")
        else:
            self.get_logger().info("Cat not found.")

        if self.something_detected:
            if self.rescue_mode:
                self.goto_rescue = True
                self.move_to_waypoint([self.current_coords[0], self.current_coords[1], -5.0], self.current_yaw, s1)
            else:
                print("Follower mode")



def main(args=None):

    waypoints = [
        (x, y, z, yaw),
        (x + 8, y, z, yaw),
        (x + 8, y + 4, z, yaw),
        (x, y + 4, z, yaw),
        (x, y + 8, z, yaw),
        (x + 8, y + 8, z, yaw),
        (x + 8, y + 12, z, yaw)
    ]

    rclpy.init(args=args)
    node = Maneuver()
    node.start_moving(waypoints=waypoints)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()