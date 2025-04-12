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
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/custom_trajectory', 10)
        self.current_coords = np.array([0.0, 0.0, 0.0])
        self.detection_subscriber = self.create_subscription(Detection2DArray, '/detections', self.ball_detection_callback, 10)
        self.current_yaw = 0.0
        self.break_time = 5.0
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

    def move_to_waypoint(self, target_coords, yaw, speed=1.0, step_size=0.1, tolerance=0.2):
        target_coords = np.array(target_coords)
        
        while np.linalg.norm(self.current_coords - target_coords) > tolerance:
            direction = target_coords - self.current_coords
            distance = np.linalg.norm(direction)
            if self.something_detected and self.goto_rescue == False:
                return 
            if distance > 0:
                step = direction / distance * speed * step_size 
                self.current_coords += step 

            self.publish_trajectory(self.current_coords[0], self.current_coords[1], self.current_coords[2], yaw)
            #self.get_logger().info(str([self.current_coords[0], self.current_coords[1], self.current_coords[2]]))
            
            rclpy.spin_once(self, timeout_sec=0.05)
            
    def rotate(self, yaw, speed=1.0, step_size=0.1, tolerance=0.2):
        while abs(self.current_yaw - yaw) > tolerance:
            direction = np.sign(yaw - self.current_yaw) 
            step = direction * speed * step_size
            self.current_yaw += step
            
            self.publish_trajectory(self.current_coords[0], self.current_coords[1], self.current_coords[2], self.current_yaw)
            self.get_logger().info(f"Yaw: {self.current_yaw:.2f}")
            time.sleep(step_size)  
            
        rclpy.spin_once(self, timeout_sec=self.break_time)
        time.sleep(3)
    
    def perform_motions(self, motions, speed=2.5):
        for waypoint in motions:
            self.move_to_waypoint(waypoint[:3], waypoint[3], speed=speed)
            rclpy.spin_once(self, timeout_sec=self.break_time)
            time.sleep(1)
            if self.something_detected:
                break
        
        # self.rotate(6.28, speed=speed)
        # self.rotate(0.0, speed=speed)
    
    def start_moving(self):
        time.sleep(5)
        rclpy.spin_once(self, timeout_sec=self.break_time)
        x, y, z = self.current_coords[0], self.current_coords[1], self.current_coords[2] # hardcode z
        yaw = self.current_yaw

        waypoints = [
            (x, y, z, yaw),
            (x + 8, y, z, yaw),
            (x + 8, y + 4, z, yaw),
            (x, y + 4, z, yaw),
            (x, y + 8, z, yaw),
            (x + 8, y + 8, z, yaw),
            (x + 8, y + 12, z, yaw)
        ]

        s1 = 4.0
        self.perform_motions(waypoints, s1)

        self.get_logger().info("Drone movement complete!")
        if self.something_detected:
            if self.rescue_mode:
                self.goto_rescue = True
                self.move_to_waypoint([self.current_coords[0], self.current_coords[1], -5.0], self.current_yaw, s1)
            else:
                print("Follower mode")


def main(args=None):
    rclpy.init(args=args)
    node = Maneuver()
    node.start_moving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()