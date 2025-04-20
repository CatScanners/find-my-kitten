#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from vision_msgs.msg import Detection2DArray
import numpy as np
import time
import math


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
        self.coords = np.array([0.0, 0.0, 0.0])
        self.detection_subscriber = self.create_subscription(Detection2DArray, '/detections', self.ball_detection_callback, 10)
        self.current_yaw = 0.0
        self.break_time = 5.0
        self.something_detected = False
        self.goto_rescue = False
        self.rescue_mode = False

        self.ball_center_x = None
        self.ball_center_y = None

        self.image_height = 720 # change manually depending on our camera
        self.image_width = 1280
        
    
    def vehicle_local_position_callback(self, msg):
        self.coords = np.array([msg.x, msg.y, msg.z])
        self.current_yaw = msg.heading
    
    def ball_detection_callback(self, msg):
        # Process the received message (msg) here
        try:
            for detection in msg.detections:
                for hypothesis in detection.results:
                    if hypothesis.hypothesis.class_id == "32.0" and self.something_detected == False:  # Check for class_id 32
                        self.get_logger().info("Object with class_id 32 detected!")
                        self.something_detected = True
                self.ball_center_x = detection.bbox.center.position.x
                self.ball_center_y = detection.bbox.center.position.y
                return
        except Exception:
            self.get_logger().info("Exception")
            return


    def publish_trajectory(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.trajectory_pub.publish(msg)

    def move_to_waypoint(self, target_coords, yaw, speed=4.0, step_size=0.1, tolerance=0.2):
        target_coords = np.array(target_coords)
        ramp_factor = 0.70
        ramp_increment = 0.1
        max_ramp = 1.0

        while np.linalg.norm(self.coords - target_coords) > tolerance + 0.2:
            if self.something_detected and not self.goto_rescue:
                return
            
            direction = target_coords - self.coords
            distance = np.linalg.norm(direction)

            if distance == 0:
                break

            # Clamp small movements to 0 per axis
            step_vector = []
            for i in range(3):  # x, y, z
                axis_diff = direction[i]
                if abs(axis_diff) < tolerance:  # Don't adjust if already close enough on this axis
                    step_vector.append(0.0)
                else:
                    axis_dir = axis_diff / distance
                    step_vector.append(axis_dir * speed * step_size)

            new_coords = self.coords + np.array(step_vector) * ramp_factor
            self.publish_trajectory(new_coords[0], new_coords[1], new_coords[2], yaw)

            ramp_factor = min(max_ramp, ramp_factor + ramp_increment)
            rclpy.spin_once(self, timeout_sec=self.break_time)

            
    def rotate(self, yaw, speed=1.0, step_size=0.1, tolerance=0.2):
        while abs(self.current_yaw - yaw) > tolerance:
            direction = np.sign(yaw - self.current_yaw) 
            step = direction * speed * step_size
            self.current_yaw += step
            
            self.publish_trajectory(self.coords[0], self.coords[1], self.coords[2], self.current_yaw)
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


    def centralize(self):
        img_x = self.image_width / 2.0
        img_y = self.image_height / 2.0
        init_x = self.ball_center_x
        init_y = self.ball_center_y

        x, y, z = self.getxyz()
        z = -5.5
        print("Calibrating")
        self.move_to_waypoint([x + 1, y + 1, z], self.current_yaw)
        rclpy.spin_once(self, timeout_sec=self.break_time)

        time.sleep(2)
        ball_dx = self.ball_center_x - init_x
        ball_dy = self.ball_center_y - init_y
        theta_rad = math.atan2(ball_dy, ball_dx)
        theta_deg = math.degrees(theta_rad)
        x2, y2, z2 = self.getxyz()
        print(f"Image X axis is rotated {theta_deg:.2f}° from VLP.x")
        offset_x = img_x - self.ball_center_x
        offset_y = img_y - self.ball_center_y
        norm_i = math.sqrt(offset_x**2 + offset_y**2)
        ix, iy = [- offset_x / norm_i, - offset_y / norm_i]
        print(ix, iy, " ix and iy before transformation")
        vlp_dx = ix * math.cos(theta_rad) - iy * math.sin(theta_rad)
        vlp_dy = ix * math.sin(theta_rad) + iy * math.cos(theta_rad)
        z2 = -5.5
        offset_vec_vlp = [vlp_dx, vlp_dy]
        print(offset_vec_vlp, " drone movement")
        target_x = x2 + vlp_dx * 10
        target_y = y2 + vlp_dy * 10
        # Command the drone to move to this target position
        self.move_to_waypoint([target_x, target_y, z2], self.current_yaw, speed=5.0)


        print("Centralized")
        return

    def getxyz(self):
        return self.coords[0], self.coords[1], self.coords[2]

    def start_moving(self):
        time.sleep(2)
        rclpy.spin_once(self, timeout_sec=self.break_time)
        x, y, z = self.getxyz() # hardcode z
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
                self.move_to_waypoint([self.coords[0], self.coords[1], -5.0], self.current_yaw, s1)
            else:
                print("Follower mode")
                self.goto_rescue = True
                self.centralize()


def main(args=None):
    rclpy.init(args=args)
    node = Maneuver()
    node.start_moving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()