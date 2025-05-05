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
        self.detection_subscriber = self.create_subscription(Detection2DArray, '/detections', self.ball_detection_callback, 10)
        
        # Constants.
        self.BREAK_TIME = 5.0
        self.IMAGE_HEIGHT = 720 # CHANGE MANUALLY DEPENDING ON CAMERA
        self.IMAGE_WIDTH = 1280
        self.RESCUE_MODE = False # True
        self.MOVE_SPEED = 20.0 # Drone max move speed
        self.STEP_SIZE=0.05
        self.TOLERANCE=0.5
        
        # Drone location states.
        self.coords = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0

        # Ball location states.
        self.ball_center_x = None
        self.ball_center_y = None
        
        # Ball rescue mode states.
        self.something_detected = False
        self.goto_rescue = False
        self.rotate_to_zero = False
        self.is_gone_down_already = False
        self.NED = True
        

    def vehicle_local_position_callback(self, msg):
        # Set the current drone states.
        self.coords = np.array([msg.x, msg.y, msg.z])
        self.current_yaw = msg.heading
    
    def ball_detection_callback(self, msg):
        # Process the received message (msg) here
        try:
            # Boilerplate code for handling ball detection.
            for detection in msg.detections:
                for hypothesis in detection.results:
                    if hypothesis.hypothesis.class_id == "32.0" or hypothesis.hypothesis.class_id == "9.0":  # Check for class_id 32 (ball) and 9 (frisbee)
                        self.ball_center_x = detection.bbox.center.position.x
                        self.ball_center_y = detection.bbox.center.position.y
                        if not self.something_detected:
                            self.get_logger().info("Object with class_id 32 detected!")
                            self.something_detected = True
                self.ball_center_x = detection.bbox.center.position.x
                self.ball_center_y = detection.bbox.center.position.y
                if not self.something_detected:
                    self.get_logger().info("Object with class_id 32 detected!")
                    self.something_detected = True

                return
        except Exception:
            self.get_logger().info("Something wetn wrong handling ball detection.")
            return

    def publish_trajectory(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        positions = [x, y, z]
        msg.position = positions
        msg.yaw = yaw
        self.trajectory_pub.publish(msg)
    
    def is_in_middle(self):
        x_dist = self.ball_center_x - self.IMAGE_WIDTH / 2
        y_dist = self.ball_center_y - self.IMAGE_HEIGHT / 2
        dist_from_middle = np.linalg.norm(np.array([ x_dist, y_dist ]))
        return dist_from_middle < 100

    def move_to_waypoint(self, target_coords, yaw, stop_at_middle=False, go_down=True):
        target_coords = np.array(target_coords)
        ramp_factor = 0.70  # Start at 70% speed
        ramp_increment = 0.1  # How fast to ramp up
        max_ramp = 1.0  # Cap at full speed
        rclpy.spin_once(self, timeout_sec=self.BREAK_TIME)

        orig_coords = self.getxyz()
        direction = target_coords - orig_coords
        
        if np.linalg.norm(direction) > 0: # Normalize the direction.
            direction = direction / np.linalg.norm(direction)

        for i in range(3):
            if (abs(direction[i]) <= 0.5):
                direction[i] = 0.0
  
        while np.linalg.norm(target_coords - self.coords) > self.TOLERANCE:

            if stop_at_middle:
                is_in_middle = self.is_in_middle()
                if is_in_middle:
                    self.get_logger().info("We are in the middle! Let's go on top of the cat.")
                    if go_down:
                        x, y, z = self.getxyz()
                        self.is_gone_down_already = True
                        self.get_logger().info("Lets go down.")
                        self.move_to_waypoint([x, y, z + 2.00], yaw=self.current_yaw, stop_at_middle=False)
                        time.sleep(10)
                    return

            if self.something_detected and not self.goto_rescue:
                return
            
            step = direction * self.MOVE_SPEED * self.STEP_SIZE
            
            delta_coords = self.coords + step * ramp_factor
            for i in range(3):
                if abs(direction[i]) <= 0.0001:
                    delta_coords[i] = orig_coords[i]

            self.publish_trajectory(delta_coords[0], delta_coords[1], delta_coords[2], yaw)
            ramp_factor = min(max_ramp, ramp_factor + ramp_increment)
            rclpy.spin_once(self, timeout_sec=self.BREAK_TIME)
    
    def perform_motions(self, motions):
        for waypoint in motions:
            self.move_to_waypoint(waypoint[:3], waypoint[3])
            rclpy.spin_once(self, timeout_sec=self.BREAK_TIME)
            time.sleep(1)
            if self.something_detected:
                break

    # Gazebo coordinate system
    def go_on_top(self):
        rclpy.spin_once(self, timeout_sec=self.BREAK_TIME)

        if self.ball_center_x is None or self.ball_center_y is None:
            self.get_logger().warn("No ball detected, something went wrong!")
            return

        self.get_logger().info(f"yaw = {self.current_yaw}")
        center_image_coords = np.array([self.IMAGE_WIDTH / 2.0, self.IMAGE_HEIGHT / 2.0, 0])
        ball_center_coords = np.array([self.ball_center_x, self.ball_center_y, 0])

        direction_vector = ball_center_coords - center_image_coords
        a = direction_vector[0]
        b = direction_vector[1]
        direction_vector[0] = -b
        direction_vector[1] = a

        # Find unit vector
        direction_vector /= np.linalg.norm(direction_vector)

        # Rotate vector based on yaw angle.
        rotated_x = direction_vector[0] * np.cos(self.current_yaw) - direction_vector[1] * np.sin(self.current_yaw)
        rotated_y = direction_vector[0] * np.sin(self.current_yaw) + direction_vector[1] * np.cos(self.current_yaw)
        rotated_direction_vector = np.array([rotated_x, rotated_y, 0])

        self.get_logger().info(f"direction = [{rotated_direction_vector[0]}, {rotated_direction_vector[1]}]")
        x, y, z = self.getxyz()
        current_coords = np.array([x, y, z])
        target_location = current_coords + rotated_direction_vector
        self.move_to_waypoint(target_location, yaw=self.current_yaw, stop_at_middle=True)

        return

        # NED coordinate system.
    def go_on_top_2(self):
        rclpy.spin_once(self, timeout_sec=self.BREAK_TIME)

        if self.ball_center_x is None or self.ball_center_y is None:
            self.get_logger().warn("No ball detected, something went wrong!")
            return

        self.get_logger().info(f"yaw = {self.current_yaw}")
        center_image_coords = np.array([self.IMAGE_WIDTH / 2.0, self.IMAGE_HEIGHT / 2.0, 0])
        ball_center_coords = np.array([self.ball_center_x, self.ball_center_y, 0])

        direction_vector = ball_center_coords - center_image_coords
        a = direction_vector[0]
        b = direction_vector[1]
        direction_vector[0] = b
        direction_vector[1] = -a

        # Find unit vector
        direction_vector /= np.linalg.norm(direction_vector)

        # Rotate vector based on yaw angle.
        rotated_x = direction_vector[0] * np.cos(self.current_yaw) - direction_vector[1] * np.sin(self.current_yaw)
        rotated_y = direction_vector[0] * np.sin(self.current_yaw) + direction_vector[1] * np.cos(self.current_yaw)
        rotated_direction_vector = np.array([rotated_x, rotated_y, 0])

        self.get_logger().info(f"direction = [{rotated_direction_vector[0]}, {rotated_direction_vector[1]}]")
        x, y, z = self.getxyz()
        current_coords = np.array([x, y, z])
        target_location = current_coords + rotated_direction_vector
        self.move_to_waypoint(target_location, yaw=self.current_yaw, stop_at_middle=True, go_down=True)

        return

    def getxyz(self):
        return self.coords[0], self.coords[1], self.coords[2]

    def start_moving(self):
        time.sleep(2)
        rclpy.spin_once(self, timeout_sec=self.BREAK_TIME)
        x, y, z = self.getxyz()
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
        
        self.perform_motions(waypoints)

        self.get_logger().info("Drone movement complete!")
        if self.something_detected:
            self.goto_rescue = True

            if self.RESCUE_MODE:
                x, y, z = self.getxyz()
            else:
                self.get_logger().info("Ball detected. Lets go on top of it.")
                while True and not self.is_gone_down_already:
                    if self.NED:
                        self.go_on_top_2()
                    else:
                        self.go_on_top()

def main(args=None):
    rclpy.init(args=args)
    node = Maneuver()
    node.start_moving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()