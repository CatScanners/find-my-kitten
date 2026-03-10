# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for Oak-d pro."""

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',

        parameters=[{
            'enable_image_denoising': True,
            'rectified_images': False,
            'enable_localization_n_mapping': True,
            'enable_imu_fusion': False,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'gyro_noise_density': 4.9522e-03,
            'gyro_random_walk': 2.3666e-05,
            'accel_noise_density': 2.5703e-02,
            'accel_random_walk': 6.1030e-04,
            'calibration_frequency': 400.0,
            'image_jitter_threshold_ms': 40.00,
            'imu_jitter_threshold_ms': 10.00,
            'localize_on_startup': False,
            'base_frame': 'oak-d_frame',
            'imu_frame': 'oak_imu_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            #'gravitational_force': [-9.8, 0.0, 0.0],
            'gravitational_force': [-8.95, -0.47, -0.45],
            'camera_optical_frames': [
                'oak_left_camera_optical_frame',
                'oak_right_camera_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', '/left/image_rect'),
            ('visual_slam/camera_info_0', '/left/camera_info'),
            ('visual_slam/image_1', '/right/image_rect'),
            ('visual_slam/camera_info_1', '/right/camera_info'),
            ('visual_slam/imu', '/imu'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([visual_slam_launch_container])







