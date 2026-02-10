#!/usr/bin/env python

import os
import carb
import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    })

import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from scipy.spatial.transform import Rotation

# Import the Pegasus API
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

class PegasusApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch the world
        self.pg.load_environment(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenes", "simple_obstacles.usd")))

        # Multirotor configuration
        config_multirotor = MultirotorConfig()
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
            "udp_port": 14550,
        })

        # Config the backends
        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ROS2Backend(
                vehicle_id=1,
                config={
                    "namespace": "drone",
                    "pub_sensors": True,
                    "pub_graphical_sensors": True,
                    "pub_state": False,
                    "sub_control": False,
                },
            ),
        ]

        # Define and add a down-facing camera
        down_facing_camera = MonocularCamera(
            camera_name="down_facing_camera",
            config={
                "update_rate": 30.0,
                "intrinsics": np.array(
                    [[609.25, 0.0, 960.0], [0.0, 609.25, 540.0], [0.0, 0.0, 1.0]]
                ),
                "diagonal_fov": 160.0,
                "resolution": (1920, 1080),
                "position": np.array([0.0, 0.0, -0.04]),
                "orientation": np.array([180, -90.0, 0.0]),
            }
        )

        # ZED Stereo Camera - Left Camera
        stereo_left_camera = MonocularCamera(
            camera_name="zed_left_camera",
            config={
                "update_rate": 30.0,
                "intrinsics": np.array(
                    [[1049.7, 0.0, 960.0], [0.0, 1049.7, 540.0], [0.0, 0.0, 1.0]]
                ),
                "diagonal_fov": 100.0, 
                "resolution": (1920, 1080),
                "position": np.array([0.10, -0.06, 0.0]),  # Front of drone, left side
                "orientation": np.array([0.0, 0.0, 180.0]),
            }
        )

        # ZED Stereo Camera - Right Camera
        stereo_right_camera = MonocularCamera(
            camera_name="zed_right_camera",
            config={
                "update_rate": 30.0,
                "intrinsics": np.array(
                    [[1049.7, 0.0, 960.0], [0.0, 1049.7, 540.0], [0.0, 0.0, 1.0]]
                ),
                "diagonal_fov": 100.0,
                "resolution": (1920, 1080),
                "position": np.array([0.10, 0.06, 0.0]),  # Front of drone, right side
                "orientation": np.array([0.0, 0.0, 180.0]),  # Front-facing, rotated to the right
            }
        )

        config_multirotor.graphical_sensors = [
            down_facing_camera,
            stereo_left_camera,
            stereo_right_camera
            ]


        # Create and add a multirotor vehicle to the world
        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,

        )

        # Reset the simulation environment
        self.world.reset()

    def run(self):
        # Start the simulation
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()


if __name__ == "__main__":
    main()