# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Bring up the physics sim: Gazebo, the pool, the ROV, the ROS bridge and Foxglove.

ros2 launch rov_physics_sim sim.launch.py headless:=true station:=wall_left
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rov_physics_sim import world as pool_world
from rov_physics_sim.vehicle import Vehicle

PKG = "rov_physics_sim"


def _setup(context):
    share = get_package_share_directory(PKG)

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    vehicle_cfg = arg("vehicle_config") or os.path.join(share, "config", "vehicle.yaml")
    pool_cfg = arg("pool_config") or os.path.join(share, "config", "pool.yaml")
    vehicle = Vehicle.from_file(vehicle_cfg)
    pool = pool_world.load(pool_cfg)

    spawn_z = arg("spawn_z")
    pose = pool_world.spawn_pose(pool, float(spawn_z) if spawn_z else None)
    mesh = os.path.join(
        share, vehicle.config.get("visual", {}).get("mesh", "meshes/rov_visual.stl")
    )
    model = vehicle.sdf(
        pose=pose,
        mesh_uri=f"file://{mesh}" if os.path.exists(mesh) else None,
        show_volumes=arg("show_volumes") == "true",
    )
    sdf = pool_world.world_sdf(
        pool, model, station=arg("station"), water_density=vehicle.rho
    )

    world_file = os.path.join(
        tempfile.mkdtemp(prefix="rov_physics_sim_"), "rov_world.sdf"
    )
    with open(world_file, "w") as f:
        f.write(sdf)

    gz_args = ("-r -s " if arg("headless") == "true" else "-r ") + world_file
    return [
        LogInfo(
            msg=f"World written to {world_file} (station {arg('station')}, ROV at {pose[:3]})"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    zenoh = os.environ.get("RMW_IMPLEMENTATION", "") == "rmw_zenoh_cpp"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its GUI",
            ),
            DeclareLaunchArgument(
                "station",
                default_value="no_wall",
                description=f"Launch station layout: {', '.join(pool_world.STATIONS)}",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="",
                description="Spawn height (m); default from pool.yaml",
            ),
            DeclareLaunchArgument(
                "show_volumes",
                default_value="false",
                description="Draw the buoyancy boxes",
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value="",
                description="Override config/vehicle.yaml",
            ),
            DeclareLaunchArgument(
                "pool_config", default_value="", description="Override config/pool.yaml"
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="true",
                description="Start a Foxglove bridge. Set false when the team's infrastructure container runs one",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8766",
                description="Foxglove bridge port. Not 8765, which infrastructure_bringup and rov_gazebo_sim use",
            ),
            DeclareLaunchArgument(
                "zenoh_router",
                default_value="true" if zenoh else "false",
                description="Start a Zenoh router. Set false when the team's infrastructure container runs one",
            ),
            # The router comes first; Zenoh nodes wait for it.
            ExecuteProcess(
                cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("zenoh_router")),
            ),
            OpaqueFunction(function=_setup),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": os.path.join(share, "config", "bridge.yaml"),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                parameters=[
                    {"port": LaunchConfiguration("foxglove_port"), "use_sim_time": True}
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("foxglove")),
            ),
        ]
    )
