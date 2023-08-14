# Copyright 2023 Haoru Xue
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def get_share_file(package_name, *args):
    return os.path.join(get_package_share_directory(package_name), *args)


def get_sim_time_launch_arg():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )

    return declare_use_sim_time_cmd, {"use_sim_time": use_sim_time}


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    mpc_config = get_share_file(
        "racing_mpc", "param", "sample_mpc.param.yaml")
    dt_model_config = (
        get_share_file("single_track_planar_model"),
        "/param/",
        "sample_vehicle.param.yaml",
    )
    base_model_config = (
        get_share_file("base_vehicle_model"),
        "/param/",
        "sample_vehicle.param.yaml",
    )
    track_file = get_share_file(
        "racing_trajectory", "test_data", "putnam")

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="racing_mpc",
                executable="racing_mpc_node_exe",
                name="racing_mpc_node",
                output="screen",
                parameters=[
                    mpc_config,
                    dt_model_config,
                    base_model_config,
                    use_sim_time,
                    {
                        "racing_mpc_node.dt": 0.02,
                        "racing_mpc_node.vehicle_model_name": "single_track_planar_model",
                        "racing_mpc_node.default_traj_idx": 27,
                        "racing_mpc_node.traj_folder": track_file,
                        "racing_mpc_node.velocity_profile_scale": 1.0,
                    },
                ],
                remappings=[
                ],
                prefix=['taskset -c 22,23'],
                emulate_tty=True,
            ),
        ]
    )
