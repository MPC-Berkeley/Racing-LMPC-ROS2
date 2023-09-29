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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from lmpc_utils.lmpc_launch_utils import get_share_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    sim_config = get_share_file(
        "racing_lmpc_launch", "param", "racing_simulator", "continuous_simulator.param.yaml")
    dt_model_config = get_share_file(
        "racing_lmpc_launch", "param", "barc", "barc_single_track.param.yaml")
    base_model_config = get_share_file(
        "racing_lmpc_launch", "param", "barc", "barc_base.param.yaml")
    mpc_config = get_share_file(
        "racing_lmpc_launch", "param", "racing_mpc", "barc_lmpc.param.yaml")
    sim_track_file = get_share_file(
        "racing_trajectory", "test_data", "barc", "02_barc_center.txt")
    track_file_folder = get_share_file(
        "racing_trajectory", "test_data", "barc")

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="racing_simulator",
                executable="racing_simulator_node_exe",
                name="continuous_racing_simulator_node",
                output="screen",
                parameters=[
                    sim_config,
                    dt_model_config,
                    base_model_config,
                    use_sim_time,
                    {
                        "racing_simulator.race_track_file_path": sim_track_file,
                        "modeling.use_frenet": False,
                        "racing_simulator.x0": [1.0, 0.0, 0.0, 1.5, 0.0, 0.0]
                        # "racing_simulator.x0": [-2.0, 3.5, -1.5708, 1.5, 0.0, 0.0]
                    },
                ],
                remappings=[
                    ("abscissa_polygon", "/simulation/abscissa_polygon"),
                    ("left_boundary_polygon", "/simulation/left_boundary_polygon"),
                    ("right_boundary_polygon", "/simulation/right_boundary_polygon"),
                ],
                emulate_tty=True,
            ),
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
                        "racing_mpc_node.dt": 0.025,
                        "racing_mpc_node.vehicle_model_name": "single_track_planar_model",
                        "racing_mpc_node.default_traj_idx": 2,
                        "racing_mpc_node.traj_folder": track_file_folder,
                        "racing_mpc_node.velocity_profile_scale": 0.9,
                        "racing_mpc_node.delay_step": 0,
                    },
                ],
                remappings=[
                ],
                # prefix=['taskset -c 22,23'],
                emulate_tty=True,
            ),
        ]
    )
