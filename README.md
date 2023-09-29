# Racing-LMPC-ROS2

[![ROS2 Humble](https://github.com/HaoruXue/Racing-LMPC-ROS2/actions/workflows/ros2-humble-ci.yaml/badge.svg)](https://github.com/HaoruXue/Racing-LMPC-ROS2/actions/workflows/ros2-humble-ci.yaml)

C++ ROS2 packages that implement learning model predictive control for real-world autonomous race cars.

Paper: [Learning Model Predictive Control with Error Dynamics Regression for Autonomous Racing](https://arxiv.org/abs/2309.10716)

![lmpc-foxglove](vis.gif)

## Install

### Dependencies

- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- CasADi >= 3.6.3. Must be built from source, since the pre-compiled binary is C++ 11 and therefore not ABI compatible with C++ 17.
  - See [here](https://github.com/casadi/casadi/wiki/InstallationLinux) for official build instructions.
  - Refer to the "Build Casadi" section in CI file [here](.github/workflows/ros2-humble-ci.yaml) for a working build instruction. Note that we installed `llvm` for JIT compilation, and ipopt for nonlinear optimization. Also note that we appended `/usr/local/lib` to `LD_LIBRARY_PATH` in order to find the shared library `libcasadi.so` at runtime.
- Foxglove Studio (optional). Rviz2 is fine too. See [here](https://foxglove.dev/) for installation instructions.

### Build

```bash
# Clone the repo
cd Racing-LMPC-ROS2
git checkout humble-release
# Install rosdep dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# Build
colcon build --packages-up-to racing_lmpc_launch --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run LMPC

Open Foxglove Studio. Load `lmpc.foxglove.json` layout file from the root of this repo. Open a Foxglove Bridge connection with the default port and IP setting.

In terminal 1, run the following command to launch the simulator:

```bash
source install/setup.bash
ros2 launch racing_lmpc_launch sim_barc_tracking_mpc.launch.py
```

In terminal 2, run the following command to launch the foxglove bridge:

```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

You can now use Foxglove Studio to visualize the simulation.

### Run Tracking MPC

Same as above, except run the following command in terminal 1:

```bash
source install/setup.bash
ros2 launch racing_lmpc_launch sim_barc_tracking_mpc.launch.py
```

For IAC Putnam full course, run the following command in terminal 1:

```bash
source install/setup.bash
ros2 launch racing_lmpc_launch sim_putnam_config_a_tracking_mpc.launch.py
```

Remember to change the line scales of the 3D pannel accordingly to view them easily. Change display frame to `base_link` because the track is huge.

## Cite As

```bibtex
@misc{xue2023lmpc,
      title={Learning Model Predictive Control with Error Dynamics Regression for Autonomous Racing}, 
      author={Haoru Xue and Edward L. Zhu and Francesco Borrelli},
      year={2023},
      eprint={2309.10716},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
