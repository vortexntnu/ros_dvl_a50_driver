# ros_dvl_a50_driver
[![Build status](https://github.com/vortexntnu/ros_dvl_a50_driver/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/ros_dvl_a50_driver/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/ros_dvl_a50_driver/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/ros_dvl_a50_driver/main)

ROS2 driver for the WaterLinked DVL-A50. Currently work-in-progress.

# Get started
Build with `colcon build --symlink-install --packages-select ros_dvl_a50_driver` and run with `ros2 launch ros_dvl_a50_driver ros_dvl_a50_driver.launch.py` :)
Currently, only ip and port are settable parameters, but they default to `192.168.194.95` and `16171` respectively, which should work out of the box with the A50.
