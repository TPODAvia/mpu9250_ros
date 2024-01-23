# mpu9250_ros

This repository contains the Python ROS package for interfacing with the MPU9250 Inertial Measurement Unit (IMU). It is based on the `imusensor` library, which has been modified for better integration with ROS. The package has been tested with ROS Melodic and Noetic.

## Prerequisites

Before you can use this package, make sure you have the following installed:
- [ROS Melodic](http://wiki.ros.org/melodic/Installation) or [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Python 2.7 (for Melodic) or Python 3 (for Noetic)
- pip (Python package installer)

## Installation

To install the `mpu9250_ros` package, follow these steps:

1. Navigate to your ROS workspace's `src` directory. If you don't have a workspace, you can create one with:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

2. Clone the `mpu9250_ros` repository into the `src` directory:

```bash
cd ~/catkin_ws/src
git clone https://github.com/TPODAvia/mpu9250_ros
```

3. Install any dependencies using `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Build your workspace with the new package:

```bash
cd ~/catkin_ws
catkin_make
```

5. Source the ROS environment for your shell session:

```bash
source devel/setup.bash
```

## Usage

To use the `mpu9250_ros` package, you need to launch the provided ROS node. You can do this with:

```bash
roslaunch mpu9250_ros mpu9250.launch
```

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

- Original `imusensor` library by [niru-5](https://github.com/niru-5)
- ROS community for guidance and support

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Support

For questions and support, please open an issue in the GitHub issue tracker.

Happy Coding!