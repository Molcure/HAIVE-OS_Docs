# Getting Started

If you have not yet setup Ubuntu 22.04 and ROS2, you can follow the [ros](https://github.com/Molcure/HAIVE-OS/blob/master/docs/ros/) guide.

Clone the repository on an Ubuntu 22.04 system:
```shell
git clone git@github.com:Molcure/HAIVE-OS.git
```

Make sure you have `rosdep` installed:
```shell
sudo apt install python3-rosdep2
rosdep update
```

Install dependencies:
```shell
cd HAIVE-OS/
rosdep install --from-paths src -y --ignore-src
```

Build and run:
```shell
colcon build
. install/setup.bash
ros2 launch hos_run hos_run.launch.py
```

See the [hos_run](https://github.com/Molcure/HAIVE-OS/blob/master/docs/hos_run/) documentation for more details about the HAIVE OS configuration.
