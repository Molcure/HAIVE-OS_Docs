# Getting Started

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
ros2 launch hos_sim_haive4 hos_sim_haive4.launch.py
```
