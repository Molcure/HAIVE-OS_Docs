# Developing with ROS 2

This is a summary of practices for developing ROS 2 applications using Python.

## Create a workspace

First, create a directory (`ros2_ws`) to contain our workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Creating a Python package

Move to your workspace `src` folder:

```bash
cd ~/ros2_ws/src
```

Create an empty package:

```bash
ros2 pkg create --build-type ament_python <package_name>
```

Create a package with a node:

```bash
ros2 pkg create --build-type ament_python --node-name <node_name> <package_name>
```

## Build a workspace

In the root of the workspace, run `colcon build`. Since build types such as `ament_cmake` do not support the concept of the `devel` space and require the package to be installed, colcon supports the option `--symlink-install`. This allows the installed files to be changed by changing the files in the `source` space (e.g. Python files or other not compiled resourced) for faster iteration.

```bash
colcon build --symlink-install
```

This currently creates several warnings as described here: [https://github.com/colcon/colcon-core/issues/454](https://github.com/colcon/colcon-core/issues/454). To quiet the deprecation warning set the following environment variable:

```bash
echo -e "\n# Quiet python warnings when building with colcon\nexport PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources" >> ~/.bashrc
```

## Run tests

To run tests for the packages we just built, run the following:

```bash
colcon test
```

## Source the environment

When colcon has completed building successfully, the output will be in the `install` directory. Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. colcon will have generated bash/bat files in the `install` directory to help setup the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.

```bash
. install/setup.bash
```

## Run the node

```bash
ros2 run <package_name> <node_name>
```

## Setup `colcon_cd`

The command `colcon_cd` allows you to quickly change the current working directory of your shell to the directory of a package. As an example `colcon_cd some_ros_package` would quickly bring you to the directory `~/ros2_install/src/some_ros_package`.

```bash
echo -e "\n# Setup colcon_cd\nsource /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```

Depending to the way you installed `colcon_cd` and where your workspace is, the instructions above may vary, please refer to [the documentation](https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes) for more details. To undo this in Linux and macOS, locate your system’s shell startup script and remove the appended source and export commands.

## Setup `colcon` tab completion

The command `colcon` [supports command completion](https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion) for bash and bash-like shells if the `colcon-argcomplete` package is installed.

```bash
echo -e "\n# Setup colcon tab completeion\nsource /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

Depending to the way you installed `colcon` and where your workspace is, the instructions above may vary, please refer to [the documentation](https://colcon.readthedocs.io/en/released/user/installation.html) for more details. To undo this in Linux and macOS, locate your system’s shell startup script and remove the appended source command.

<!---

WORK IN PROGRESS

## Creating a launch file

TODO: Check commit for launch file

Create the launch file:

```bash
mkdir launch
touch launch/my_ros_system.launch.py
```

[https://github.com/Molcure/haive_os](https://github.com/Molcure/haive_os)
[https://github.com/Molcure/HAIVE4_ROS/tree/ros2](https://github.com/Molcure/HAIVE4_ROS/tree/ros2)
[https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
[https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)
[https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
[https://github.com/Molcure/HAIVE4_ROS/tree/ros2/haive_ws/src/haive_system](https://github.com/Molcure/HAIVE4_ROS/tree/ros2/haive_ws/src/haive_system)

Inside our `setup.py` file:

**`import** **osfrom** **glob** **import** glob
**from** **setuptools** **import** setup

package_name = 'py_launch_example'

setup(
    *# Other parameters ...*data_files=[
        *# ... Other data files# Include all launch files.*(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)`

[https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
[https://github.com/gramaziokohler/ros_docker/blob/master/ros-kinetic-moveit/Dockerfile](https://github.com/gramaziokohler/ros_docker/blob/master/ros-kinetic-moveit/Dockerfile)
[https://github.com/Molcure/HAIVE4_ROS/blob/master/Dockerfile](https://github.com/Molcure/HAIVE4_ROS/blob/master/Dockerfile)
[https://docsify.js.org/#/quickstart](https://docsify.js.org/#/quickstart)

CURRENT

[https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)
[https://github.com/gramaziokohler/ros_docker/blob/master/ros-kinetic-moveit/Dockerfile](https://github.com/gramaziokohler/ros_docker/blob/master/ros-kinetic-moveit/Dockerfile)
[https://github.com/Molcure/HAIVE4_ROS/blob/master/Dockerfile](https://github.com/Molcure/HAIVE4_ROS/blob/master/Dockerfile)
[https://github.com/Molcure/HAIVE4_ROS/blob/ros2/haive_ws/src/haive_system/setup.py](https://github.com/Molcure/HAIVE4_ROS/blob/ros2/haive_ws/src/haive_system/setup.py)
[https://hub.docker.com/_/ros/](https://hub.docker.com/_/ros/)

-->
