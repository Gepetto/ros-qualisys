ros_qualisys
============

This is a ROS wrapper around the https://github.com/qualisys/qualisys_cpp_sdk package.

## Getting started

### Dependencies

All dependencies are listed in the [package.xml](https://github.com/Gepetto/ros-qualisys/blob/main/package.xml) file.

For building the package:
- git
- doxygen
- cmake

For executing the package:
- catkin + roscpp (if with ROS1)
- ament_cmake + rclcpp (if with ROS2)
- Python 2 or 3 (Standard Ubuntu 18.04 or higher)
- boost (Standard Ubuntu 18.04 or higher)
- qualisys_cpp_sdk (installable via [robotpkg](http://robotpkg.openrobots.org/))
- tf2 (ROS 1 or 2)
- tf2_ros (ROS 1 or 2)
- geometry_msgs (ROS 1 or 2)

### Install from source

You can download this package via github:

    mkdir -p workspace/src
    cd workspace/src
    git clone --recursive git@github.com:Gepetto/ros-qualisys.git

And then build this package via colcon or catkin:

    export CMAKE_PREFIX_PATH=/opt/openrobots
    source /opt/ros/XXXX/setup.bash
    cd workspace
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

This will install the binaries in `workspace/install/ros-qualisys`.
One can use the binaries install by activating the workspace see below.

### Install from binaries

One can install this package through the robotpkg ppa:

    sudo apt install robotpkg-ros-qualisys

### Execute the node

If you are in the LAAS-CNRS buiilding in the Gerard bauzil room, one can just
use the currently available roslaunch in `launch/` after activating the
different workspace:

    source /opt/ros/XXXX/setup.bash
    export LD_LIBRARY_PATH /opt/openrobots/lib:$LD_LIBRARY_PATH
    source workspace/install/setup.bash
    roslaunch ros-qualisys qualisys_bauzil_bringup.launch

The parameter of the node are loaded from the `config/bauzil-qualisys.yaml` file.
The yaml is documented so please check the file [directly here](config/bauzil-qualisys.yaml)

## Online documentation

Online documentation is available at TBD.
