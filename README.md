ros_qualisys
============

This is ROS wrapper around the https://github.com/qualisys/qualisys_cpp_sdk package.

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

    git clone --recursive git@github.com:Gepetto/ros-qualisys.git

And then build the package via CMake and Make:

    export CMAKE_PREFIX_PATH=/opt/openrobots
    source /opt/ros/XXXX/setup.bash
    cd ros-qualisys
    mkdir _build
    cd _build
    cmake .. -DCMAKE_INSTALL_PREFIX=install
    make
    make install

This will install the binaries in `ros-qualisys/install`, one should change the
install path.

### Install from binaries

One can install this package through the robotpkg ppa:

    sudo apt install robotpkg-ros-qualisys

### Execute the node

If you are in the LAAS-CNRS buiilding in the Gerard bauzil room, one can just
use the currently available roslaunch (available in `launch/`):

    roslaunch ros-qualisys qualisys_bauzil_bringup.launch

The parameter of the node are loaded from the `config/bauzil-qualisys.yaml` file.
The yaml is documented so please check the file [directly here](https://github.com/Gepetto/ros-qualisys/blob/main/config/bauzil-qualisys.yaml)

## Online documentation

Online documentation is available at TBD.
