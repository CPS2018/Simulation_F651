# Pre-requisite #
* Ubuntu 16.04 LTS
* ROS Kinetic
* Assuming you use [catkin](http://wiki.ros.org/catkin) build system
* Gazebo 7.14 
  * You can use this script [here](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh):
   1. Download or save it somewhere
   2. Go to the location that you saved it
   3. source ubuntu_sim_ros_gazebo.sh
# OpenUAV #
This simulation is based on [OpenUAV](https://github.com/Open-UAV) project.
1. In your home directory create a `src` folder by typing `mkdir src` in your terminal
2. `cd src`
3. Clone all git repositories in OpenUAV to this folder by `git clone` command
4. Go to the `Firmware` repository by typing `cd Firmware`
5. `source setup-install.sh` (press <kbd>Return</kbd> to continue when asked)
6. When it finishes, type `make posix_sitl_default`
7. Then try to run Gazebo by typing `gazebo` in your terminal
8. Connect the `sitl_gazebo` folder with your `catkin_ws/src` folder by typing `sudo ln -s ~/sitl_gazebo  ~/catkin_ws/src` in your terminal
9. Connect the `src/Firmware` folder with your `catkin_ws/src` folder by typing `sudo ln -s ~/src/Firmware ~/catkin_ws/src` in your terminal
10. Go to your `catkin_ws/src` folder (`cd ~/catkin_ws/src`) and build it by typing `catkin build` in the terminal.
11. `source devel/setup.bash`
12. Now do the folliwing to configure your ROS environment
  1. `roscd`
  2. `cd ..`
  3. `cd src/Firmware`
  4. `make posix_sitl_default`
  5. `source ~/catkin_ws/devel/setup.bash`
  6. `source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default`
  7. `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)`
  8. `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo`
  9. `roslaunch px4 posix_sitl.launch`
  10. `roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14557"`
`


# FAQ #

 ## If after making `posix_sitl`, Gazebo doesn’t launch and gives you a error message regarding `geographiclib` ##
 Do the following:
  * In your terminal type:  `sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`
  * When that installation is ready type: 
  * `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`
  * `./install_geographiclib_datasets.sh` (you might have to make the install_geographiclib_datasets.sh executable.)
