# Pre-requisite #
* Ubuntu 16.04 LTS
* ROS Kinetic
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

# FAQ #

 ## If after making `posix_sitl`, Gazebo doesn’t launch and gives you a error message regarding `geographiclib`##
 Do the following:
  * In your terminal type:  `sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`
  * When that installation is ready type: 
  * `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`
  * `./install_geographiclib_datasets.sh` (you might have to make the install_geographiclib_datasets.sh executable.)
