# Pre-requisite #
* Ubuntu 16.04 LTS
* ROS Kinetic
* Assuming you use [catkin](http://wiki.ros.org/catkin) build system
* Gazebo 7.14 
  * OR You can use download and run this script [here](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh):
   1. Download and save it
   2. Go to the location that you saved it
   3. Run `source ubuntu_sim_ros_gazebo.sh`
   ![gazebo_quadtest](https://user-images.githubusercontent.com/2436747/48624033-bdee7180-e9ab-11e8-8414-801eee34f4e9.png)
   
# OpenUAV #
This simulation is based on [OpenUAV](https://github.com/Open-UAV) project.
1. In your home directory create a `src` folder by typing `mkdir src` in your terminal
2. Go to the src folder `cd src`
3. Clone all git repositories in OpenUAV to this folder by `git clone` command
4. Go to the `Firmware` repository by typing `cd Firmware`
5. `source setup-install.sh` (press <kbd>Return</kbd> to continue when asked)
6. When it finishes, type `make posix_sitl_default`
7. Then run Gazebo by typing `gazebo` in your terminal
8. Connect the `sitl_gazebo` folder with your `catkin_ws/src` folder by typing `sudo ln -s ~/sitl_gazebo  ~/catkin_ws/src` in your terminal
9. Connect the `src/Firmware` folder with your `catkin_ws/src` folder by typing `sudo ln -s ~/src/Firmware ~/catkin_ws/src` in your terminal
10. Go to your `catkin_ws/src` folder (`cd ~/catkin_ws/src`) and build it by typing `catkin build` in the terminal.
11. Type `source devel/setup.bash`
12. Now do the following to configure your ROS environment:
    1. `roscd`
    2. `cd ..`
    3. `cd src/Firmware`
    4. `make posix_sitl_default`
    5. `source ~/catkin_ws/devel/setup.bash`
    6. `source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default`
    7. `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)`
    8. `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo`
13. Open a terminal and type: `roslaunch px4 posix_sitl.launch`
14. Open a terminal and type: `roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14557"`

# Gripper Setup #
1. Open a terminal and type `printenv | grep GAZEBO`
2. Two or more paths should be displayed:
   GAZEBO_MODEL_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/models:/home/user/catkin_ws/src/simulation_control/src/models
   GAZEBO_PLUGIN_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/Build (Gazebo gets it's models in this case from two paths which are told apart by the `:` sign.)
3. Add the file `gripper_plugin.cc` in the folder one step up from your GAZEBO_PLUGIN_PATH. In this case, it would be "/home/user/src/Firmware/Tools/sitl_gazebo/src"
4. Add the following under plugins to the `CMakeLists.txt` file located at "/home/user/src/Firmware/Tools/sitl_gazebo/src"
	` -----GripperPlugin`
	`find_package(roscpp REQUIRED)`
	`find_package(std_msgs REQUIRED)`
	`include_directories(${roscpp_INCLUDE_DIRS})`
	`include_directories(${std_msgs_INCLUDE_DIRS})	`
	`find_package(gazebo REQUIRED)`
	`include_directories(${GAZEBO_INCLUDE_DIRS})`
	`link_directories(${GAZEBO_LIBRARY_DIRS})`
	`set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")`
        `add_library(gripper_plugin SHARED src/gripper_plugin.cc)`
	`target_link_libraries(gripper_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})`
5. Open a terminal and type the following:
	1. `cd ~/home/user/src/Firmware/Tools/sitl_gazebo/Build  (GAZEBO_PLUGIN_PATH)`
	2. `cmake ../`
	3. `make`
6. Add the folder `simulation_control` to your `catkin_ws/src` folder.
7. Open a terminal and type the following to build the catkin workspace:
	1. `cd ~/catkin_ws`
	2. `catkin build`
8. Double check if you're missing any dependencies like `pysci` etc. and install them if not. Then build again.
9. Open a terminal and type: (adding gazebo model path to bashrc)
	`sudo nano ~/.bashrc`
10. In your bashrc, add the following line along with your GAZEBO_MODEL_PATH:
	`:/home/user/catkin_ws/src/simulation_control/src/models`
   For example if it looks like this:
	`export GAZEBO_MODEL_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/models`
   After editing it should look like this:
	`export GAZEBO_MODEL_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/models:/home/user/catkin_ws/src/simulation_control/src/models`
11. Add the file `f550_amazing` to `/home/user/src/Firmware/posix-configs/SITL/init/lpe`

# FAQ #
 ## If after making `posix_sitl`, Gazebo doesn’t launch and gives you a error message regarding `geographiclib` ##
 Do the following:
  * In your terminal type:  `sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`
  * When that installation is ready type: 
  * `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`
  * `./install_geographiclib_datasets.sh` (you might have to make the install_geographiclib_datasets.sh executable.)
