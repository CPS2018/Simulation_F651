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
13. `roslaunch px4 posix_sitl.launch`
14. `roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14557"`

# Gripper Setup #

1. Open the terminal and write ""printenv | grep GAZEBO"

2. Two or more paths should be displayed:
   GAZEBO_MODEL_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/models:/home/user/catkin_ws/src/simulation_control/src/models
   GAZEBO_PLUGIN_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/Build

3. Gazebo gets it's models in this case from two paths which are told apart by the : sign.

4. Add the file gripper_plugin.cc in the folder one step up from your GAZEBO_PLUGIN_PATH which in this case would be "/home/user/src/Firmware/Tools/sitl_gazebo/src"

5. Add the following under plugins to the CMakeLists.txt file located at "/home/user/src/Firmware/Tools/sitl_gazebo/src"
	# -----GripperPlugin
	find_package(roscpp REQUIRED)
	find_package(std_msgs REQUIRED)
	include_directories(${roscpp_INCLUDE_DIRS})
	include_directories(${std_msgs_INCLUDE_DIRS})	
	find_package(gazebo REQUIRED)
	include_directories(${GAZEBO_INCLUDE_DIRS})
	link_directories(${GAZEBO_LIBRARY_DIRS})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

	add_library(gripper_plugin SHARED src/gripper_plugin.cc)
	target_link_libraries(gripper_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
	# -----/GripperPlugin

6. Type in TERMINAL:
	cd ~/home/user/src/Firmware/Tools/sitl_gazebo/Build  (GAZEBO_PLUGIN_PATH)
	cmake ../
	make

7. Add the folder "simulation_control" to your "catkin_ws/src" folder.

8. Type in TERMINAL: (building the catkin workspace)
	cd ~/catkin_ws
	catkin build

9. Double check if you're missing any dependencies like pysci etc. and install them if not. Then build again.
   If you want it to work there can be no errors in the build.

10. Type in terminal: (adding gazebo model path)
	sudo nano ~/.bashrc

11. Add the following line to the end of your GAZEBO_MODEL_PATH:
	:/home/user/catkin_ws/src/simulation_control/src/models

   For example if it looks like this:
	export GAZEBO_MODEL_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/models

   After editing it should look like this:
	export GAZEBO_MODEL_PATH=:/home/user/src/Firmware/Tools/sitl_gazebo/models:/home/user/catkin_ws/src/simulation_control/src/models

12. Add the file f550_amazing to "/home/user/src/Firmware/posix-configs/SITL/init/lpe".


# FAQ #

 ## If after making `posix_sitl`, Gazebo doesn’t launch and gives you a error message regarding `geographiclib` ##
 Do the following:
  * In your terminal type:  `sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`
  * When that installation is ready type: 
  * `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`
  * `./install_geographiclib_datasets.sh` (you might have to make the install_geographiclib_datasets.sh executable.)
