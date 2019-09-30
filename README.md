# Pra_Vale

This repository contains the ROS package and the V-REP files required to run the submission of Pra Vale team to function in the ROSI challenge.

The description of the competition, as well as the official V-REP files, can be found in the following repository: https://github.com/filRocha/rosiChallenge-sbai2019

# Description of repository:
This repository consists in the same structure as an ROS package. The folders are organized as follows:

- `config` - Contains file with simulation parameters

- `launch` - Contains files with the ROS' .launch type

- `msg` - Contains files with the ROS' launch type required to communicate with the simulation

- `resources` - Contains the competition's rules and banner

- `script` - Team's codes in python made to processing the data from simulation and robot's control

- `src` - Contains the C++ code to data processing of Velodyne  

- `urdf` - Contains the ROSI robot URDF model

- `vrep_content` - Contains the V-REP files required to simulation


# Installation
The codes were programming in the **Ubuntu 18.2** SO, with the **ROS Melodic** and **VREP 3.6.2 (rev.0)**.
In case your system already have installed ROS, VREP and the compettion base code, you can skip to step 5

# 1. ROS Melodic Installation
The detailed explanation on how to install ROS Melodic can be found in the follow link:
http://wiki.ros.org/melodic/Installation/Ubuntu

If you want only copy and paste the required commands, without knowing the details, follow the steps below:
``` 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
``` 

``` 
sudo apt update
sudo apt install ros-melodic-desktop-full
``` 

``` 
sudo rosdep init
rosdep update
``` 

``` 
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
``` 

``` 
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
``` 

# 2. Setting Catkin Workspace
In the same way as the first item, the detailed explanation on how to configure the workspace can be found in the follow link:
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

If you want only copy and paste the required commands, follow the steps below:
``` 
sudo apt-get install ros-melodic-catkin python-catkin-tools
``` 

``` 
cd ~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
``` 

``` 
catkin_make
source devel/setup.bash
catkin init
``` 

# 3. Installing V-REP
**3.1** Download **V-REP PRO EDU V3.6.2 rev0** from the Coppelia Robotics website: http://www.coppeliarobotics.com/downloads.html


**3.2** Unzip it (preferentially) to your **home** folder and rename the folder as `vrep`.


**3.3** Add V-REP folder location to your `.bashrc`, an alias to run it, and source the `.bashrc` again: 
```
echo "export VREP_ROOT='<path_to_your_vrep_folder>'" >> $HOME/.bashrc
echo "alias vrep=$VREP_ROOT/vrep.sh" >> ~/.bashrc
source $HOME/.bashrc
```

**3.4** Test the V-REP functionality by running directly on your terminal the following command:

```
vrep
```

# 4. Installing competition base code
The detailed explanation about how to install the ROSI challenge base code is in the following link:
https://github.com/filRocha/rosiChallenge-sbai2019

In case you only want the terminal commands, they are listed below:
```
echo "export ROS_CATKIN_WS='<path_to_your_catkin_ws_folder>'" >> $HOME/.bashrc
echo "source $ROS_CATKIN_WS/devel/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc
```
``` 
cd ~/catkin_ws/src/
git clone https://github.com/filRocha/sbai2019-rosiDefy rosi_defy
``` 

```
cd $ROS_CATKIN_WS/src/
git clone --recursive https://github.com/CoppeliaRobotics/v_repExtRosInterface.git vrep_ros_interface
git clone https://github.com/filRocha/vrep_plugin_velodyne.git
```
``` 
sudo apt install python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joint-state-publisher
``` 

``` 
cd $ROS_CATKIN_WS
catkin clean
catkin build
source $HOME/.bashrc
``` 

``` 
echo -e "rosi_defy/ManipulatorJoints\nrosi_defy/RosiMovement\nrosi_defy/RosiMovementArray\nrosi_defy/HokuyoReading" >> $ROS_CATKIN_WS/src/vrep_ros_interface/meta/messages.txt
``` 

# 4.1 Set base code dependencies:
Add "<depend>rosi_defy</depend>" to 'catkin_ws/src/vrep_ros_interface/package.xml'

Add `rosi_defy` package dependence on the `catkin_ws/src/vrep_ros_interface/CMakeLists.txt`:
```
set(PKG_DEPS
  ... (many many other packages)
  rosi_defy
)
```
compile ros packpages
``` 
cd ~/catkin_ws
catkin build
``` 
copy the vrep-ros Interface
``` 
cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosInterface.so $VREP_ROOT
cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosVelodyne.so $VREP_ROOT
``` 
# 5. Checking Pra Vale codes dependencies
Before cloning the team's repository from git, it is necessary to check if some dependencies are updated in your system.

The following commands install or update the OpenCV libraries in your computer to enable python and C++ files

```
pip install opencv-python
pip install opencv-contrib-python
sudo apt-get install libopencv-dev
```
The following command install the keyboard library necessary in the arm_joy and arm_joint scripts
```
sudo pip install keyboard
```

# 6. Cloning Pra Vale submission
To access the Pra Vale's git repository, run the following commands in your terminal
``` 
cd $ROS_CATKIN_WS/src
git clone https://github.com/vinicius-r-silva/Pra_Vale.git pra_vale
``` 
When the repository is cloned, it is necessary to do the following changes.

Add the pra_vale package in the PKG_DEPS of the CMakeLists.txt, with the following commands
 
```
cd $ROS_CATKIN_WS/src/vrep_ros_interface
``` 
Open the CMakeLists.txt file with a text editor (gedit can be used) and add ```pra_vale``` in a new line inside the parentesis of the PKG_DEPS

After that, open the package.xml file with a text editor and, just like it seems in the file, add the following line

``` 
<depend>pra_vale</depend>
``` 
After that, run the following commands

``` 
cd $ROS_CATKIN_WS/
catkin clean
catkin build
``` 

With that, the repository setting is completed. 

# 7. Running the program
To run the program, it is necessary to do as listed below:

Open a terminal window and execute the roscore:
```
roscore
```

Add a new window in the terminal and open the vrep in the challenge scenario:
```
vrep $ROS_CATKIN_WS/src/pra_vale/vrep_content/challenge_scenario.ttt
```

Add one more window and run the following command:
```
roslaunch pra_vale rosi.launch
```

In the VREP, start the simulation and it is done.



