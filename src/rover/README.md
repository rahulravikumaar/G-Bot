# **Rover**

## **Idea**
A rover that can be controlled with your hand gestures!
 
*This package has been created and tested on Ubuntu 22.04 with ROS2 Humble and Gazebo 11.10.2*

## **How to build**
*Creating a workspace to build the package*
```
mkdir -p ~/rover_ws/src && cd ~/rover_ws/src
```
*Cloning the package*
```
git clone -b hand_gestures https://github.com/kalashjain23/rover
cd ~/rover_ws
```
*Installing the dependencies and building the workspace*
```
rosdep install --from-paths src -y --ignore-src
colcon build
```

## **Controlling the rover**
#### **Available gestures**: 
Victory Sign (✌️) --> Go straight  
Pointing finger (☝️) ( left tilted ) --> Turn left on its spot  
Pointing finger (☝️) ( right tilted ) --> Turn right on its spot  
Raised Hand (✋) ( left tilted ) --> Turn left and go straight as well  
Raised Hand (✋) ( right tilted ) --> Turn right and go straight as well  
Fist (✊) --> Stop  
```
# source the workspace
source ~/rover_ws/install/setup.bash

# Launching Gazebo and controllers
ros2 launch rover launch_sim.launch.py world:=./src/rover/worlds/obstacles.world

# Running the controller node
ros2 run rover controller.py
```

*Every movement is recorded and is saved into a file named <u>history.txt</u>*

