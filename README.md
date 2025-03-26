# Behavior Tree Localization and Navigation

## Authors: Gregory Sarrouf, Alexa Fahel, Jacques Dergham

Our project is a turtlebot that navigates to four different waypoints in gazebo while monitoring its battery percentage and charging itself when needed.
The behavior tree used is this one

![Tree picture](bt_diagram.png)


## Building and Running the Project

```
git clone git@github.com:BehaviorTree/BehaviorTree.CPP.git  
cd BehaviorTree.CPP  
sudo apt-get install libzmq3-dev libboost-dev  
mkdir build  
cd build  
cmake ..  
make  
sudo make install  
```

```
cd ../.. 
mkdir build  
cd build  
cmake ..  
make  
```

Launch gazebo in one terminal:
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

And launchh RViz2 in another terminal:
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/path/to/map/world_map_save.yaml
```

Then add the 2D point estimate on the Rviz2 map to approximate the robot's location and run the following commands from your project’s directory
```
cd build  
./NavBT
```


