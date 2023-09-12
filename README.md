# NaSoNav: Natural Social Navigation Testbed

This simulation environment was created for the publication **NaSoNav: Natural Social Navigation Testbed** submitted to the International Conference on Robotics and Automation (ICRA 2024). This environment is presented as an alternative to evaluate social navigation algorithms in robots, taking into account real trajectories from the BIWI Walking Pedestrians dataset (https://icu.ee.ethz.ch/research/datsets.html), as well as the dynamics of the robot. This simulation environment allows the possibility to easily integrate new navigation algorithms, create new scenes and modify the speed at which the agents move.

## 1. Requirements

1. Ubuntu 20
2. ROS Noetic
3. Turtlebot2 Simulator in ROS

**Note**: We offer installation files that are designed to fully meet the prerequisites 2 and 3 on our GitHub repository at https://github.com/cafemesa/NoeticFocalInstaller.


## 2. Compile the solution

```
cd ~/catkin_ws/src
https://github.com/cafemesa/NaSoNav
chmod +x NaSoNav/scripts/*.py
cd ..
catkin_make
```

For this simulator we create an standard enviroment based on the observed in the dataset. For this reason, we desing some walls that help for the localization of the robot during the simulation. To run succesfully the simulator you need to add these models the the gazebo library. To do this, just copy the folders inside gazebo_models into ~/gazebo/model/ folder.

## 3. Run the simulator

1. First terminal
```
roscore
```

2. Second terminal

```
rosrun social_navigation_testbed run_experiment.py
```

The results of the experiment will be stored in annotations/tests/, and the general information about collisions and timeouts, will be stored en the folder where you run this script

## 4. Customize the simulator

To customize the simulator you can edit the run_experiment in the script folder as follow

1. Testing algorithms: The algorithms are defined in navigationMethods (line 16). As default  the simulator works with ["unaware", "sf", "rvo" , "sacadrl"]. If you want, is possible to remove from the simulator some algorithms just removing from the array. If you want to add a new one, add the name to the array and your controller node into the turtlebot2_mavigation_lidar.launch file in the launch folder in the lines 65-80 where the controllers are placed.

2. Testing speeds: as default we define 6 different simulation speeds. These speeds are the normalized maximal speeds of agent in the different scenes. These speeds are: 0.3, 0.4, 0.5, 0.7, 1.0 and 1.5. To avoid simulate one speed, just remove from the array. 

If you want to create simulated environments with different normalized speeds, edit the node file in src/create_simulated_environments.cpp, line 133. The array speedLimits[] contains the different speed limits. Then compile your workspace and run the following commant to generate the .world files:

```
roslaunch social_navigation_testbed create_simulated_environments.launch 
```

To run the simulator with the new speed modify the lines 23 and 24 in the run_experiment.py file. In line 23 the variables speedsValuesAll contain all the values the user can choose, add your new speed here. In the line 24, speedsValues contain the speed that will be executed for the experiment, add the desired ones here from the existing in the previous line.

3. Testing scenes: The scenes are defined in sceneIds (line 20). As default  the simulator works with the scenes 0 to 9. If you want, is possible to remove from the simulator some scenes just removing from the array. 

If you want to create a new simulated environment on another fragment of the dataset, edit the node file in src/create_simulated_environments.cpp, lines 55-57. Here you can define the initFrame, the endFrame and the personId that willl be removed from the scene and be replaced with the robot. Then compile your workspace and run:

```
roslaunch social_navigation_testbed create_simulated_environments.launch 
```

To run the simulator with the new scene modify the lines 20 in the run_experiment.py file. In line 20 the variable sceneIds contain all the ids of scenes the will be executed, add here your sceneID. In simple words the sceneId is the index of the elements you added in the initFrame, endFrame and the personId arrays. i.e. if you add a new scene after the existing one the arrays will looks as follow, in this case your sceneId will be 10.

```
  int initFrames[] = {824, 2199, 4249, 6299, 7299, 7749, 8349, 9624, 10324, 10625, xxxxx};
  int endFrames[] = {1026, 2399, 4449, 6474, 7449, 7899, 8499, 9724, 10499, 10950, yyyyy};
  int hiddenPersonsIds[] = {7, 48, 77, 124, 150, 159, 189, 226, 273, 303, zzz};
```

## 5. Other tools

1. Path irregularity calculation.

As part of the simulator, we have added the file path_irregularity.py. This script will calculate the irregularity of the path for the conducted experiments. You only need to modify lines 11-13. In line 11, you should change the scene IDs, in line 12 the velocities, and in line 13 the navigation algorithms. The script will print the results in the terminal for the scenarios with the selected combinations. To run the script execute the following commands.
```
cd ~/catkin_ws/src/NaSoNav
python path_irregularity.py
```

2. Plot resulnting paths.

As part of the simulator, we have added the file graph_paths.py. This script will plot the path of the agent and robot in the selected results files. You only need to modify lines 40 and 41. In line 40, you should add the labels of each path you will plot and in line 41 you should add the corresponding files with the imprmation to be plotted. The paths are stores in the annotation folder, in "model" are the results for the expermient with the selected agent and in "tests" are the results with the robots. the files follow the following pattern:

sceneX_Y.Y_Z_agentPositionsFileName.csv (for the experiment with agent)
sceneX_Y.Y_Z_METHOD_robotPositionsFileName.csv (for the experiment with robot)

Where X is the sceneID, Y.Y is the normalized speed. Z is the indentifier that tell us if is a simulation with robot or person. And METHOD is the navigation algorith name.

```
cd ~/catkin_ws/src/NaSoNav
python graph_paths.py
```