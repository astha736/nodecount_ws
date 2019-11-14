# Node Counting - Experimental Robotics Assignment 1 

This project is intented as a submission for Assignment 1 of Experimental Robotics couse at DIBRIS, University of Genoa. 

## Author(Team)
	Astha Gupta
	Student ID: S4899512

## Description

The submission contains two packages 
* spawn_omni : Submission for Node counting, where the map of the world is given 
* robot_laser : Submission for Node counting, where map is not given, but explored by the robots using cameras. 

The code for spawn_omni and robot_laser are pretty much smilar, the idea is to have a server node that has a map(given by user for spawn_omni, or a empty matrix for robot_laser). The server has the responsibility of updating and providing information about the map.The communication between robots(clients) and the server and is through two services provided by the server(node_init and node_info) the robots. 

### spawn_omni

Assumptions:
* environment: empty_world
* Values in a matrix: 
	* 0 -> empty 
	* 1 -> explored 
	* -1-> wall/blocked 
	* -2-> The node has been given as goal(to one of the robots)
	* Note: once a node has been allocated status 1, though robots might trace back to their previous location, the status does not change   


* config/
	* map1_6_10.txt: a example map of size 6x10
	* map2_8_10.txt: a example map of size 8x10
* include/
	* utility.h: A utility file used by central_server 
* launch/
	* robot1_map1.launch: map1_6_10.txt with 1 robot
	* robot1_map2.launch: map2_8_10.txt with 1 robot
	* robot2_map1.launch: map1_6_10.txt with 2 robots
	* robot2_map2.launch: map2_8_10.txt with 2 robots
	* robot3_map2.launch: map2_8_10.txt with 3 robots
	* rviz_test.launch  : for loading robots in rviz 
* src/
	* central_server.cpp: advertises two services "node_info" and "node_init"
		
		* node_init: used to initialize the robot's start position by giving (row,column). The checks insure that there can be only one robot at a given position. 
		
		* node_info: used throughout the code, each time when robot has reached a given goal location and asks for a new goal location. If the next goal is present then the robot updates the goal position and move towards it, if there is no next goal, then the robot traces back to its previous goal location and asks server for a new goal. 
		All the goals are saved in a stack, in the case when the stack is empty and there are no next goals given by the Server, the robot kills itself. 

	* robot_client.cpp  : The logic for robot clients is very simple, they loop over callback for odometry, check if the goal has been reached. If yes, then ask server for a new goal, if the server gives a new goal, update the velocity towards the new goal, otherwise go back to previous goal location and ping the server again. 

	* utility.cpp       : has few functions and datastructure to support the central_server. 

* srv/
	* initnode.srv : for init_node service 
	* node.srv     : for info_node service 
* urdf/
	* robot5.gazebo 
	* robot5.xacro


### robot_laser

Assumptions:
* environment: maze.world
* Values in a matrix: 
	* 0 -> empty 
	* 1 -> explored 
	* -1-> wall/blocked 
	* -2-> The node has been given as goal(to one of the robots)
	* Note: once a node has been allocated status 1, though robots might trace back to their previous location, the status does not change   


* config/
	* maze.world
* include/
	* utility.h: A utility file used by central_server 
* launch/
	* robot1.launch:  1 robot
	* robot2.launch:  2 robot
* src/
	* central_server.cpp: advertises two services "node_info" and "node_init"
		
		* node_init: used to initialize the robot's start position by giving (row,column). The checks insure that there can be only one robot at a given position. 
		Adittionally it is also used to update the map when the robot sees a obstacle at some node. 

		* node_info: used throughout the code, each time when robot has reached a given goal location and asks for a new goal location. The server just returns location of adjcent node that has 0, if no adjacent node has 0, then isNext is set to false. 

	* robot_client.cpp  : The logic for robot clients is very simple
```
	loop over callback for odometry 
		check if the goal has been reached. 
			If yes:
				srvice: then ask server for a new goal, 
				if the server gives a new goal:
					check if the goal is blocked using laserscan 
						if yes: srvice update the node as a wall 
						otherwise: update the velocity towards the new goal, 
				 otherwise go back to previous goal location (as a goal)
			otherwise:
				update the velocity
```

	* utility.cpp       : has few functions and datastructure to support the central_server. 

* srv/
	* initnode.srv : for init_node service 
	* node.srv     : for info_node service 
```
 node.srv
		int64 robotRow
		int64 robotCol
		int64 val -> 1 when initializing a robot, -1 when initializing a wall
		---
		bool clash
```
* urdf/
	* robot5.gazebo : Not being used 
	* robot5.xacro : Not being used 
	* robot_test.gazebo : Used, updated to have a camera  
	* robot_test.gazebo: Used, updated to have a camera  