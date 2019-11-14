#include "ros/ros.h"
#include "robot_laser/node.h"
#include "robot_laser/initnode.h"
#include <ros/console.h>
#include <utility.h>



void printMap(){
  for (int i = 0; i < Utility::nRowMap; i++) 
  { 
    std::string temp;
     for (int j = 0; j < Utility::nColMap; j++) 
     { 
       temp = temp + std::to_string(Utility::Map[i][j]) + " "; 
     } 
     ROS_DEBUG_STREAM("Server: " << temp);
  }
}


bool response_infonode(robot_laser::node::Request  &req,
         robot_laser::node::Response &res)
{

  bool health = Utility::updateMap(req.cRow, req.cCol);

  if(health == false){
    ROS_ERROR_STREAM("Server: Something went wrong while updating the map");
    return false;
  }
  
  // default values
  res.nRow = req.cRow;
  res.nCol = req.cCol;
  res.isComplete = Utility::isComplete();

  int resNextRow, resNextCol;

  bool isNext = Utility::pickNext(req.cRow, req.cCol,resNextRow, resNextCol);
  if(isNext){
    res.nRow = resNextRow;
    res.nCol = resNextCol;
  }
  res.isNext = isNext;

  // check if all the nodes are either 0,-1,1
  if(res.isComplete){
      ROS_DEBUG_STREAM("Server: Exploration is complete");
    }
  printMap();
  return true;
}

bool response_initnode(robot_laser::initnode::Request  &req,
         robot_laser::initnode::Response &res){

  bool health = Utility::initMap(req.robotRow, req.robotCol, req.val);

  res.clash = !(health);

  return true;

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_server");
  ros::NodeHandle n("~");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  int mapRow;
  int mapCol;

  n.param("map_row", mapRow, 10);
  n.param("map_col",mapCol, 10);

  ros::ServiceServer service = n.advertiseService("node_info", response_infonode);
  ros::ServiceServer service_initnode = n.advertiseService("node_init", response_initnode);

  
  bool cond1 = Utility::updateParams(mapRow,mapCol);

  if(cond1 == false ){
    ROS_DEBUG_STREAM("Server: map_row or map_col == 0, please check parameters again");
    return 0;
  }

  printMap();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}