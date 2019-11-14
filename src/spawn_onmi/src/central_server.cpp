#include "ros/ros.h"
#include "spawn_onmi/node.h"
#include "spawn_onmi/initnode.h"
#include <ros/console.h>
#include <utility.h>
#include <string> 


void printMap(){
  for (int i = 0; i < Utility::nRowMap; i++) 
  { 
    std::string temp;
     for (int j = 0; j < Utility::nColMap; j++) 
     { 
       temp = temp + std::to_string(Utility::Map[i][j]) + " "; 
     } 
     // std::cout << endl; 
     ROS_DEBUG_STREAM("Server: " << temp);
  }
}

bool response_node(spawn_onmi::node::Request  &req,
         spawn_onmi::node::Response &res)
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
  if(res.isComplete){
      ROS_DEBUG_STREAM("Server: Exploration is complete!");
  }

  int resNextRow, resNextCol;
  res.isNext = Utility::pickNext(req.cRow, req.cCol,resNextRow, resNextCol);

  if(res.isNext){
    res.nRow = resNextRow;
    res.nCol = resNextCol;
  }  
  printMap();
  return true;
}

bool response_initnode(spawn_onmi::initnode::Request  &req,
         spawn_onmi::initnode::Response &res){

  bool health = Utility::initMap(req.robotRow, req.robotCol);
  res.clash = !(health);

  printMap();

  return true;

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_server");
  ros::NodeHandle n("~");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  std::string mapAbsPath;
  int mapRow;
  int mapCol;

  n.getParam("map_row", mapRow);
  n.getParam("map_col",mapCol);
  n.getParam("map_abs_path",mapAbsPath);

  bool cond1 = Utility::updateParams(mapRow,mapCol);
  bool cond2 = Utility::getFileContent(mapAbsPath);

  if(cond1 != true || cond2 != true){
    ROS_ERROR_STREAM("Server: Error while loading the map_abs_path file ");
    ROS_ERROR_STREAM("Server: Please check the size of map_row and map_col matches the input in map_abs_path file");
    return 0;

  } 

  ros::ServiceServer service = n.advertiseService("node_info", response_node);
  ros::ServiceServer service_initnode = n.advertiseService("node_init", response_initnode);

  printMap();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}