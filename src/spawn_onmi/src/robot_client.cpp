#include "ros/ros.h"
#include "spawn_onmi/node.h"
#include "spawn_onmi/initnode.h"
#include <ros/console.h>
#include <cstdlib>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <stack> 
#include <utility> // for the pair 
#include <cmath>


ros::Publisher pub;
ros::ServiceClient client;

float goalX;
float goalY;

std::stack<std::pair<int,int>> nodeStack;
std::string robotSpace;

float threshNode;;
float vehicleSpeed;

bool nodeReached(float x, float y){
  // if node is 0.2 away
  float sum = std::sqrt(std::pow(goalX-x,2) + std::pow(goalY-y,2));
  if(sum < threshNode){
    return true;
  }
  else return false;
}

void setVelocity(float x, float y){

  double norm_dir = std::sqrt(std::pow(goalX-x,2) + std::pow(goalY-y,2));
  
  geometry_msgs::Twist vel;
  
  vel.linear.x = double ((goalX-x)/ norm_dir)*vehicleSpeed;
  vel.linear.y = double ((goalY-y)/ norm_dir)*vehicleSpeed;

  pub.publish(vel);

}

void stopRobot(){
  geometry_msgs::Twist vel;
  vel.linear.x =0;
  vel.linear.y =0;
  pub.publish(vel);
}



void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
   {

    float cX = msg->pose.pose.position.x;
    float cY = msg->pose.pose.position.y;

    if(nodeReached(cX,cY)){
      stopRobot();

      spawn_onmi::node srv;
      srv.request.cRow =  goalX;
      srv.request.cCol =  goalY;

      // service request to get the next node to go
      if (client.call(srv)){

        if(srv.response.isComplete == true){
            ROS_INFO_STREAM(robotSpace << " : Dear Human, all the nodes has been explored");
            ros::shutdown();
        }

        if(srv.response.isNext == true){
          // save previous achived goals
          nodeStack.push(std::make_pair(goalX, goalY));
          // update the goals
          goalX = srv.response.nRow;
          goalY = srv.response.nCol;
          setVelocity(cX,cY);
          ROS_INFO_STREAM(robotSpace << " : Next (" << goalX << ", " << goalY << ")");
        }
        else{
          // trace back to previous goal
          if(!nodeStack.empty()){
            std::pair<int,int> prev = nodeStack.top();
            nodeStack.pop();
            goalX = prev.first;
            goalY = prev.second;
            setVelocity(cX,cY);
            ROS_INFO_STREAM(robotSpace << " : Back (" << goalX << ", " << goalY << ")");
          }
          else{
            // stack is empty, returned to initial location
            ROS_INFO_STREAM(robotSpace << " : Dear Human, I have returned to my initial position, no new place to go");
            ros::shutdown();
          }
        }
      }
      else{
        // if service request failed
        ROS_ERROR("Failed to call service node_info");
        return;
      }

    }
    else{
      // update velocity to reach the current goal
      setVelocity(cX,cY);
    }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_client");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  ros::NodeHandle n("~");

  n.getParam("robot_space", robotSpace);
  n.param("thresh_node",threshNode, float(0.2));
  n.param("vehicleSpeed",vehicleSpeed, float(1));


  int robotRow, robotCol;
  n.getParam("robot_row",robotRow);
  n.getParam("robot_col",robotCol);

  
  std::string cmdVel = "/" + robotSpace + "/" +  "cmd_vel";
  std::string odom = "/" + robotSpace + "/" +  "odom";
  
  pub = n.advertise<geometry_msgs::Twist>(cmdVel, 5);
  
  ros::Subscriber sub = n.subscribe(odom, 1000, odomCallback);

  client = n.serviceClient<spawn_onmi::node>("/centralserver/node_info");
  ros::ServiceClient client_initnode = n.serviceClient<spawn_onmi::initnode>("/centralserver/node_init");


  // set the location of robot as 1 in the map
  spawn_onmi::initnode srv_init;
  srv_init.request.robotRow =  robotRow;
  srv_init.request.robotCol =  robotCol;
  while(true){
    if(client_initnode.call(srv_init)){
      if(srv_init.response.clash == false) {
        goalX = robotRow;
        goalY = robotCol;
        break;
      }
      else{
        ROS_ERROR_STREAM(robotSpace << " : The start location should be unique and not be a wall(-1)");
        return 0;
      }
    }
  }


  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
   return 0;

}