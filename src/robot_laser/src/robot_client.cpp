#include "ros/ros.h"
#include "robot_laser/node.h"
#include "robot_laser/initnode.h"
#include <ros/console.h>
#include <cstdlib>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include <stack> 
#include <utility> // for the pair 
#include <cmath>


ros::Publisher pub;

int goalX;
int goalY;

float currentRobotX;
float currentRobotY;

std::stack<std::pair<int,int>> nodeStack;
std::string robotSpace;

float threshNode;;
float vehicleSpeed;

int robotStop =0;

ros::ServiceClient client_infonode;
ros::ServiceClient client_initnode;

std::vector<float> checkAngles{3.14159, 1.5708, 0, -1.5708};


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

bool checkGoalInScan(const int gX,const int gY, const sensor_msgs::LaserScan& msg ){
  // check if given goalX and goalY the position is empty  
  // we check nearest for +-.5 rad 
  float angle_inc = msg.angle_increment;
  int range_inc = 0.20/angle_inc;
  int size_range = (msg.angle_max - msg.angle_min)/angle_inc;

  int cX = round(currentRobotX);
  int cY = round(currentRobotY);

  if(gX - cX == 0){
    if (gY - cY > 0){
      // case for +90
      int idx =  round((1.5708 - msg.angle_min)/angle_inc);
      for(int i = idx-range_inc; i < idx + range_inc; i++){
        if(msg.ranges[i] < 1.2){
          // it means that there is something in less than 1.5 radius 
          return 0;
        }
      }
      return 1;
    }
    else{
      // case for -90
      int idx =  round((-1.5708 - msg.angle_min)/angle_inc);
      for(int i = idx-range_inc; i < idx + range_inc; i++){
        if(msg.ranges[i] < 1.2){
          // it means that there is something in less than 1.5 radius 
          return 0;
        }
      }
      return 1;

    }
  }else{
    if(gX - cX > 0){
      // case of 0
      int idx =  round((0 - msg.angle_min)/angle_inc);
      for(int i = idx-range_inc; i < idx + range_inc; i++){
        if(msg.ranges[i] < 1.2){
          // it means that there is something in less than 1.5 radius 
          return 0;
        }
      }
      return 1;

    }
    else{
      // case of 180
      // check the first few 
      int idx = 0;
      for(int i = idx; i < idx + range_inc; i++){
        if(msg.ranges[i] < 1.2){
        	 
          // it means that there is something in less than 1.5 radius 
          return 0;
        }
      }
      // check the last few
      idx = size_range;
      for(int i = idx-range_inc; i < idx; i++){
        if(msg.ranges[i] < 1.2){
          // it means that there is something in less than 1.5 radius 
          return 0;
        }
      }
      return 1;

    }
  } 
}

void laserCallBack(const sensor_msgs::LaserScan& msg){
  if(robotStop){

      robot_laser::node srv_info;
      srv_info.request.cRow =  round(currentRobotX);
      srv_info.request.cCol =  round(currentRobotY);
      
      // service request 
      if (client_infonode.call(srv_info)){

        if(srv_info.response.isComplete == true){
            ROS_INFO_STREAM(robotSpace << ": Dear Human, all the nodes has been explored");
            ros::shutdown();
        }

        if(srv_info.response.isNext == true){

          bool isFree = checkGoalInScan(srv_info.response.nRow,srv_info.response.nCol,msg );

          if(isFree == false){

          	ROS_INFO_STREAM(robotSpace << " : Wall "<< "(" << srv_info.response.nRow << "," << srv_info.response.nCol << ")");

            // send a service such that it updates that the given goal was -1
              robot_laser::initnode srv_wall;
              srv_wall.request.robotRow =  srv_info.response.nRow;
              srv_wall.request.robotCol =  srv_info.response.nCol;
              srv_wall.request.val =  -1;
              
              while(true){
                if(client_initnode.call(srv_wall)){
                  if(srv_wall.response.clash == false) {
                    break;
                  }
                  else{
                    ROS_ERROR_STREAM(robotSpace << ": The location could not be updated as a wall");
                    return;
                  }
                } // wait till a reponse from srv!
              }

            return;
          }

          ROS_INFO_STREAM(robotSpace << " : Next "<< "(" << srv_info.response.nRow << "," << srv_info.response.nCol << ")");

          nodeStack.push(std::make_pair(goalX, goalY));
          
          // update the goals
          goalX = srv_info.response.nRow;
          goalY = srv_info.response.nCol;
          setVelocity(currentRobotX,currentRobotY);
          robotStop = 0;
        }
        else{
          // go back to the previous place
          if(!nodeStack.empty()){
            std::pair<int,int> prev = nodeStack.top();
            nodeStack.pop();
            goalX = prev.first;
            goalY = prev.second;
            setVelocity(currentRobotX,currentRobotY);
            robotStop = 0;
          }
          else{
            ROS_INFO_STREAM(robotSpace << ": Dear Human, my mission is accomplished,there is no place to go");
            ros::shutdown();
          }
        }
      }
      else{
        // if service request failed
        ROS_ERROR_STREAM(robotSpace << ": Service node_info returned false");
        return;
      }
  }
}



void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
   {

    if(robotStop) return;

    currentRobotX = msg->pose.pose.position.x;
    currentRobotY = msg->pose.pose.position.y;

    if(nodeReached(currentRobotX,currentRobotY)){
      stopRobot();
      robotStop = 1;

    }
    else{
      // update the velocity to reach the current goal
      setVelocity(currentRobotX,currentRobotY);
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
  std::string lasersan = "/" + robotSpace + "/" +  "laserscan";

  client_infonode = n.serviceClient<robot_laser::node>("/centralserver_robot/node_info");
  client_initnode = n.serviceClient<robot_laser::initnode>("/centralserver_robot/node_init");
  
  pub = n.advertise<geometry_msgs::Twist>(cmdVel, 5);
  
  ros::Subscriber sub = n.subscribe(odom, 1000, odomCallback);
  ros::Subscriber laserSub = n.subscribe(lasersan,10, laserCallBack);


  robot_laser::initnode srv_init;
  srv_init.request.robotRow =  robotRow;
  srv_init.request.robotCol =  robotCol;
  srv_init.request.val =  1;

  while(true){
    if(client_initnode.call(srv_init)){
      if(srv_init.response.clash == false) {
        goalX = robotRow;
        goalY = robotCol;
        break;
      }
      else{
        ROS_ERROR_STREAM(robotSpace << " : The start location should be unique and not be a wall");
        return 0;
      }
    } // wait till a reponse from srv!
  }

  goalX = robotRow;
  goalY = robotCol;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
   return 0;

}