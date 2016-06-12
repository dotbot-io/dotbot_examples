#include "ros/ros.h"
#include "dotbot_msgs/GetRobotInfo.h"
#include "dotbot_msgs/RobotInfo.h"


dotbot_msgs::RobotInfo robot;


bool getCb(dotbot_msgs::GetRobotInfo::Request  &req,
         dotbot_msgs::GetRobotInfo::Response &res)
{
  res.robot = robot;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_robot_info_server");
  ros::NodeHandle n;

  robot.name = "black";
  robot.hp_max = 100;
  robot.atk = 50;
  robot.def = 90;
  robot.spd = 30;
  robot.type = "fire";

  robot.m1.name = "Quick Attack";
  robot.m1.damage = 40;
  robot.m1.atk_mod = 0;
  robot.m1.def_mod = 0;
  robot.m1.priority = true;
  robot.m1.type = "normal";
  robot.m1.cure = 0;

  robot.m2.name = "Flamethrower";
  robot.m2.damage = 60;
  robot.m2.atk_mod = 0;
  robot.m2.def_mod = 0;
  robot.m2.priority = false;
  robot.m2.type = "fire";
  robot.m2.cure = 0;

  robot.m3.name = "Water Dance";
  robot.m3.damage = 0;
  robot.m3.atk_mod = 2;
  robot.m3.def_mod = 0;
  robot.m3.priority = false;
  robot.m3.type = "water";
  robot.m3.cure = 0;

  robot.m4.name = "Synthesis";
  robot.m4.damage = 0;
  robot.m4.atk_mod = 0;
  robot.m4.def_mod = 0;
  robot.m4.priority = false;
  robot.m4.type = "grass";
  robot.m4.cure = 50;


  ros::ServiceServer service = n.advertiseService("black/get_robot_info", getCb);
  ROS_INFO("Read to get info.");
  ros::spin();

  return 0;
}
