#ifndef __RefeeNode__HPP__
#define __RefeeNode__HPP__

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "RobotCommunication.hpp"

class RobotCommunication;

class RefeeNode {
private:
  ros::Publisher _turn_pub;
  RobotCommunication white, silver;
  RobotCommunication *first, *second, *ko_robot;
  int _turn_cnt;

  void turn();
  void start_turn();
  void move(RobotCommunication *robot);
  bool check_ko();

public:

  ros::NodeHandle nh;
  RefeeNode();
  void run();
  void ko(RobotCommunication * who) { this->ko_robot = who; }
};

#endif
