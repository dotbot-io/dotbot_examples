#ifndef __NodeExample_HPP__
#define __NodeExample_HPP__

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"

#include <string>

class SimpleNode {
private:
  ros::NodeHandle _nh;
  ros::Publisher _pub_end;
  ros::Publisher _pub_hp;
  ros::Publisher _pub_speed;
  ros::Publisher _pub_attack;

  ros::Subscriber _sub_turn;
  ros::Subscriber _sub_move;
  ros::Subscriber _sub_attack;

  int _hp;
  void turnCb(const std_msgs::Empty & msg);
  void startCb(const std_msgs::Empty & msg);
  void attackCb(const std_msgs::UInt8 & msg);

  std::string _other, _name;

public:
  SimpleNode();
  void run();

  int get_hp();
  void set_hp(int _hp);
  void inc_hp(int increment);
};

#endif
