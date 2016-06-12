#ifndef __RobotCommunication__HPP__
#define __RobotCommunication__HPP__

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"

class RefeeNode;

class RobotCommunication {

private:
  std::string _name;
  ros::Publisher _start_pub;
  ros::Subscriber _stop_sub;
  ros::Subscriber _hp_sub;
  ros::Subscriber _move_sub;
  ros::Subscriber _speed_sub;

  RefeeNode * _referee;
  int _speed;
  bool _move_ended;

  void _hpCb(const std_msgs::UInt8 & msg);
  void _stopCb(const std_msgs::Empty & msg);
  void _speedCb(const std_msgs::UInt8 & msg);

public:
  RobotCommunication(std::string name, RefeeNode * referee);
  void move();
  std::string name() { return _name; }
  bool move_ended() { return this->_move_ended; }
  void resetSpeed() { this->_speed = -1; }
  int getSpeed() { return this->_speed; }

  void start_connections();


};


#endif
