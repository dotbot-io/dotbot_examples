#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include <iostream>

// ============= RobotCommunication.hpp ================

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


// ============= RefeeNode.hpp ================

class RefeeNode {
private:
  ros::Publisher _turn_pub;
  RobotCommunication white, silver;
  RobotCommunication *first, *second, *ko_robot;
  int _turn_cnt;

  void turn();
  void start_turn();
  void move(RobotCommunication robot);
  bool check_ko();

public:

  ros::NodeHandle nh;
  RefeeNode();
  void run();
  void ko(RobotCommunication * who) { this->ko_robot = who; }
};




// ============= RobotCommunication.cpp ================


void RobotCommunication::_hpCb(const std_msgs::UInt8 & msg) {
  if (msg.data <= 0) {
    this->_referee->ko(this);
  }
}

void RobotCommunication::_stopCb(const std_msgs::Empty & msg) {
  this->_move_ended = true;
}

void RobotCommunication::_speedCb(const std_msgs::UInt8 & msg) {
  this->_speed = msg.data;
}

RobotCommunication::RobotCommunication(std::string name, RefeeNode * referee):
  _name(name),
  _referee(referee)
{
  std::cout << "Costruisco " << _name << std::endl;

}

void RobotCommunication::start_connections() {
  std::cout << "Inizio Connessioni " << _name << std::endl;
  this->_start_pub = _referee->nh.advertise<std_msgs::Empty>("/" + _name + "/move_start", 1000);
  this->_stop_sub = _referee->nh.subscribe("/" + _name + "/move_stop", 1000, &RobotCommunication::_stopCb, this);
  this->_hp_sub = _referee->nh.subscribe("/" + _name + "/hp", 1000, &RobotCommunication::_hpCb, this);
  this->_speed_sub = _referee->nh.subscribe("/" + _name + "/move_speed", 1000, &RobotCommunication::_speedCb, this);

}


void RobotCommunication::move() {
  this->_move_ended = false;
  std_msgs::Empty msg;
  this->_start_pub.publish(msg);
}


// ============= RefeeNode.cpp ================

void RefeeNode::turn() {
  this->start_turn();
  this->move(*first);
  if (this->check_ko()) return;
  this->move(*second);
}

void RefeeNode::start_turn() {
  white.resetSpeed();
  silver.resetSpeed();
  std_msgs::Empty msg;
  this->_turn_pub.publish(msg);
  _turn_cnt++;

  while (white.getSpeed() < 0 || silver.getSpeed() < 0) {
    ros::spinOnce();
  }

  if (white.getSpeed() > silver.getSpeed()) {
    first = &white;
    second = &silver;
  } else if (white.getSpeed() < silver.getSpeed()) {
      first = &silver;
      second = &white;
  } else if (rand()%2 == 1) {
      first = &silver;
      second = &white;
  } else {
    first = &white;
    second = &silver;
  }
}

void RefeeNode::move(RobotCommunication robot) {
  robot.move();
  while (!robot.move_ended()) {
    ros::spinOnce();
  }
}


bool RefeeNode::check_ko() {
  return ko_robot != NULL;
}

RefeeNode::RefeeNode():
_turn_cnt(0),
white("white", this),
silver("silver", this),
ko_robot(NULL)
{
  std::cout << "Costruisco RefeeNode" << std::endl;
  white.start_connections();
  silver.start_connections();
  this->_turn_pub = this->nh.advertise<std_msgs::Empty>("/new_turn", 1000);
}

void RefeeNode::run() {
  std::cout << "run" << std::endl;
  while( !this->check_ko() ) {
    std::cout << "nuovo turno" << std::endl;
    this->turn();
  }
  std::cout << "KO: " << ko_robot->name() << std::endl;
}


// ============= main.cpp ================

int main(int argc, char **argv)
{
  ros::init(argc, argv, "referee");

  RefeeNode node;
  node.run();


  return 0;
}
