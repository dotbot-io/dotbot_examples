#include "RobotCommunication.hpp"
#include "RefeeNode.hpp"
#include <iostream>



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
  std::cout << "[Refee] got speed from " << _name << ": " << this->_speed << std::endl;
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
  this->_stop_sub = _referee->nh.subscribe("/" + _name + "/move_ended", 1000, &RobotCommunication::_stopCb, this);
  this->_hp_sub = _referee->nh.subscribe("/" + _name + "/hp", 1000, &RobotCommunication::_hpCb, this);
  this->_speed_sub = _referee->nh.subscribe("/" + _name + "/move_speed", 1000, &RobotCommunication::_speedCb, this);

}


void RobotCommunication::move() {
  this->_move_ended = false;
  std_msgs::Empty msg;
  this->_start_pub.publish(msg);
}
