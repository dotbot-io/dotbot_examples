#include "SimpleNode.hpp"
#include <iostream>
#include <cstdlib>

int SimpleNode::get_hp() { return this->_hp; }

void SimpleNode::set_hp(int _hp) {
  this->_hp = _hp;

  std_msgs::UInt8 hp_msg;
  hp_msg.data = (this->_hp > 0 ? this->_hp : 0);
  this->_pub_hp.publish(hp_msg);

  std::cout << "[" << _name << "]"<< " HP: " << this->_hp << std::endl;

}

void SimpleNode::inc_hp(int increment) {
  this->set_hp(this->_hp + increment);
}




void SimpleNode::run() {
  ros::spin();
}

void SimpleNode::turnCb(const std_msgs::Empty & m) {
  std_msgs::UInt8 msg;
  msg.data = rand()%100;
  std::cout << "SPD: " << (int)msg.data << std::endl;
  _pub_speed.publish(msg);
}

void SimpleNode::startCb(const std_msgs::Empty & m) {
  std_msgs::UInt8 msg;
  msg.data = rand()%30;
  std::cout << "[" << _name << "]"<< " ATK: " << (int)msg.data << std::endl;
  _pub_attack.publish(msg);

  std_msgs::Empty msg2;
  _pub_end.publish(msg2);
}

void SimpleNode::attackCb(const std_msgs::UInt8 & msg) {
  std::cout << "[" << _name << "]"<< " Received damane!" << std::endl;
  inc_hp(-msg.data);
}


SimpleNode::SimpleNode() {

  ros::NodeHandle private_nh("~");
  _name = ros::this_node::getNamespace();

  private_nh.param<std::string>("other", _other, "white");

  _pub_end = _nh.advertise<std_msgs::Empty>("move_ended", 1000);
  _pub_hp = _nh.advertise<std_msgs::UInt8>("hp", 1000);
  _pub_speed = _nh.advertise<std_msgs::UInt8>("move_speed", 1000);
  _pub_attack = _nh.advertise<std_msgs::UInt8>("attack", 1000);

  _sub_turn = _nh.subscribe("/new_turn", 1000, &SimpleNode::turnCb, this);
  _sub_move = _nh.subscribe("move_start", 1000, &SimpleNode::startCb, this);
  _sub_attack = _nh.subscribe("/" + _other + "/attack", 1000, &SimpleNode::attackCb, this);

  set_hp(100);

  srand(time(NULL));
}
