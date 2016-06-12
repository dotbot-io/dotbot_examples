#include "RefeeNode.hpp"
#include "RobotCommunication.hpp"

#include <iostream>



void RefeeNode::turn() {
  ros::Rate rate(0.5);
  rate.sleep();
  rate.sleep();
  this->start_turn();
  rate.sleep();

  this->move(first);

  if (this->check_ko()) return;
  rate.sleep();

  this->move(second);
}

void RefeeNode::start_turn() {
  white.resetSpeed();
  silver.resetSpeed();
  std_msgs::Empty msg;
  this->_turn_pub.publish(msg);
  _turn_cnt++;

  std::cout << "[Refee] waiting for robots!" << std::endl;
  ros::Rate rate(5);

  while (white.getSpeed() < 0 || silver.getSpeed() < 0) {
    rate.sleep();
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

void RefeeNode::move(RobotCommunication *robot) {
  std::cout << "[refee] moving robot: " << robot->name() << std::endl;
  robot->move();
  ros::Rate rate(5);

  while (!robot->move_ended()) {
    rate.sleep();
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
  _turn_pub = nh.advertise<std_msgs::Empty>("/new_turn", 1000);
}

void RefeeNode::run() {
  std::cout << "run" << std::endl;
  while( !this->check_ko() ) {
    std::cout << "nuovo turno" << std::endl;
    this->turn();
  }
  std::cout << "KO: " << ko_robot->name() << std::endl;
}
