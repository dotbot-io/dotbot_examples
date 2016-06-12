#include "RobotCommunication.hpp"
#include "RefeeNode.hpp"
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "referee");

  RefeeNode node;
  node.run();


  return 0;
}
