#include "SimpleNode.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "referee");

  SimpleNode node;
  node.run();


  return 0;
}
