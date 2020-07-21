#include <ros/ros.h>
#include "TestKuki.h"
#include "iiwa_wrapper.hpp"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "test_kuki");

  ros::NodeHandle n;
  float frequency = 400.0f;

  iiwa_wrapper kuki(n);

  TestKuki testKuki(n,frequency, kuki);
  //ros::Subscriber _subRobArm = n.subscribe("/robArm/cmd", 10, &MoveRobotWithGripper::callBackRob, &moveRobotWithGripper);

  if (!testKuki.init()) 
  {
    return -1;
  }
  else
  {
    testKuki.run();
  }

  return 0;

}