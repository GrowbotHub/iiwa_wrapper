#include "TestKuki.h"
//#include "Utils.h"
#include <boost/bind.hpp>
#include <iostream>
#include "iiwa_wrapper.hpp"
//#include <iiwa_ros/state/time_to_destination.hpp>

using namespace std;
TestKuki* TestKuki::me = NULL;

iiwa_wrapper* kuki = NULL;

TestKuki::TestKuki(ros::NodeHandle &n, float frequency, iiwa_wrapper &test):
  _n(n),
  _loopRate(frequency)
{
  ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
  //_subRobArm = _n.subscribe("/robArm/cmd", 10, &TestKuki::callBackRob);
  kuki = &test;
  //iiwa_subCartPos = iiwa_nh.subscribe<iiwa_msgs::JointPosition>("/iiwa/state/CartesianPose",10, iiwa_wrapper::callBackCartPos, this);
}

bool TestKuki::init() 
{
  me = this;


  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);
  _firstRobotPose = true;

  _stop = false;
  _toolOffsetFromEE = 0.0f;
  //_msgCommand.data.resize(6);
  /*for(int k = 0; k < 6 ;k++)
  {
    _msgCommand.data[k] = 0.0f;
  }*/
  signal(SIGINT,TestKuki::stopNode);

   ros::spinOnce();
  _loopRate.sleep();

  _reached = false;
  _fini=true;
  
  _aero= true;
  _pick=false;
  _potID=3;

  
  _first = false;

  if (_n.ok())
  { 
    // Wait for callback to be called
    ros::spinOnce();
    ROS_INFO("The ros node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void TestKuki::run() {

  std::cerr<<"pick"<< _pick <<std::endl;
  _fini=false;
  _id=0;
  int State = 0; //0 is homed, 1 is selection, 2 is over shelve, 3 is over pot, 4 is on pot, 5 waits the gripper to be closed/open, 6 for carousel moving
  int LastState = 0;

  int pot_selected = 0;



  ros::Duration(2.0).sleep();

  // init the robot
  _mutex.lock();
  // home the robot
  //homePosition();
  // Publish data to topics
  //publishData();
  _mutex.unlock();

  ros::Duration(2.0).sleep();
  kuki->PositionMode();

  kuki->goToPosition(0.620, 0.046, -0.645, -90, 0, -179);
  kuki->waitForFinishedMovement(30);
  //goToPosition(0.620, 0.046, -0.645, -0.707, 0.707, -0.00617, 0.00617);
  
  //ros::Duration(10.0).sleep();

  //kuki.goToPosition(0.620, 0.046, -0.8, -0.5, 0.5, -0.5, 0.5);
  kuki->CartesianImpedanceMode(2000.0, 2000.0, 100.0, 300.0, 300.0, 300.0);

  // go lower than the shelve
  kuki->goToPosition(0.620, 0.046, -0.645, -90, 0, -179);
  kuki->waitForFinishedMovement(30);

  float x,y,z,a,b,c;
  kuki->getRealRobotPosition(&x,&y,&z,&a,&b,&c);

  ROS_INFO_STREAM("Robot Real Position: X: "<< x << " Y: "<< y << " Z: " << z);

  
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);

  //publishData();

  //_gripper.fullOpen();

  ros::spinOnce();
  _loopRate.sleep();

  ros::Duration(2.0).sleep();

  //_gripper.reset();
  while(!_stop){
    ros::spinOnce();
    _loopRate.sleep();
  }

  ros::shutdown();
}

void TestKuki::stopNode(int sig)
{
  me->_stop = true;
}