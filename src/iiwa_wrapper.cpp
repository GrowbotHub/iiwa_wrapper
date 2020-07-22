#include "iiwa_wrapper.hpp"

using namespace std;



iiwa_wrapper::iiwa_wrapper(ros::NodeHandle &n/* args */)
{
  iiwa_nh = n;
  iiwa_pubJointPos = iiwa_nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition",10);
  iiwa_pubCartEuler = iiwa_nh.advertise<iiwa_msgs::CartesianEulerPose>("/iiwa/command/CartesianEulerPose",10);
  iiwa_pubCartPos = iiwa_nh.advertise< geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);
  iiwa_subJointPos = iiwa_nh.subscribe<iiwa_msgs::JointPosition>("/iiwa/state/JointPosition",10, &iiwa_wrapper::callBackPos, this);
  iiwa_subCartPos = iiwa_nh.subscribe<iiwa_msgs::CartesianPose>("/iiwa/state/CartesianPose",10, &iiwa_wrapper::callBackCartPos, this);
  iiwa_TimeToDestClient = iiwa_nh.serviceClient<iiwa_msgs::TimeToDestination>("/iiwa/state/timeToDestination");
}

iiwa_wrapper::~iiwa_wrapper()
{
}

void iiwa_wrapper::callBackPos(const iiwa_msgs::JointPosition::ConstPtr& msg){
  //ROS_INFO_STREAM("JointPosition received");
  if (iiwa_pose_type == JOINT){
    iiwa_fini=true;

    // compare actual position with wanted one (_msgCommand.position.aX)
    if (iiwa_msgCommand.position.a1 - msg->position.a1 > 0.0005 )
      iiwa_fini = false;
    if (iiwa_msgCommand.position.a2 - msg->position.a2 > 0.0005 )
      iiwa_fini = false;
    if (iiwa_msgCommand.position.a3 - msg->position.a3 > 0.0005 )
      iiwa_fini = false;
    if (iiwa_msgCommand.position.a4 - msg->position.a4 > 0.0005 )
      iiwa_fini = false;
    if (iiwa_msgCommand.position.a5 - msg->position.a5 > 0.0005 )
      iiwa_fini = false;
    if (iiwa_msgCommand.position.a6 - msg->position.a6 > 0.0005 )
      iiwa_fini = false;
    if (iiwa_msgCommand.position.a7 - msg->position.a7 > 0.0005 )
      iiwa_fini = false;
  }
}

void iiwa_wrapper::callBackCartPos(const iiwa_msgs::CartesianPose::ConstPtr& msg){
  //ROS_INFO_STREAM("CartesianPosition received"<< msg->poseStamped.pose.position.x);
  if (iiwa_pose_type == CART){
    iiwa_fini=true;
    iiwa_RealCartPos = msg->poseStamped;
    if (iiwa_CartPos.pose.position.x - msg->poseStamped.pose.position.x > 0.005 )
      iiwa_fini = false;
    if (iiwa_CartPos.pose.position.y - msg->poseStamped.pose.position.y > 0.005 )
      iiwa_fini = false;
    if (iiwa_CartPos.pose.position.z - msg->poseStamped.pose.position.z > 0.005 )
      iiwa_fini = false;
    if (iiwa_CartPos.pose.orientation.x - msg->poseStamped.pose.orientation.x > 0.001 )
      iiwa_fini = false;
    if (iiwa_CartPos.pose.orientation.y - msg->poseStamped.pose.orientation.y > 0.001 )
      iiwa_fini = false;
    if (iiwa_CartPos.pose.orientation.z - msg->poseStamped.pose.orientation.z > 0.001 )
      iiwa_fini = false;
    if (iiwa_CartPos.pose.orientation.w - msg->poseStamped.pose.orientation.w > 0.001 )
      iiwa_fini = false;
  }
}

void iiwa_wrapper::waitForFinishedMovement(int timeoutSec){
  iiwa_fini = false;
  int counter = 0;
  while(!iiwa_fini && counter < (timeoutSec*10)){//wait until move finished, timeout at 1 min. If timeout, kill the process
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ++counter;
  }
  if(counter >= timeoutSec*10){
    kill(getpid(), SIGINT);
    ROS_INFO_STREAM("Robot never arrived at position! Killing the process");
  }
  else{
    ROS_INFO_STREAM("Arrived at wanted position");
  }
}

void iiwa_wrapper::getRealRobotPosition(float* x, float* y, float* z, float* a, float* b, float* c){
  *x = iiwa_RealCartPos.pose.position.x;
}

void iiwa_wrapper::getRealRobotJoint(float* a1,float* a2,float* a3,float* a4,float* a5,float* a6,float* a7){

}

float iiwa_wrapper::deg2rad(float degrees){
  return degrees / 180.0 * 3.1415965;
}

Eigen::Vector4f iiwa_wrapper::deg2quat(float A, float B, float C){
  float ca = cos(deg2rad(A)/2);
  float sa = sin(deg2rad(A)/2);
  float cb = cos(deg2rad(B)/2);
  float sb = sin(deg2rad(B)/2);
  float cc = cos(deg2rad(C)/2);
  float sc = sin(deg2rad(C)/2);

  Eigen::Vector4f q;
  float w = cc * cb * ca + sc * sb * sa;
  float z = sc * cb * ca - cc * sb * sa;
  float y = cc * sb * ca + sc * cb * sa;
  float x = cc * cb * sa - sc * sb * ca;

  Eigen::Vector4f quaternion;

  quaternion = Eigen::Vector4f(x,y,z,w);  

  return quaternion;

}

void iiwa_wrapper::goToPosition(Eigen::Vector3f coord, Eigen::Vector3f orient){
  iiwa_pose_type = CART;
  // Select the base frame of the robot
  iiwa_CartPos.header.frame_id = "iiwa_link_0";
  // Put the XYZ coordinates into the payload
  iiwa_CartPos.pose.position.x = coord[0];
  iiwa_CartPos.pose.position.y = coord[1];
  iiwa_CartPos.pose.position.z = coord[2];
  // Transform the angles into a quaternion and populate the payload
  Eigen::Vector4f quat = deg2quat(orient[2],orient[1],orient[0]);
  iiwa_CartPos.pose.orientation.x = quat[0];
  iiwa_CartPos.pose.orientation.y = quat[1];
  iiwa_CartPos.pose.orientation.z = quat[2];
  iiwa_CartPos.pose.orientation.w = quat[3];
  // Publish the data to the roscore
  iiwa_pubCartPos.publish(iiwa_CartPos);

}

void iiwa_wrapper::goToPosition(float x, float y, float z, float a, float b, float c){
  iiwa_pose_type = CART;
  // Select the base frame of the robot
  iiwa_CartPos.header.frame_id = "iiwa_link_0";
  // Put the XYZ coordinates into the payload
  iiwa_CartPos.pose.position.x = x;
  iiwa_CartPos.pose.position.y = y;
  iiwa_CartPos.pose.position.z = z;
  // Transform the angles into a quaternion and populate the payload
  Eigen::Vector4f quat = deg2quat(c,b,a);
  iiwa_CartPos.pose.orientation.x = quat[0];
  iiwa_CartPos.pose.orientation.y = quat[1];
  iiwa_CartPos.pose.orientation.z = quat[2];
  iiwa_CartPos.pose.orientation.w = quat[3];
  // Publish the data to the roscore
  iiwa_pubCartPos.publish(iiwa_CartPos);

}

void iiwa_wrapper::goToPosition(float x, float y, float z, float qx, float qy, float qz, float qw){
  iiwa_pose_type = CART;
  // Select the base frame of the robot
  iiwa_CartPos.header.frame_id = "iiwa_link_0";
  // Put the XYZ coordinates into the payload
  iiwa_CartPos.pose.position.x = x;
  iiwa_CartPos.pose.position.y = y;
  iiwa_CartPos.pose.position.z = z;
  // Populate the payload
  iiwa_CartPos.pose.orientation.x = qx;
  iiwa_CartPos.pose.orientation.y = qy;
  iiwa_CartPos.pose.orientation.z = qz;
  iiwa_CartPos.pose.orientation.w = qw;
  // Publish the data to the roscore
  iiwa_pubCartPos.publish(iiwa_CartPos);
  
}

void iiwa_wrapper::goToJointDegrees(float a1, float a2, float a3, float a4, float a5, float a6, float a7){
  iiwa_msgCommand.position.a1= deg2rad(a1);
  iiwa_msgCommand.position.a2= deg2rad(a2);
  iiwa_msgCommand.position.a3= deg2rad(a3);
  iiwa_msgCommand.position.a4= deg2rad(a4);
  iiwa_msgCommand.position.a5= deg2rad(a5);
  iiwa_msgCommand.position.a6= deg2rad(a6);
  iiwa_msgCommand.position.a7= deg2rad(a7);

  iiwa_pubJointPos.publish(iiwa_msgCommand);

}

void iiwa_wrapper::PositionMode(){
  ros::ServiceClient cimClient = iiwa_nh.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
  iiwa_command_mode = 0;
  iiwa_msgs::ConfigureControlMode configCIM;
  configCIM.request.control_mode = 0;
  cimClient.call(configCIM);
}

void iiwa_wrapper::CartesianImpedanceMode(float x, float y, float z, float a, float b, float c){
  ros::ServiceClient cimClient = iiwa_nh.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
  iiwa_command_mode = 2;
  iiwa_msgs::ConfigureControlMode configCIM;
  configCIM.request.control_mode = 2;
  configCIM.request.cartesian_impedance.cartesian_stiffness.x = x;
  configCIM.request.cartesian_impedance.cartesian_stiffness.y = y;
  configCIM.request.cartesian_impedance.cartesian_stiffness.z = z;
  configCIM.request.cartesian_impedance.cartesian_stiffness.a = a;
  configCIM.request.cartesian_impedance.cartesian_stiffness.b = b;
  configCIM.request.cartesian_impedance.cartesian_stiffness.c = c;

  cimClient.call(configCIM);

}
