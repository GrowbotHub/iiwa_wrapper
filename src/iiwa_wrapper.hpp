#ifndef __IIWA_WRAP_HPP__
#define __IIWA_WRAP_HPP__

#include <signal.h>
#include "Eigen/Eigen"
#include <ros/ros.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/CartesianEulerPose.h>
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/TimeToDestination.h>
#include <iiwa_msgs/ConfigureControlMode.h>

#define JOINT 0
#define CART  1


class iiwa_wrapper
{
private:
    /* data */
    float deg2rad(float degrees);
    Eigen::Vector4f deg2quat(float A, float B, float C);
    ros::NodeHandle iiwa_nh;
    ros::Subscriber iiwa_subJointPos;
    ros::Subscriber iiwa_subCartPos;
    //ros::Subscriber _subJointDes;
    ros::Publisher iiwa_pubJointPos;
    ros::Publisher iiwa_pubCartPos;
    ros::Publisher iiwa_pubCartEuler;
    geometry_msgs::PoseStamped iiwa_CartPos;
    geometry_msgs::PoseStamped iiwa_RealCartPos;
    iiwa_msgs::JointPosition iiwa_msgCommand;
    iiwa_msgs::CartesianEulerPose iiwa_CartEuler;

    bool iiwa_fini = false;

    
    ros::ServiceClient iiwa_TimeToDestClient;
    iiwa_msgs::TimeToDestination config;

    // IIWA Stack status variables
    int iiwa_command_mode; // position, joint impedance, cartesian impedance etc
    int iiwa_pose_type; // joint or cartesian. Used for 




public:
    iiwa_wrapper(ros::NodeHandle &n/* args */);
    ~iiwa_wrapper();

    // Wait for the last send movement to be done. Locking process, but runs ros:spinOnce. 
    // If it times out, it kills the process to avoid problems with ros and the robot.s
    void waitForFinishedMovement(int timeoutSec);

    // Go to a given cartesian position
    void goToPosition(Eigen::Vector3f coord, Eigen::Vector3f orient);
    void goToPosition(float x, float y, float z, float a, float b, float c);
    void goToPosition(float x, float y, float z, float qx, float qy, float qz, float qw);

    // Go to a given joint position
    void goToJointDegrees(float a1, float a2, float a3, float a4, float a5, float a6, float a7);

    // Get the robot actual position, returns the values in the associated pointers
    void getRealRobotPosition(float* x, float* y, float* z, float* a, float* b, float* c);
    void getRealRobotJoint(float* a1,float* a2,float* a3,float* a4,float* a5,float* a6,float* a7);


    // Activates the Impedance Mode. Robot needs to be calibrated and having a tool selected.
    void CartesianImpedanceMode(float x, float y, float z, float a, float b, float c);
    void PositionMode();


    // Callback functions for the waitForFinishedMovement function. 
    void callBackCartPos(const iiwa_msgs::CartesianPose::ConstPtr& msg);
    void callBackPos(const iiwa_msgs::JointPosition::ConstPtr& msg);


};


#endif