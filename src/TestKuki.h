#ifndef __TEST_KUKI_H__
#define __TEST_KUKI_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "iiwa_wrapper.hpp"

#include <ros/ros.h>
//#include "std_msgs/Float64MultiArray.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/CartesianEulerPose.h>
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/TimeToDestination.h>
#include <iiwa_msgs/ConfigureControlMode.h>
//#include <grasp_interface/r2f_gripper_interface.h>
//#include "growbot_msg/RobArm_cmd.h"
//#include "growbot_msg/RobArm_moving.h"
//#include "growbot_msg/Dispenser_moving.h"
//#include "growbot_msg/Dispenser_cmd.h"


class TestKuki 
{

	private:
		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		


		// Subscribers and publishers definition
		ros::Subscriber _subJointPos;
		//ros::Subscriber _subJointDes;
		ros::Publisher _pubJointPos;
		ros::Publisher _pubCartPos;
		ros::Publisher _pubCartEuler;
		geometry_msgs::PoseStamped _CartPos;
		iiwa_msgs::JointPosition _msgCommand;
		iiwa_msgs::CartesianEulerPose _CartEuler;
		//ros::Subscriber _subRobotPose;
		//ros::Subscriber _subRobArm;
		 // Subscribe to robot current pose
		//ros::Publisher _pubCommand;  // Publish desired orientation
		//ros::Publisher _pubFini;   
		//ros::Publisher _pubDispenser;

		ros::ServiceClient _TimeToDestClient;
		iiwa_msgs::TimeToDestination config;

		
		//std_msgs::Float64MultiArray _msgCommand;
		//growbot_msg::RobArm_moving _msgMoving;
		//growbot_msg::Dispenser_cmd _msgCom;

		// Node variables
		Eigen::Vector3f _x;
		Eigen::Vector3f _xd;
		Eigen::Vector4f _qd;
		Eigen::Vector3f _aa;
		Eigen::Vector3f _aad;
		Eigen::Vector4f _q;
		Eigen::Matrix3f _wRb;
		Eigen::Vector3f _omegad;
		Eigen::Vector3f _vd;
		bool _firstRobotPose;
		bool _aero;
		bool _pick;
		int nb;
		int _potID;
		float _toolOffsetFromEE;
		tf::TransformListener _lr;
		tf::StampedTransform _transform;

		double _reachedTime;

		Eigen::Vector3f _attractor[15];
		//Eigen::Vector3f _plants[8];
		Eigen::Vector4f _quat[10];
		int _id;
		bool _reached;

		// Class variables
		std::mutex _mutex;

		//r2fGripperInterface _gripper;
		bool _first;
		bool _fini;

		bool _stop = false;
		static TestKuki* me;
		//const Eigen::Vector4f _quat[10]=Eigen::Vector4f::Constant(10, Eigen::Vector4f(0.999f, -0.02f, 0.027f, 0.027f));
		const Eigen::Vector3f _plants[11]={Eigen::Vector3f (0.359f, 0.368f, 0.88f), Eigen::Vector3f (0.359f, 0.239f, 0.88f), Eigen::Vector3f (0.359f, -0.093f, 0.88f), Eigen::Vector3f (0.359f, -0.228f, 0.88f),Eigen::Vector3f (0.359f, -0.363f, 0.88f), Eigen::Vector3f(0.595f, 0.245f, 0.89f), Eigen::Vector3f(0.595f, 0.115f, 0.89f), Eigen::Vector3f(0.595f, -0.109f, 0.88f), Eigen::Vector3f (0.57f, -0.242f, 0.87f),Eigen::Vector3f (0.555f, -0.372f, 0.856f)};


	public:
		TestKuki(ros::NodeHandle &n, float frequency, iiwa_wrapper &test);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		//void callBackRobTime( const ros::Time& msg);
		void callBackRob( const iiwa_msgs::JointPosition::ConstPtr& msg);

		Eigen::Vector4f deg2quat(float A, float B, float C);
		void goToPosition(float x, float y, float z, float qx, float qy, float qz, float qw);
		void goToPosition(float x, float y, float z, float a, float b, float c);
		void waitFinished();

		void CartesianImpedanceMode();



	private:

		static void stopNode(int sig);

		void receiveFrames();

		void homePosition();
		void carouselMovePosition();

		float deg2rad(float degrees);

		void overShelve(int pot_value);

		void overPot(int pot_value);

  		void computeCommand(int pot_value);
  
 	 	void publishData();

};


#endif
