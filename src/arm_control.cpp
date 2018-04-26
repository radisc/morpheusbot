#include <urdf/model.h>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/kdl.hpp>
#include <kdl/solveri.hpp>
#include <kdl/joint.hpp>

// TF
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1
#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

KDL::Frame F_start, F_dest;

using namespace std;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){

	float delta_x = msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
	float delta_y = msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
	float delta_z = msg->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];

	F_dest.p.data[0] += delta_x/500;
	F_dest.p.data[1] += delta_y/500;
	F_dest.p.data[2] += delta_z/500;

}

int main(int argc, char** argv){

	ros::init(argc, argv, "arm_control");

	ros::NodeHandle nh;

	tf::TransformBroadcaster br;

	if (argc != 2){
		ROS_ERROR("Need a urdf file as argument");
		return -1;
	}

	std::string urdf_file = argv[1];
	urdf::Model model;
	KDL::Tree my_tree;
	KDL::Chain chain;

	if (!model.initFile(urdf_file)){

		ROS_ERROR("Failed to parse urdf file");
   		return -1;
	}

	if(!kdl_parser::treeFromUrdfModel(model, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	} else {

		if(!my_tree.getChain("base_link", "hand", chain)){
			ROS_ERROR("Failed to construct kdl chain");
			return false;
		} else {

		}

	}

	ROS_INFO("Successfully parsed urdf file");

	ros::Rate rate(30);

	ros::Subscriber joy_sub 	=	nh.subscribe("/joy", 5, joyCallback);
	ros::Publisher	joint_pub	=	nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

	F_start = KDL::Frame(KDL::Vector(0.7, 0 , 0.1));
	F_dest = KDL::Frame(KDL::Vector(F_start.p[0],
									F_start.p[1],
									F_start.p[2]));

	//Resolve IK
	KDL::ChainFkSolverPos_recursive fksolver1(chain);
	KDL::ChainIkSolverVel_pinv iksolver1v(chain);
	KDL::ChainIkSolverPos_NR iksolver1(chain, fksolver1, iksolver1v, 100, 1e-6);

	KDL::JntArray q			(chain.getNrOfJoints());
	KDL::JntArray q_init	(chain.getNrOfJoints());

	while(ros::ok()){

		tf::StampedTransform tf_dest;
		tf_dest.child_frame_id_ = "arm_goal";
		tf_dest.frame_id_ = "base_link";
		tf_dest.stamp_ = ros::Time::now();
		tf_dest.setOrigin(tf::Vector3(	F_dest.p[0],
										F_dest.p[1],
										F_dest.p[2]));
		tf::Quaternion quat;
		quat.setRPY(0,0,0);
		tf_dest.setRotation(quat);
		//tf_dest.setRotation()

		br.sendTransform( tf_dest );





		int ret = iksolver1.CartToJnt(q_init, F_dest, q);

		sensor_msgs::JointState jnt_msg;
		jnt_msg.header.frame_id = "";
		jnt_msg.header.stamp = ros::Time::now();

		//std::cout << "Joints: " << q.data << "\n";

		jnt_msg.name.push_back("joint1");
		jnt_msg.name.push_back("joint2");
		jnt_msg.name.push_back("joint3");
		jnt_msg.name.push_back("joint4");
		jnt_msg.name.push_back("joint5");


		jnt_msg.position.push_back(q.data[0]);
		jnt_msg.position.push_back(q.data[1]);
		jnt_msg.position.push_back(q.data[2]);
		jnt_msg.position.push_back(q.data[3]);
		jnt_msg.position.push_back(q.data[4]);

//		jnt_msg.position.data() = &q.data[0];

//		jnt_msg.position.data()[0] = q.data[0];
//		jnt_msg.position.data()[1] = q.data[1];
//		jnt_msg.position.data()[2] = q.data[2];
//		jnt_msg.position.data()[3] = q.data[3];


		//cout << "Publishing.." << endl;

		joint_pub.publish(jnt_msg);




		ros::spinOnce();
		rate.sleep();
		}

	return 0;
}
