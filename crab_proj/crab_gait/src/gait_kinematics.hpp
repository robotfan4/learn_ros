

#ifndef GAIT_TRIPOD_HPP_
#define GAIT_TRIPOD_HPP_

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
//#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <crab_msgs/GetLegIKSolver.h>
#include <crab_msgs/LegsJointsState.h>
#include <crab_msgs/GaitCommand.h>
#include "gait.hpp"
#include <dynamic_reconfigure/server.h>
#include <crab_gait/Dynamic_paraConfig.h>
#include<crab_msgs/sensorRY.h>
#define NUM_LEGS 6
#define NUM_JOINTS 3

class GaitKinematics {
	public:
		GaitKinematics();
		bool init();
		void gaitGenerator();
		void update_para();
        int run_time;
	private:
		ros::NodeHandle node;
		std::string root_name, tip_name;
		std::vector<KDL::Frame> frames;
        std::vector<KDL::Frame> old_frames;
        volatile int change_flag;
		double fi;
		crab_msgs::LegsJointsState legs;
		crab_msgs::GetLegIKSolver srv;
		crab_msgs::GaitCommand gait_command;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;
		double trap_low_r, trap_high_r, trap_h, trap_z, trap_radius;
		double d_ripple, d_tripod;
		double test_sec;
		ros::ServiceClient client;
		ros::Publisher joints_pub;
		ros::Subscriber gait_control_sub;
        ros::Subscriber ry_sub;

		bool loadModel(const std::string xml);
		bool callService (KDL::Vector* vector);
		void teleopGaitCtrl (const crab_msgs::GaitCommandConstPtr &gait_cmd);
        void get_ry(const crab_msgs::sensorRYConstPtr &ry);
        Gait gait;

};

GaitKinematics::GaitKinematics(){}



#endif /* GAIT_TRIPOD_HPP_ */
