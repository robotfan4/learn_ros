#include "ros/ros.h"
#include "crab_msgs/LegsJointsState.h"
#include <sensor_msgs/JointState.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float32.h>
//#include"crab_msgs/le"
typedef boost::shared_ptr<crab_msgs::LegsJointsState const> LegsStateConstPtr;
typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;
typedef boost::shared_ptr<std_msgs::Int32 const> useflagConstPtr;
typedef boost::shared_ptr<std_msgs::Float32 const> jonitangleConstPtr;

float joint[4];
//const std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const static std::string suffixes[6] = {"_RF", "_RM", "_RR", "_LF", "_LM", "_LR"};
const std::string names[3] = {"coxa_joint", "femur_joint", "tibia_joint"};
ros::Publisher joint_msg_pub;
sensor_msgs::JointState joint_msg;
int flag =0;
void chatterLegsState (const LegsStateConstPtr& state){
	std::string joint_name;
	for (int name=0; name<3; name++){
		for(int suf=0; suf<6; suf++){
			joint_name = names[name] + suffixes[suf];
			joint_msg.name.push_back(joint_name.c_str());
			joint_msg.position.push_back(state->joints_state[suf].joint[name]);
            joint_msg.effort.push_back(state->joints_state[suf].pwm[name]);
		}
	}

    if(flag==1)
    {
        joint_msg.position[0]=joint[0];
        joint_msg.position[6]=joint[1];
        joint_msg.position[12]=joint[2];
    }
    if(flag== 2)
    {
        joint_msg.position[3]=joint[3];
    }


	joint_msg_pub.publish(joint_msg);
	joint_msg.name.clear();
	joint_msg.position.clear();
        joint_msg.effort.clear();
}
void addcallback(const JointStateConstPtr& state){
    for(int i=0;i<3;i++)
    {
        joint[i] = state->position[i];
    }
}

void flagcallback(const useflagConstPtr& state){

    flag = state->data;
}
void wheelcallback(const jonitangleConstPtr& state){

    joint[3] = state->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "crab_joint_publisher");
	ros::NodeHandle node;

        joint_msg_pub = node.advertise<sensor_msgs::JointState>("/crab_joint_publisher", 1);
	ros::Rate loop_rate(20);

        ros::Subscriber sub = node.subscribe("/joints_to_controller", 1, chatterLegsState);
    ros::Subscriber part_sub = node.subscribe("leg_joints_states",1,addcallback);
    ros::Subscriber flag_sub = node.subscribe("use_marker_flag",1,flagcallback);
    ros::Subscriber joint_sub = node.subscribe("/wheel_angle",1,wheelcallback);
	ros::spin();

	return 0;
}
