#include "gait_kinematics.hpp"

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
int callback_flag =0;
int last_gait_type = 0;
void callback(crab_gait::Dynamic_paraConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f", 
            config.clearance);
callback_flag = 1;
//para[0]=trapezoid_low_radius;
//para[1]=trapezoid_high_radius;
//para[2]=trapezoid_h;
//para[3]=clearance;
//para[4]=duration_ripple;
//para[5]=duration_tripod;
//para[6]=trapezoid_leg_radius;

}



bool GaitKinematics::init() {
	std::string robot_desc_string;
	// Get URDF XML
	if (!node.getParam("robot_description", robot_desc_string)) {
		   ROS_FATAL("Could not load the xml from parameter: robot_description");
		   return false;
	   }

	// Get Root and Tip From Parameter Server
	node.param("root_name_body", root_name, std::string("leg_center"));
	node.param("tip_name_body", tip_name, std::string("coxa"));

	// Get Gait Settings From Parameter Server
	node.param("trapezoid_low_radius", trap_low_r, 40.00);
	node.param("trapezoid_high_radius", trap_high_r, 30.00);
	node.param("trapezoid_h", trap_h, 30.00);
	node.param("clearance", trap_z, 120.00);
	node.param("duration_ripple", d_ripple, 10.5);
	node.param("duration_tripod", d_tripod, 10.0);
	node.param("trapezoid_leg_radius", trap_radius, 200.0);
	// Load and Read Models
	if (!loadModel(robot_desc_string)) {
		ROS_FATAL("Could not load models!");
		return false;
	}

	client = node.serviceClient<crab_msgs::GetLegIKSolver>("/crab_leg_kinematics/get_ik");
	joints_pub = node.advertise<crab_msgs::LegsJointsState>("joints_to_controller", 1);
	gait_control_sub = node.subscribe<crab_msgs::GaitCommand>("/teleop/gait_control", 1, &GaitKinematics::teleopGaitCtrl, this);
    ry_sub = node.subscribe<crab_msgs::sensorRY>("/imu/rp",1, &GaitKinematics::get_ry, this);
    run_time = 0;
	return true;
}

void GaitKinematics::update_para()

{
    node.getParam("gait_kinematics/root_name_body", root_name);
    node.getParam("gait_kinematics/tip_name_body", tip_name);

	// Get Gait Settings From Parameter Server
    node.getParam("gait_kinematics/trapezoid_low_radius", trap_low_r);
    node.getParam("gait_kinematics/trapezoid_high_radius", trap_high_r);
    node.getParam("gait_kinematics/trapezoid_h", trap_h);
    node.getParam("gait_kinematics/clearance", trap_z);
    node.getParam("gait_kinematics/duration_ripple", d_ripple);
    node.getParam("gait_kinematics/duration_tripod", d_tripod);
    node.getParam("gait_kinematics/trapezoid_leg_radius", trap_radius);

}
void GaitKinematics::gaitGenerator(){

	KDL::Vector* final_vector;
 	ros::Rate loop_rate(25);
	while (node.ok()){
        if (callback_flag == 1 )
        {
            gait.setTrapezoid(trap_low_r, trap_high_r, trap_h, trap_z);

            for (int i=0; i<num_legs; i++)
            {
                frames[i] = old_frames[i]* KDL::Frame (KDL::Vector (trap_radius,0,0));
            }
            callback_flag = 0;
            gait.Pause();
        }


		if (gait_command.cmd == gait_command.RUNRIPPLE){

            if(last_gait_type != 1)
            {
                gait.setGait(1);
                gait.Stop();

            }

			final_vector = gait.RunRipple(frames.begin(), gait_command.fi, gait_command.scale,
											gait_command.alpha, d_ripple);
			if (callService(final_vector)){
				joints_pub.publish(legs);
			}
            last_gait_type = 1;
		}



		else if (gait_command.cmd == gait_command.RUNTRIPOD){

            run_time = 0;

			final_vector = gait.RunTripod(frames.begin(), gait_command.fi, gait_command.scale,
											gait_command.alpha, d_tripod);


            //test_sec = ros::Time::now().toSec();
//            if(gait.passed_sec > d_tripod - 0.5)
//            {
//                for(int i=0;i<6;i++){

//                }

//                legs.joints_state[i].pwm[j]
//            }


			if (callService(final_vector)){
				joints_pub.publish(legs);
			}
           last_gait_type = 3;

		}



        else if (gait_command.cmd == gait_command.RUNTWAVE){
            if(last_gait_type != 2)
            {
                gait.setGait(2);
                gait.Stop();

            }

            final_vector = gait.RunWave(frames.begin(), gait_command.fi, gait_command.scale,
                                            gait_command.alpha, d_ripple);
            test_sec = ros::Time::now().toSec();
            if (callService(final_vector)){
                joints_pub.publish(legs);
            }
            last_gait_type = 2;

        }




        else if (gait_command.cmd == gait_command.TOINIT)
        {
            float i = 0;

            for( i=0;i<5;i=i+0.2)
            {
                gait.passed_sec = i;

                final_vector = gait.TOINIT(frames.begin(), 0, gait_command.scale,
                                                gait_command.fi, 5,2);
                //test_sec = ros::Time::now().toSec();
                if (callService(final_vector)){
                    joints_pub.publish(legs);
                }
                last_gait_type = 3;

                loop_rate.sleep();
            }

            gait_command.cmd = gait_command.STOP;
            gait.Stop();
            //run_time =1;

        }



		else if (gait_command.cmd == gait_command.PAUSE){
            gait.Pause();

		}



		else if (gait_command.cmd == gait_command.STOP){
           // run_time = 0;
            if (last_gait_type ==3&& run_time ==0)
            {
                    change_flag = gait.phase;
                    while(gait.passed_sec < d_tripod)
                    {
                        final_vector = gait.RunTripod(frames.begin(), gait_command.fi, gait_command.scale,
                                                        gait_command.alpha, d_tripod);
                        test_sec = ros::Time::now().toSec();
                        if (callService(final_vector)){
                            joints_pub.publish(legs);
                        }
                       last_gait_type = 3;

                       if(change_flag != gait.phase)
                       {
                           break;
                       }

                    }

                    float i = 0;

                    for( i=0;i<5;i=i+0.2)
                    {
                        gait.passed_sec = i;

                        final_vector = gait.TOINIT(frames.begin(), !gait.phase, gait_command.scale,
                                                        gait_command.fi, 5,1);
                        //test_sec = ros::Time::now().toSec();
                        if (callService(final_vector)){
                            joints_pub.publish(legs);
                        }
                        last_gait_type = 3;

                        loop_rate.sleep();
                    }

                    gait_command.cmd = gait_command.STOP;



                run_time = 1;
            }




                gait.Stop();
            }

		test_sec = ros::Time::now().toSec();

		ros::spinOnce();
		
		loop_rate.sleep();
		update_para();
		test_sec = ros::Time::now().toSec()-test_sec ;
        //ROS_INFO("use time2 %f" ,test_sec);
		
		
	}

}


void GaitKinematics::teleopGaitCtrl (const crab_msgs::GaitCommandConstPtr &gait_cmd){
	gait_command.cmd = gait_cmd->cmd;
	gait_command.fi = gait_cmd->fi;
	gait_command.alpha = gait_cmd->alpha;
    gait_command.scale = gait_cmd->scale;

}

void GaitKinematics::get_ry(const crab_msgs::sensorRYConstPtr &ry)
{
	float roll = ry->roll*3.14/180;
	float pitch = ry->pitch*3.14/180;
    gait.rotation = KDL::Rotation::RPY(-roll,-pitch,0);
}


bool GaitKinematics::callService (KDL::Vector* vector){
	crab_msgs::LegPositionState leg_pos_buf;
	srv.request.leg_number.clear();
	srv.request.target_position.clear();
	srv.request.current_joints.clear();

	//Creating message to request
	for (int i=0; i<num_legs; i++){
		srv.request.leg_number.push_back(i);
		leg_pos_buf.x = vector[i].x();
		leg_pos_buf.y = vector[i].y();
		leg_pos_buf.z = vector[i].z();
		srv.request.target_position.push_back(leg_pos_buf);
		srv.request.current_joints.push_back(legs.joints_state[i]);
	}
	//Call service and parsing response
	if (client.call(srv)){
		if (srv.response.error_codes==srv.response.IK_FOUND){
			for (int i=0; i<num_legs; i++){
				for (int j = 0; j < num_joints; j++) {
						legs.joints_state[i].joint[j] = srv.response.target_joints[i].joint[j];
                        			
				}
                legs.joints_state[i].pwm[0]=0;
                legs.joints_state[i].pwm[1]=100;
                legs.joints_state[i].pwm[2]=100;
//				ROS_DEBUG("Joints received leg%s\t1: %f\t2: %f\t3: %f", suffixes[i].c_str(),
//							legs.joints_state[i].joint[0],
//							legs.joints_state[i].joint[1],
//							legs.joints_state[i].joint[2]);
			}
		}
		else {
			ROS_ERROR("An IK solution could not be found");
			return 0;
		}
	}
	else {
		ROS_ERROR("Failed to call service");
		return 0;
	}
	return true;
}


bool GaitKinematics::loadModel(const std::string xml){
	//Construct tree with kdl_parser
	KDL::Tree tree;

	if (!kdl_parser::treeFromString(xml, tree)) {
		ROS_ERROR("Could not initialize tree object");
		return false;
	}
	ROS_INFO("Construct tree");

	//Get coxa and leg_center frames via segments (for calculating vectors)
	std::map<std::string,KDL::TreeElement>::const_iterator segments_iter;
	std::string link_name_result;
	for (int i=0; i<num_legs; i++){
		link_name_result = root_name + suffixes[i];
		segments_iter = tree.getSegment(link_name_result);
		frames.push_back((*segments_iter).second.segment.getFrameToTip());
	}
	for (int i=0; i<num_legs; i++){
		link_name_result = tip_name + suffixes[i];
		segments_iter = tree.getSegment(link_name_result);
		frames.push_back((*segments_iter).second.segment.getFrameToTip());
        old_frames.push_back((*segments_iter).second.segment.getFrameToTip());
	}
	ROS_INFO("Get frames");
    // get the first frame
	//Vector iterators
    // the tip frame coresponing to the first frame is KDL::Vector (trap_radius,0,0); * KDL::Frame (KDL::Vector (trap_radius,0,0))
	for (int i=0; i<num_legs; i++){
        old_frames[i] = frames[i] * frames[i+num_legs];  //!!!!!!!!!!!!!!!!! xiugai,yinggai wei radius
	}


    //init_frames.resize(num_legs);
// get the tip
	for (int i=0; i<num_legs; i++){
		for (int j = 0; j < num_joints; j++) {
			legs.joints_state[i].joint[j] = 0;
		}
	}

	return true;
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "gait_kinematics");
	GaitKinematics g;
	dynamic_reconfigure::Server<crab_gait::Dynamic_paraConfig> server;
  	dynamic_reconfigure::Server<crab_gait::Dynamic_paraConfig>::CallbackType f;
	//void callback(crab_gait::Dynamic_paraConfig &config, uint32_t level);
	f = boost::bind(&callback, _1, _2);

  	server.setCallback(f);
    if (g.init()<0) {
        ROS_ERROR("Could not initialize gait node");
        return -1;
    }
    g.gaitGenerator();

    ros::spin();
    return 0;
}
