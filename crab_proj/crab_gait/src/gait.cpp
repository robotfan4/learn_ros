#include "gait.hpp"

Gait::Gait(){
    ////////////////////3 0 2 4 1 5 ///////////////
//    legs_queue.push(3);
//    legs_queue.push(2);
//    legs_queue.push(1);
//    legs_queue.push(4);
//    legs_queue.push(5);
//    legs_queue.push(0);
    state = 0;
    phase = 0;

}


void Gait::setTrapezoid(double low_r, double high_r, double h, double z){
	low_rad = low_r;
	high_rad = high_r;
	height = z - h;
	z_body = z;
}

void Gait::setFi (double fi){
	//Set vectors like x = r * cos(fi), y = r * sin(fi) in 3d space
	//     B--high_rad---C
	//    /               \  <- h
	//   A-----low_rad-----D <- z

	a.p = KDL::Vector (-low_rad * cos(fi), -low_rad * sin(fi), -z_body);
	b.p = KDL::Vector (-high_rad * cos(fi), -high_rad * sin(fi), -height);
	c.p = KDL::Vector (high_rad * cos(fi), high_rad * sin(fi), -height);
	d.p = KDL::Vector (low_rad * cos(fi), low_rad * sin(fi), -z_body);
}

void Gait::setAlpha (double alpha){
	a.M = KDL::Rotation::RotZ(-alpha);
	b.M = KDL::Rotation::RotZ(-alpha/2);
	c.M = KDL::Rotation::RotZ(alpha/2);
	d.M = KDL::Rotation::RotZ(alpha);
}

void Gait::setPath (){
	path_support = new KDL::Path_Line(d, a, &rot, 0.5, true);

	path_transfer = new KDL::Path_RoundedComposite (20.00,0.5,&rot);
	path_transfer -> Add(a);
	path_transfer -> Add(b);
	path_transfer -> Add(c);
	path_transfer -> Add(d);
	path_transfer -> Finish();
}

void Gait::setTrajectory (double sup_path_duration, double tran_path_duration){
	prof_support.SetProfileDuration(0,path_support->PathLength(), sup_path_duration);
	prof_transfer.SetProfileDuration(0,path_transfer->PathLength(), tran_path_duration);
	trajectory_transfer = new KDL::Trajectory_Segment (path_transfer, &prof_transfer);
	trajectory_support = new KDL::Trajectory_Segment (path_support, &prof_support);
}

KDL::Vector* Gait::RunTripod (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration, duration);
	KDL::Frame frame;
	if (run_state == false){
		run_state = true;
		phase = 0;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}
	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	if (passed_sec >= duration-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
		phase = !phase;
        state = !state;
	}

//    double my_time = trajectory_transfer->Duration();
//	ROS_INFO("begin seconds %f" ,passed_sec);
	for (int i=phase; i<num_legs; i+=2){
		frame = trajectory_transfer -> Pos(passed_sec);
		frame.p.x(frame.p.data[0]*scale);
		frame.p.y(frame.p.data[1]*scale);
        final_vector[i] = rotation*(frame.M *(*(vector_iter+i)).p + frame.p);
	}
	for (int i=!phase; i<num_legs; i+=2){
		frame = trajectory_support -> Pos(passed_sec);
		frame.p.x(frame.p.data[0]*scale);
		frame.p.y(frame.p.data[1]*scale);
        final_vector[i] = rotation*(frame.M *(*(vector_iter+i)).p + frame.p);
	}


	return final_vector;
}

KDL::Vector* Gait::RunRipple (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
    /////////////// set the para of leg tip trajectory ///////////////////////////////
    double phase_offset = duration / num_legs;
	setFi(fi);
	setAlpha(alpha);
	setPath();
    //setGait(2);
    ////////// 2/3 support 1/3 transfer ///////////
    setTrajectory(duration*2/3, duration/3);
	KDL::Frame frame;

	if (run_state == false){
		run_state = true;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}

	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

    ///////////////////////////////////////
    getTipVector(trajectory_transfer,	phase_offset,	vector_iter, scale);
    getTipVector(trajectory_transfer,	0,				vector_iter, scale);
    getTipVector(trajectory_support,	3*phase_offset,	vector_iter, scale);
    getTipVector(trajectory_support,	2*phase_offset,	vector_iter, scale);
    getTipVector(trajectory_support,	phase_offset,	vector_iter, scale);
    getTipVector(trajectory_support,	0,				vector_iter, scale);

//    getTipVector(trajectory_transfer,	0,	vector_iter, scale);
//    getTipVector(trajectory_support,	4*phase_offset, vector_iter, scale);
//    getTipVector(trajectory_support,	3*phase_offset,	vector_iter, scale);
//    getTipVector(trajectory_support,	2*phase_offset,	vector_iter, scale);
//    getTipVector(trajectory_support,	phase_offset,	vector_iter, scale);
//    getTipVector(trajectory_support,	0,				vector_iter, scale);

    if (passed_sec >= phase_offset-0.1 && run_state==true){
        begin_sec = ros::Time::now().toSec();
        passed_sec = 0;

        legs_queue.push(legs_queue.front());
        legs_queue.pop();
    }
	return final_vector;
}


KDL::Vector* Gait::RunWave (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
    /////////////// set the para of leg tip trajectory ///////////////////////////////
    double phase_offset = duration / num_legs;
    setFi(fi);
    setAlpha(alpha);
    setPath();
   // setGait(2); //// 2 for wave gait
    ////////// 5/6 support 1/6 transfer ///////////
    setTrajectory(duration*5/6, duration/6);
    KDL::Frame frame;

    if (run_state == false){
        run_state = true;
        begin_sec = ros::Time::now().toSec();
        passed_sec = 0;
    }

    if (pause_state == true){
        begin_sec = ros::Time::now().toSec() - passed_sec;
        pause_state = false;
    }


    passed_sec = ros::Time::now().toSec() - begin_sec;

    if (passed_sec >= phase_offset-0.1 && run_state==true){
        begin_sec = ros::Time::now().toSec();
        passed_sec = 0;

        legs_queue.push(legs_queue.front());
        legs_queue.pop();
    }
    ///////////////////////////////////////
//    getTipVector(trajectory_transfer,	phacse_offset,	vector_iter, scale);
//    getTipVector(trajectory_transfer,	0,				vector_iter, scale);
//    getTipVector(trajectory_support,	3*phase_offset,	vector_iter, scale);
//    getTipVector(trajectory_support,	2*phase_offset,	vector_iter, scale);
//    getTipVector(trajectory_support,	phase_offset,	vector_iter, scale);
//    getTipVector(trajectory_support,	0,				vector_iter, scale);

    getTipVector(trajectory_transfer,	0,	vector_iter, scale);
    getTipVector(trajectory_support,	4*phase_offset, vector_iter, scale);
    getTipVector(trajectory_support,	3*phase_offset,	vector_iter, scale);
    getTipVector(trajectory_support,	2*phase_offset,	vector_iter, scale);
    getTipVector(trajectory_support,	phase_offset,	vector_iter, scale);
    getTipVector(trajectory_support,	0,				vector_iter, scale);


    return final_vector;
    ROS_INFO(" passed time %f" ,passed_sec);
}


KDL::Vector *Gait::TOINIT(std::vector<KDL::Frame>::const_iterator vector_iter, int phase_state , double scale, double alpha, double duration, int i)
{

    init_for_3_3(i, duration, alpha);

    KDL::Frame frame;
    //double my_time = trajectory_transfer->Duration();
    //passed_sec = ros::Time::now().toSec() - begin_sec;

    ROS_INFO("begin seconds %f" ,passed_sec);
    if(passed_sec< duration){

        for (int i=!phase_state; i<num_legs; i=i+2){
            frame = trajectory_back_init-> Pos(passed_sec);
            frame.p.x(frame.p.data[0]*scale);
            frame.p.y(frame.p.data[1]*scale);
            final_vector[i] = rotation*(frame.M *(*(vector_iter+i)).p + frame.p);
        }

        for (int i=phase_state; i<num_legs; i=i+2){
            frame = trajectory_foward_init-> Pos(passed_sec);
            frame.p.x(frame.p.data[0]*scale);
            frame.p.y(frame.p.data[1]*scale);
            final_vector[i] = rotation*(frame.M *(*(vector_iter+i)).p + frame.p);
        }

    }



    return final_vector;


}
////////////////////
void Gait::getTipVector (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
	frame = trajectory -> Pos(passed_sec + phase_offset);
	frame.p.x(frame.p.data[0]*scale);
	frame.p.y(frame.p.data[1]*scale);
    final_vector[legs_queue.front()] = rotation*(frame.M * (*(vector_iter + legs_queue.front())).p + frame.p);
	legs_queue.push(legs_queue.front());
	legs_queue.pop();
}


void Gait::Pause (){
	pause_state = true;
}

void Gait::Stop (){
    run_state = false;
}

void Gait::setGait(int i)
{
    while(!legs_queue.empty()){
        legs_queue.pop();
    }
    ////////////////// this is for the ripple gait //////
    ////////////3 0 2 4 1 5 ///////////////////////
    if(i == 1)
    {
        legs_queue.push(3);
        legs_queue.push(0);
        legs_queue.push(2);
        legs_queue.push(4);
        legs_queue.push(1);
        legs_queue.push(5);
    }
    /////////////// this is for wave gait ///////////
    if(i == 2)
    {
        legs_queue.push(3);
        legs_queue.push(2);
        legs_queue.push(1);
        legs_queue.push(4);
        legs_queue.push(5);
        legs_queue.push(0);
    }


}

void Gait::init_for_3_3(int i,double duration,double fi)
{
    a.p = KDL::Vector (0, 0, -z_body);
    b.p = KDL::Vector (0 , 0, -z_body+20);
    c.p = KDL::Vector (-low_rad * cos(fi), -low_rad * sin(fi), -z_body+30);
    d.p = KDL::Vector (-low_rad * cos(fi), -low_rad * sin(fi), -z_body);


    e.p = KDL::Vector (0, 0, -z_body+20);
    f.p = KDL::Vector (low_rad * cos(fi), low_rad * sin(fi), -z_body+30);
    g.p = KDL::Vector (low_rad * cos(fi), low_rad * sin(fi), -z_body);
    h.p = KDL::Vector (0, 0, -80);

    /*
    a.p = KDL::Vector (-low_rad * cos(fi), -low_rad * sin(fi), -z_body);
    b.p = KDL::Vector (-high_rad * cos(fi), -high_rad * sin(fi), -height);
    c.p = KDL::Vector (high_rad * cos(fi), high_rad * sin(fi), -height);
    d.p = KDL::Vector (low_rad * cos(fi), low_rad * sin(fi), -z_body);
*/



    if (i ==1)
    {
        path_init_back = new KDL::Path_RoundedComposite (10.00,0.5,&rot);
        path_init_back -> Add(d);
        path_init_back -> Add(c);
        path_init_back -> Add(b);
        path_init_back -> Add(a);
        path_init_back -> Finish();

        path_init_foward = new KDL::Path_Line(g, a, &rot, 0.5, true);
//        path_init_foward = new KDL::Path_RoundedComposite (10.00,0.5,&rot);
//        path_init_foward -> Add(a);
//        path_init_foward -> Add(e);
//        path_init_foward -> Add(f);
//        path_init_foward -> Add(g);
//        path_init_foward -> Finish();
    }
    else if(i ==2)
    {
        path_init_back = new KDL::Path_RoundedComposite (10.00,0.5,&rot);
        path_init_back -> Add(a);
        path_init_back -> Add(e);
        path_init_back -> Add(f);
        path_init_back -> Add(g);
        path_init_back -> Finish();

        path_init_foward = new KDL::Path_Line(a, d, &rot, 0.5, true);

    }

    prof_init_back.SetProfileDuration(0,path_init_back->PathLength(),duration);
    prof_init_foward.SetProfileDuration(0,path_init_foward->PathLength(),duration);

    trajectory_back_init = new KDL::Trajectory_Segment (path_init_back, &prof_init_back);
    trajectory_foward_init = new KDL::Trajectory_Segment (path_init_foward, &prof_init_foward);
}
