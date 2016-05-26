#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "Eigen/Dense"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_control/ROI.h"
// #include <iostream>
// #include <conio.h>
// #include <stdio.h>
// #include <stdlib.h>
#define ESC 0x1b
#define LOOP_RATE 20
using namespace std;
using namespace Eigen;
std_msgs::Empty order;
bool first_pos_received = false;
bool first_yaw_received = false;

int constrain(int a, int b, int c){return ((a)<(b)?(b):(a)>(c)?c:a);}
int dead_zone(int a, int b){return ((a)>(b)?(a):(a)<(-b)?(a):0);}
float constrain_f(float a, float b, float c){return ((a)<(b)?(b):(a)>(c)?c:a);}
float dead_zone_f(float a, float b){return ((a)>(b)?(a):(a)<(-b)?(a):0);}
int minimum(int a, int b){return (a>b?b:a);}
int maximum(int a, int b){return (a>b?a:b);}
int absolute(int a){return (a>0?a:-a);}
float absolute_f(float a){return (a>0?a:-a);}
class num_flight
{
	#define STATE_IDLE 0
	#define STATE_INAC 1
	#define STATE_ACCR 2
	#define STATE_ALT 3
public:
	unsigned char _state;//idle, or inaccurate pos_ctrl, or accurate image_ctrl
	bool _num_reached[9];
	Matrix<float, Dynamic, 2> _landing_pos;
	unsigned char _current_target;
	num_flight();
	~num_flight();
	bool is_all_num_finished();
	bool idle_control		(Vector3f& vel_sp);
	bool inaccurate_control	(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp);
	bool accurate_control	(const Vector3f& _image_pos, float pos_z, Vector3f& vel_sp);
	bool altitude_change	(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp);

//	void general_control();
private:
	
};
class States
{
public:
	Matrix<float, 3, 3> R_inert;
	Matrix<float, 3, 3> R_body;
	
	Vector3f acc_b;
	Vector3f vel_b;
	Vector3f pos_b;
	Vector3f pos_i;

	ros::Time navdata_stamp;

	float init_yaw;
	float yaw;

	void R_init(float yaw);
	void get_R_inert(float yaw);
	void get_R_body(float yaw);
	States();
	~States();
private:
	ros::NodeHandle n;
	ros::Subscriber states_sub;
	ros::Subscriber nav_sub;
	//void odometryCallback(const nav_msgs::Odometry &odometry);
	void navCallback(const ardrone_autonomy::Navdata &msg);
};
num_flight::num_flight()
{
	_state = STATE_IDLE;
	_current_target=0;
	for(int i = 0; i < 9; i++)
		_num_reached[i] = false;
	_landing_pos.resize(9,2);
	_landing_pos <<
	0.0, 0.0,
	2.0, 0.0,
	0.0, 0.0,
	2.0, 0.0,
	0.0, 0.0,
	2.0, 0.0,
	0.0, 0.0,
	2.0, 0.0,
	0.0, 0.0;
}

num_flight::~num_flight()
{
}

bool num_flight::is_all_num_finished(void)
{
	for(int i = 0; i < 9; i++){
		if(_num_reached[i] == false){//if not reached
			return false;
		}
	}
	return true;
}

bool num_flight::idle_control(Vector3f& vel_sp)
{
	for(int i = 0; i < 3; i++)
		vel_sp(i) = 0;
	return false;
}

bool num_flight::inaccurate_control(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp)
{
	float speed = 0.1;
	bool is_arrived;
	Vector3f err = _pos_sp - _pos;
	err(2) = 0;
	float dist = err.norm();
	if(dist > 0.8){
		Vector3f direction = err / dist;
		vel_sp = direction * speed;

		is_arrived = false;
		ROS_INFO("\npos(%f, %f)\nsetpt(%f, %f)\nvelsp(%f, %f)\n",_pos(0),_pos(1),_pos_sp(0),_pos_sp(1),vel_sp(0),vel_sp(1));
	}
	else{
		is_arrived = true;
	}
	return is_arrived;
}

bool num_flight::accurate_control(const Vector3f& image_pos, float pos_z, Vector3f& vel_sp)
{
	// Vector3f z_body(0,0,1);
	// Vector2f pos_corr_m;
	// float chord_len = pos_z / z_body(2);
	// pos_corr_m(0) = chord_len * z_body(0);
	// pos_corr_m(1) = chord_len * z_body(1);
	static Vector2f err_last;
	static Vector2f err_int;
	static bool new_start = true;
	float P_pos = 0.3, D_pos = 0.02, I_pos = 0.05;
	Vector2f vel_sp_2d;
	Vector2f image_center(320.0,180.0);
	Vector2f image_pos_2d;
	image_pos_2d(0) = image_pos(0);
	image_pos_2d(1) = image_pos(1);
	Vector2f err = image_pos_2d - image_center;
	if(new_start){
		err_last = err;
		err_int(0) = 0;
		err_int(1) = 0;
		new_start = false;
	}
	Vector2f err_d = (err - err_last) * LOOP_RATE;
	vel_sp_2d = err * P_pos + err_d * D_pos + err_int * I_pos;
	
	vel_sp(0) = -vel_sp_2d(1);
	vel_sp(1) = -vel_sp_2d(0);
	vel_sp(2) = 0;
	bool is_arrived;
	float dist = err.norm();
	if(dist > 15.0){
		is_arrived = false;
	}
	else{
		is_arrived = true;
	}
	err_last = err;
	err_int += err / LOOP_RATE;
	return is_arrived;
}

bool num_flight::altitude_change(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp)
{
	float speed = 0.2;
	bool is_arrived;
	float err = _pos_sp(2) - _pos(2);
	float dist = absolute_f(err);
	if(dist > 0.1){
		float direction = err / dist;
		vel_sp(2) = direction * speed;
		is_arrived = false;
	}
	else{
		is_arrived = true;
	}
	vel_sp(0) = 0;
	vel_sp(1) = 0;
	return is_arrived;
}


States::States()
{
	//states_sub = n.subscribe("/ardrone/odometry", 1, &States::odometryCallback, this);
	nav_sub = n.subscribe("/ardrone/navdata", 1, &States::navCallback,this);

}
States::~States()
{
}
void States::get_R_inert(float yaw)
{
	R_inert(0,0) = cos(yaw);
	R_inert(0,1) = sin(yaw);
	R_inert(0,1) = 0;
	R_inert(1,0) = sin(-yaw);
	R_inert(1,1) = cos(yaw);
	R_inert(1,2) = 0;
	R_inert(2,0) = 0;
	R_inert(2,1) = 0;
	R_inert(2,2) = 0;
}
void States::R_init(float yaw)
{
	init_yaw = -85/57.3;
	get_R_inert(init_yaw);
}
void States::get_R_body(float yaw)
{
	float dif_yaw = yaw - init_yaw;
	R_body(0,0) = cos(dif_yaw);
	R_body(0,1) = sin(-dif_yaw);
	R_body(0,1) = 0;
	R_body(1,0) = sin(dif_yaw);
	R_body(1,1) = cos(dif_yaw);
	R_body(1,2) = 0;
	R_body(2,0) = 0;
	R_body(2,1) = 0;
	R_body(2,2) = 0;
}

// #Two-integer timestamp that is expressed as:
// # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
// # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')

// void States::odometryCallback(const nav_msgs::Odometry &odometry)
// {

// 	pos(0) = odometry.pose.pose.position.x;
// 	pos(1) = -odometry.pose.pose.position.y;
// 	pos(2) = odometry.pose.pose.position.z;
// 	navdata_stamp = odometry.header.stamp;
// 	navdata_stamp = ros::Time::now();
// 	first_pos_received = true;
// }
void States::navCallback(const ardrone_autonomy::Navdata &msg)
{
	vel_b(0) = msg.vx;
	vel_b(1) = msg.vy;
	vel_b(2) = msg.vz;
	acc_b(0) = msg.ax;
	acc_b(1) = msg.ay;
	acc_b(2) = msg.az;
	navdata_stamp = msg.header.stamp;
	yaw = (msg.rotZ+80) / 57.3;
	first_yaw_received = true;
//	ROS_INFO("yaw_deg: %f",yaw*57.3);
}

Vector3f pos;
Vector3f image_pos;
Vector3f image_pos_pre;

ros::Time image_stamp;

// void positionsetpointCallback(const geometry_msgs::PoseStamped &position_setpoint)
// {
// 	pos_sp(0) = position_setpoint.pose.position.x + pos(0);
// 	pos_sp(1) = position_setpoint.pose.position.y + pos(1);
// 	pos_sp(2) = position_setpoint.pose.position.z + pos(2);

// 	//the msg provides relative position between one number and the next
// }

void imagepositionCallback(const ardrone_control::ROI &msg)
{
	image_pos_pre(0) = image_pos(0);
	image_pos_pre(1) = image_pos(1);
	image_pos(0) = msg.pose1.x;
	image_pos(1) = msg.pose1.y;
//	image_stamp = msg.header.stamp;
	image_stamp = ros::Time::now();
}

int main(int argc, char **argv)
{
	int keypress = 0;
	num_flight flight;
	States state;
	ros::init(argc, argv, "position_control2");
	ros::NodeHandle n;
//	ros::Subscriber pos_sub = n.subscribe("/ardrone/odometry", 1, odometryCallback);
	ros::Subscriber image_pos_sub = n.subscribe("/ROI", 1, imagepositionCallback);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
	ros::Publisher stop_pub = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);
	ros::Publisher pose_body_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/position_body", 1);
	ros::Publisher pose_world_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/position_world", 1);
	ros::Rate loop_rate(LOOP_RATE);
	bool isArrived = false;
	Vector3f vel_sp(0.0, 0.0, 0.0);
	Vector3f next_pos_sp(0.0,0.0,0.0);
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = 0.0;

	flight._state = STATE_INAC;
	next_pos_sp(2) = pos(2);
	ROS_INFO("started");
	while(ros::ok() && !first_pos_received){
		ros::spinOnce();
		loop_rate.sleep();
	}
	while(ros::ok() && !first_yaw_received){
		ros::spinOnce();
		loop_rate.sleep();
	}
	for(int i=0;i<9;i++){
		flight._landing_pos(i,0) += pos(0);
		flight._landing_pos(i,1) += pos(1);

	}
	state.R_init(state.yaw);
	while(ros::ok() && !flight.is_all_num_finished())
	{
	//	keypress = kbhit();
		// if(kbhit())
		// 	break;
		
		next_pos_sp(0) = flight._landing_pos(flight._current_target,0);
		next_pos_sp(1) = flight._landing_pos(flight._current_target,1);
		ros::Duration dur;
		switch(flight._state){
			case STATE_IDLE:
				isArrived = flight.idle_control(vel_sp);
				
				break;
			case STATE_INAC:
				isArrived = flight.inaccurate_control(next_pos_sp, pos, vel_sp);
				
				dur = ros::Time::now() - state.navdata_stamp;
				if(dur.toSec() > 0.5){
					vel_sp(0) = 0;
					vel_sp(1) = 0;
					vel_sp(2) = 0;
				}
				break;
			case STATE_ACCR:
				isArrived = flight.accurate_control(image_pos, pos(2), vel_sp);
				
				dur = ros::Time::now() - image_stamp;
				if(dur.toSec() > 0.5){
					vel_sp(0) = 0;
					vel_sp(1) = 0;
					vel_sp(2) = 0;
				}
				break;
			case STATE_ALT:
				isArrived = flight.altitude_change(next_pos_sp, pos, vel_sp);
				break;
		}
		if(isArrived){
			if(flight._state == STATE_INAC){
				flight._num_reached[flight._current_target] = true;
				flight._current_target++;
				vel_sp(0)=0;
				vel_sp(1)=0;
				vel_sp(2)=0;
				if(flight._current_target>=9){
					flight._state = STATE_IDLE;
				}
				else{
					flight._state = STATE_INAC;
					ROS_INFO("Target %d arrived\nState transferred from ACCURATE to INACCURATE\n", flight._current_target - 1);
				}
			//	flight._state = STATE_ACCR;
			//	ROS_INFO("State transferred from INACCURATE to ACCURATE\n");
			}
			else if(flight._state == STATE_ACCR){
				flight._num_reached[flight._current_target] = true;
				flight._current_target++;
				if(flight._current_target>=9){
					flight._state = STATE_IDLE;
				}
				else{
					flight._state = STATE_INAC;
					ROS_INFO("Target %d arrived\nState transferred from ACCURATE to INACCURATE\n", flight._current_target - 1);
				}
			}
			else if(flight._state == STATE_ALT){
				flight._state = STATE_INAC;
				ROS_INFO("New altitude arrived\nState transferred from ALT to INACCURATE\n");
			}
		}
		
		Vector3f output_vel_sp;
		state.get_R_body(state.yaw);
		output_vel_sp = state.R_body.transpose() * vel_sp;
		cmd.linear.x = output_vel_sp(0);
		cmd.linear.y = output_vel_sp(1);
		cmd.linear.z = vel_sp(2);
		cmd_pub.publish(cmd);
		ros::spinOnce();
		loop_rate.sleep();
		

	}

	
	return 0;
}