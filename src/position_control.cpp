#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "Eigen/Dense"

#define P_pos 0.5f
#define D_pos 0.01f

using namespace Eigen;

void odometryCallback(const nav_msgs::Odometry &odometry);
void positionsetpointCallback(const geometry_msgs::PoseStamped &position_setpoint);


bool takeoff = false;
bool reset_pos = false;

std_msgs::Empty order;
geometry_msgs::Twist cmd;

Vector3f pos(0.0,0.0,0.0);
Vector3f pos_pre(0.0,0.0,0.0);
Vector3f pos_sp(0.0,0.0,0.0);
Vector3f error(0.0,0.0,0.0);
Vector3f error_d(0.0,0.0,0.0);
Vector3f vel_sp(0.0,0.0,0.0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_control");
	ros::NodeHandle n;
	ros::Subscriber pos_sub = n.subscribe("/ardrone/odometry", 10, odometryCallback);
	ros::Subscriber pos_sp_sub = n.subscribe("/position_setpoint", 10, positionsetpointCallback);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 10);
	ros::Publisher stop_pub = n.advertise<std_msgs::Empty>("/ardrone/reset", 10);
	ros::Rate loop_rate(20);

	cmd.linear.x = 0.0;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = 0.0;
	while(ros::ok())
	{
		if(reset_pos)
		{
			error = pos_sp - pos;
			error_d = pos - pos_pre;
			vel_sp(0) = error(0) * P_pos + error_d(0) * D_pos;
			vel_sp(1) = error(1) * P_pos + error_d(1) * D_pos;
			cmd.linear.x = vel_sp(0);
			cmd.linear.y = vel_sp(1);
			if(cmd.linear.x > 0.2) cmd.linear.x = 0.2;
			if(cmd.linear.x < -0.2) cmd.linear.x = -0.2;
			if(cmd.linear.y > 0.2) cmd.linear.y = 0.2;
			if(cmd.linear.y < -0.2) cmd.linear.y = -0.2;
		}
			
		cmd_pub.publish(cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	cmd.linear.x = 0.0;
	cmd.linear.y = 0.0;
	cmd_pub.publish(cmd);

	return 0;
}

void odometryCallback(const nav_msgs::Odometry &odometry)
{
	pos_pre(0) = pos(0);
	pos_pre(1) = pos(1);
	pos_pre(2) = pos(2);
	pos(0) = odometry.pose.pose.position.x;
	pos(1) = odometry.pose.pose.position.y;
	pos(2) = odometry.pose.pose.position.z;
	if(!reset_pos && pos(2) > 0.3)
	{
		pos_sp = pos;
		reset_pos = true;
	}
}

void positionsetpointCallback(const geometry_msgs::PoseStamped &position_setpoint)
{
	pos_sp(0) = position_setpoint.pose.position.x;
	pos_sp(1) = position_setpoint.pose.position.y;
	pos_sp(2) = position_setpoint.pose.position.z;
}
