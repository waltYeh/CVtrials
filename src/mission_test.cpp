#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "Eigen/Dense"

#define P_pos 0.3f
#define D_pos 0.01f

using namespace Eigen;

void odometryCallback(const nav_msgs::Odometry &odometry);
void positionsetpointCallback(const geometry_msgs::PoseStamped &position_setpoint);
bool near_bool(float x, float y);


bool takeoff = false;
bool reset_pos = false;

std_msgs::Empty order;
geometry_msgs::Twist cmd;

Vector3f pos(0.0,0.0,0.0);
Vector3f pos_ref(0.0,0.0,0.0);
Vector3f pos_sp_1(3.0,0.0,0.0);
Vector3f pos_sp_2(0.5,0.5,0.0);
Vector3f pos_sp_3(0.0,0.5,0.0);
Vector3f direction(0.0,0.0,0.0);
Vector3f vel_sp(0.0,0.0,0.0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_control");
	ros::NodeHandle n;
	ros::Subscriber pos_sub = n.subscribe("/ardrone/odometry", 10, odometryCallback);
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
	pos(0) = odometry.pose.pose.position.x;
	pos(1) = odometry.pose.pose.position.y;
	if(!reset_pos && odometry.pose.pose.position.z > 0.3)
	{
		pos_ref = pos;
		reset_pos = true;
	}
	if(reset_pos)
	{
		if(near_bool(pos(0), (pos_sp_1(0) + pos_ref(0))) && near_bool(pos(1), (pos_sp_1(1) + pos_ref(1))))
		{
			cmd.linear.x = 0.0;
			cmd.linear.y = 0.0;
		}else
		{
			direction = pos_sp_1 + pos_ref - pos;
			direction = direction.normalized();
			cmd.linear.x = 0.1 * direction(0);
			cmd.linear.y = 0.1 * direction(1);
		}	
	}
}

bool near_bool(float x, float y)
{
	if(fabsf( x - y ) <  0.3)
		return true;
	else return false;
}