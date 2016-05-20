//Size of the image is 640*360
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "image_process.h"

using namespace cv;
using namespace std;

float percent;
double x, y;

IplImage *source_image;
IplImage *source_image_resized = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);

//IplImage *source_image = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
//IplImage *image_threshold = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 1);
Mat image;
IplImage temp;

IplConvKernel * myModel = cvCreateStructuringElementEx(10,10,2,2,CV_SHAPE_RECT);

geometry_msgs::PoseStamped image_pos;

void imageCallback(const sensor_msgs::Image &image);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_control");
	ros::NodeHandle n;
	ros::Subscriber image_sub = n.subscribe("/videofile/image_raw", 1, imageCallback);
	ros::Publisher image_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/image_position", 1);
	ros::Rate loop_rate(20);

	while(ros::ok())
	{	
		image_pos.pose.position.x = x;
		image_pos.pose.position.y = y;
		image_pos_pub.publish(image_pos);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

void imageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	image = cv_ptr->image;  
	//imshow("Original Image", cv_ptr->image);
	//waitKey(1);
	temp = (IplImage)image;
	source_image = &temp;
	cvResize(source_image, source_image_resized);
	cvShowImage("Original Image", source_image_resized);
	waitKey(1);

	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);

	percent = Color_Detection(source_image_resized, image_threshold, x, y);
	//ROS_INFO("\nX:%f\nY:%f\npercent:%f\n",x,y,percent);
	//cvErode(image_threshold, image_threshold, myModel, 1);
	//cvDilate(image_threshold, image_threshold, myModel, 1);
	cvShowImage("Dilate Image", image_threshold);
	waitKey(1);

	percent = find_center(image_threshold,x,y);
	ROS_INFO("\nX:%f\nY:%f\npercent:%f\n",x,y,percent);

	cvReleaseImage(&image_threshold);
}

