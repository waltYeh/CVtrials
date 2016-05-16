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

IplImage *source_image = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
IplImage *image_threshold = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 1);
Mat image;
IplImage temp;

void imageCallback(const sensor_msgs::Image &image);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_control");
	ros::NodeHandle n;
	ros::Subscriber image_sub = n.subscribe("/ardrone/bottom/image_raw", 1, imageCallback);
	ros::Publisher pos_sp = n.advertise<geometry_msgs::PoseStamped>("/image_position_sp", 1);
	ros::Rate loop_rate(10);

	float percent;
	double x, y;

	IplConvKernel * myModel;
	myModel = cvCreateStructuringElementEx(30,30,2,2,CV_SHAPE_RECT);

	while(ros::ok())
	{
		percent = Color_Detection(source_image, image_threshold, x, y);
		//ROS_INFO("\nX:%f\nY:%f\npercent:%f\n",x,y,percent);
	
		cvDilate(image_threshold, image_threshold, myModel, 1);
		cvShowImage("Dilate Image", image_threshold);
		waitKey(5);

		percent = find_center(image_threshold,x,y);
		ROS_INFO("\nX:%f\nY:%f\npercent:%f\n",x,y,percent);

		//IplImage *image_line = cvCreateImage(cvGetSize(image_threshold),IPL_DEPTH_8U, 1);
		//edge_extracting(image_threshold, image_line);

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
	cvShowImage("Original Image", source_image);
	waitKey(1);
}

