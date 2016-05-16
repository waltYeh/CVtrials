#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "image_process.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_control");
	ros::NodeHandle n;
	ros::Publisher pos_sp = n.advertise<geometry_msgs::PoseStamped>("/image_position_sp", 10);
	ros::Rate loop_rate(10);

	float percent;
	double x, y;

	IplImage *image = cvLoadImage("images/PIC6.jpg");
	IplImage *image_threshold = cvCreateImage(cvGetSize(image),IPL_DEPTH_8U, 1);

	cvShowImage("Original Image", image);

	percent = Color_Detection(image,image_threshold,x,y);
	cvShowImage("Threshold Image", image_threshold);
	ROS_INFO("X:%f\nY:%f\n",x,y);

	IplConvKernel * myModel;
	myModel = cvCreateStructuringElementEx(40,40,2,2,CV_SHAPE_RECT);
	cvDilate(image_threshold,image_threshold,myModel,1);
	cvShowImage("Dilate Image", image_threshold);

	IplImage *image_line = cvCreateImage(cvGetSize(image_threshold),IPL_DEPTH_8U, 1);
	edge_extracting(image_threshold, image_line);

	waitKey(0);
	return 0;
}