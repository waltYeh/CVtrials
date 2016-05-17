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

double minarea = 30000; 
float percent;
double x, y;

IplImage *source_image = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
IplImage *image_threshold = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 1);
Mat image;
IplImage temp;

IplConvKernel * myModel = cvCreateStructuringElementEx(30,30,2,2,CV_SHAPE_RECT);

geometry_msgs::PoseStamped image_pos;

void imageCallback(const sensor_msgs::Image &image);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_contour");
	ros::NodeHandle n;
	ros::Subscriber image_sub = n.subscribe("/ardrone/bottom/image_raw", 1, imageCallback);
	ros::Publisher image_pub = n.advertise<geometry_msgs::PoseStamped>("/image_position", 1);
	ros::Rate loop_rate(30);

	while(ros::ok())
	{
		

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

	percent = Color_Detection(source_image, image_threshold, x, y);	
	cvDilate(image_threshold, image_threshold, myModel, 1);

	CvMemStorage* storage = cvCreateMemStorage(0);  
	CvSeq* contour = 0;  
	cvFindContours(image_threshold, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	CvSeq* _contour = contour;  

	for (int iteratorIdx = 0; contour != 0; contour = contour->h_next, iteratorIdx++)  
	{  
		double tmparea = fabs(cvContourArea(contour));  
		printf("Area:%f\n", tmparea);
		if (tmparea < minarea)  
		{  
			cvSeqRemove(contour, 0);   
			continue;  
		}  
	} 
	contour = _contour;
	cvDrawContours(image_threshold, contour, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, 1, 8);  
	
	cvShowImage("Contour", image_threshold); 
	waitKey(1);
}
