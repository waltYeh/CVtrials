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
double target_image[10][2];

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
	ros::Subscriber image_sub = n.subscribe("/ardrone/image_raw", 1, imageCallback);
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
	temp = (IplImage)image;
	source_image = &temp;
	cvResize(source_image, source_image_resized);

	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);

	percent = Color_Detection(source_image_resized, image_threshold, x, y);
	//ROS_INFO("\nX:%f\nY:%f\npercent:%f\n",x,y,percent);
	//cvErode(image_threshold, image_threshold, myModel, 1);
	//cvDilate(image_threshold, image_threshold, myModel, 1);
	cvDilate(image_threshold, image_threshold, myModel, 1);

	CvMemStorage* storage = cvCreateMemStorage(0);  
	CvSeq* contour = 0;  
	cvFindContours(image_threshold, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cvZero(image_threshold);

	//get rid of the contours which don't meet the requirement
	int count = 0; 
	for( ; contour != 0; contour = contour->h_next )
	{  
	 	double tmparea = fabs(cvContourArea(contour));  
	 	printf("Area:%f\n", tmparea);
	 	if (tmparea < 1000)  
	 	{  
	 		cvSeqRemove(contour, 0); 
	 		continue; 
	 	}  
		CvRect aRect = cvBoundingRect(contour, 0 );
		if ((aRect.width/aRect.height)<0.8 || (aRect.width/aRect.height)> 1.2)    
		{    
			cvSeqRemove(contour,0); 
			continue;    
		}
		//draw the contours
		cvDrawContours(image_threshold, contour, CV_RGB( 255, 255, 255), CV_RGB( 255, 255, 255), 0, 1, 8 );
		//find the center of the contours
		target_image[count][0] = (aRect.x + aRect.x + aRect.width) / 2;
		target_image[count][1] = (aRect.y + aRect.y + aRect.height) / 2;
		//draw a rectangle of the contour region
		cvRectangle(image_threshold, cvPoint(target_image[count][0] - 50, target_image[count][1] - 50), cvPoint(target_image[count][0] + 50, target_image[count][1] + 50),CV_RGB(255,255, 255), 1, 8, 0);
		cvRectangle(source_image_resized, cvPoint(target_image[count][0] - 50, target_image[count][1] - 50), cvPoint(target_image[count][0] + 50, target_image[count][1] + 50),CV_RGB(0,255, 0), 3, 8, 0);
		count++;
		
	} 
	cvShowImage("Original Image", source_image_resized);
	waitKey(1);

	cvReleaseImage(&image_threshold);
}

