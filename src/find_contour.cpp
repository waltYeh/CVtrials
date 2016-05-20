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

double minarea = 10000 ; 
float percent;
double x, y;

IplImage *source_image;
IplImage *source_image_resized = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
IplImage *counter_image = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
Mat image;
IplImage temp;

IplConvKernel * myModel = cvCreateStructuringElementEx(20,20,2,2,CV_SHAPE_RECT);

geometry_msgs::PoseStamped image_pos;

void imageCallback(const sensor_msgs::Image &image);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_contour");
	ros::NodeHandle n;
	ros::Subscriber image_sub = n.subscribe("/videofile/image_raw", 1, imageCallback);
	ros::Publisher image_pub = n.advertise<geometry_msgs::PoseStamped>("/image_position", 1);
	ros::Rate loop_rate(20);

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
	cvResize(source_image, source_image_resized);
	cvShowImage("Original Image", source_image_resized);
	waitKey(1);

	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);
	percent = Color_Detection(source_image_resized, image_threshold, x, y);	
	
	//cvDilate(image_threshold, image_threshold, myModel, 1);
	//cvErode(image_threshold, image_threshold, myModel, 1);
	
	
	cvShowImage("Threshold", image_threshold); 
	waitKey(1);

	CvMemStorage* storage = cvCreateMemStorage(0);  
	CvSeq* contour = 0;  
	cvFindContours(image_threshold, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	cvZero(image_threshold);
	CvSeq* _contour = contour;  
	 	 
	for( ; contour != 0; contour = contour->h_next )
	{  
	 	double tmparea = fabs(cvContourArea(contour));  
	 	//printf("Area:%f\n", tmparea);
	 	if (tmparea < minarea)  
	 	{  
	 		cvSeqRemove(contour, 0); 
	 		continue; 
	 	}  
		CvRect aRect = cvBoundingRect(contour, 0 );     
		if ((aRect.width/aRect.height)<0.9 || (aRect.width/aRect.height)> 1.1)    
		{    
			cvSeqRemove(contour,0); 
			continue;    
		}
		cvDrawContours(image_threshold, contour, CV_RGB( 255, 255, 255), CV_RGB( 255, 255, 255), -1, 1, 8 );
	} 
	contour = _contour;  

	//printf("Total:%d\n", count);
	
	cvShowImage("Contour_image", image_threshold); 
	waitKey(1);
	cvReleaseImage(&image_threshold);
}
