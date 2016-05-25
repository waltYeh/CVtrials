//Size of the image is 640*360
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ardrone_control/ROI.h"
#include "image_process.h"

using namespace cv;
using namespace std;

class FindContour
{
public: 
	FindContour();
private:
	ros::NodeHandle n;
	ros::Subscriber image_sub;
	ros::Subscriber odometry_sub;
	ros::Publisher image_pub;
	ros::Publisher ROI_image_pub;
	double minarea;
	double ROI_width;
	double target_image[10][2];
	double x,y;

	IplImage *source_image;
	IplImage *source_image_resized;
	IplImage *ROI_image;
	IplImage temp;
	IplConvKernel * myModel;
	ardrone_control::ROI ROI_msg;

	void imageCallback(const sensor_msgs::Image &msg);
	void odometryCallback(const nav_msgs::Odometry &msg);

};

FindContour::FindContour()
{
	image_sub = n.subscribe("/ardrone/bottom/image_raw", 1, &FindContour::imageCallback,this);
	odometry_sub = n.subscribe("/ardrone/odometry", 1, &FindContour::odometryCallback,this);
	image_pub = n.advertise<ardrone_control::ROI>("/ROI_image", 1);
	ROI_image_pub = n.advertise<sensor_msgs::Image>("/ROI", 1);
	minarea = 15000;
	ROI_width = 70;
	source_image_resized = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
	ROI_image = cvCreateImage(cvSize(70,70),IPL_DEPTH_8U, 3);
	myModel = cvCreateStructuringElementEx(20,20,2,2,CV_SHAPE_RECT);
}

void FindContour::imageCallback(const sensor_msgs::Image &msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImage cv_to_ros;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	temp = (IplImage)(cv_ptr->image);
	source_image = &temp;
	cvResize(source_image, source_image_resized);
	
	//detect the yellow region
	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);
	//IplImage *image_threshold_origin = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);
	Color_Detection(source_image_resized, image_threshold, x, y);	
	//cvCopy(image_threshold,image_threshold_origin);

	cvDilate(image_threshold, image_threshold, myModel, 1);
	//cvErode(image_threshold, image_threshold, myModel, 1);

	//find the contours
	CvMemStorage* storage = cvCreateMemStorage(0);  
	CvSeq* contour = 0;  
	cvFindContours(image_threshold, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cvZero(image_threshold);

	//get rid of the contours which don't meet the requirement
	int count = 0; 
	for( ; contour != 0; contour = contour->h_next )
	{  
	 	double tmparea = fabs(cvContourArea(contour));  
	 	// printf("Area:%f\n", tmparea);
	 	if (tmparea < minarea)  
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
		cvRectangle(image_threshold, cvPoint(aRect.x, aRect.y), cvPoint(aRect.x + aRect.width, aRect.y + aRect.height),CV_RGB(255,255, 255), 1, 8, 0);
		cvRectangle(source_image_resized, cvPoint(aRect.x, aRect.y), cvPoint(aRect.x + aRect.width, aRect.y + aRect.height),CV_RGB(0,255, 0), 3, 8, 0);
		count++;
		
	} 
	cvShowImage("Original Image", source_image_resized);
	waitKey(1);

	ROI_msg.total = count;
	for(int i = 0 ; i < count ; i++)
	{
		CvRect rect;
		rect.x = target_image[i][0] - ROI_width/2;
		rect.y = target_image[i][1] - ROI_width/2;
		rect.width = ROI_width;
		rect.height = ROI_width;
		//get the ROI region
		cvSetImageROI(source_image_resized,rect);
		cvCopy(source_image_resized,ROI_image);  
		cvResetImageROI(source_image_resized); 

		if(i == 0){
			cv_to_ros.image = ROI_image;
			cv_to_ros.header = cv_ptr->header;
			cv_to_ros.encoding = sensor_msgs::image_encodings::BGR8;
			ROI_msg.image1  = *(cv_to_ros.toImageMsg());
			ROI_msg.pose1.x = target_image[i][0];
			ROI_msg.pose1.y = target_image[i][1];
			imshow("test1", cv_to_ros.image);
			waitKey(1);
			ROI_image_pub.publish(ROI_msg.image1);

		}

		if(i == 1){
			cv_to_ros.image = ROI_image;
			cv_to_ros.header = cv_ptr->header;
			cv_to_ros.encoding = sensor_msgs::image_encodings::BGR8;
			ROI_msg.image2  = *(cv_to_ros.toImageMsg());
			ROI_msg.pose2.x = target_image[i][0];
			ROI_msg.pose2.y = target_image[i][1];
			imshow("test2", cv_to_ros.image);
			waitKey(1);
		}

		if(i == 2){
			cv_to_ros.image = ROI_image;
			cv_to_ros.header = cv_ptr->header;
			cv_to_ros.encoding = sensor_msgs::image_encodings::BGR8;
			ROI_msg.image3  = *(cv_to_ros.toImageMsg());
			ROI_msg.pose3.x = target_image[i][0];
			ROI_msg.pose3.y = target_image[i][1];
			imshow("test3", cv_to_ros.image);
			waitKey(1);
		}	
	}
	if(count != 0){
		image_pub.publish(ROI_msg);
	}
	
	cvReleaseMemStorage(&storage); 
	cvReleaseImage(&image_threshold);
}

void FindContour::odometryCallback(const nav_msgs::Odometry &msg)
{
	//msg.pose.pose.position.z
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_contour");
	FindContour FindContour;
	ros::spin();
}
