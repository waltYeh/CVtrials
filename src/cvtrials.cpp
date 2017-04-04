#include "image_process.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
const int MAX_CORNERS = 500;


using namespace cv;

int main(int argc, char **argv)
{
	IplImage* imgA=cvLoadImage(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
	IplImage* imgB=cvLoadImage(argv[2],CV_LOAD_IMAGE_GRAYSCALE);
	CvSize img_sz = cvGetSize(imgA);
	int win_size = 10;
	IplImage* imgC = cvLoadImage(argv[2],CV_LOAD_IMAGE_UNCHANGED);
	IplImage* imgD = cvLoadImage(argv[1],CV_LOAD_IMAGE_UNCHANGED);
	IplImage* eig_image = cvCreateImage(img_sz,IPL_DEPTH_32F,1);
	IplImage* tmp_image = cvCreateImage(img_sz,IPL_DEPTH_32F,1);
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[MAX_CORNERS];
	cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,0,3,0,0.04);
	cvFindCornerSubPix(imgA,cornersA,corner_count,cvSize(win_size,win_size),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
	char features_found[MAX_CORNERS];
	float feature_errors[MAX_CORNERS];
	CvSize pyr_sz = cvSize(imgA->width+8,imgB->height/3);
	IplImage* pyrA=cvCreateImage(pyr_sz,IPL_DEPTH_32F,1);
	IplImage* pyrB=cvCreateImage(pyr_sz,IPL_DEPTH_32F,1);
	CvPoint2D32f* cornersB = new CvPoint2D32f[MAX_CORNERS];
	cvCalcOpticalFlowPyrLK(imgA,imgB,pyrA,pyrB, cornersA,cornersB,corner_count,cvSize(win_size,win_size),5,features_found,feature_errors,cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.3),0);
	for(int i=0;i<corner_count;i++){
		if(features_found[i]==0||feature_errors[i]>550){
			printf("Error is %f\n", feature_errors[i]);
			continue;
		}
		printf("Got it\n");
		CvPoint p0=cvPoint(cvRound(cornersA[i].x),cvRound(cornersA[i].y));
		CvPoint p1=cvPoint(cvRound(cornersB[i].x),cvRound(cornersB[i].y));
		cvLine(imgC,p0,p1,CV_RGB(255,0,0),2);
		cvLine(imgD,p0,p1,CV_RGB(255,0,0),2);
	}
	cvNamedWindow("ImageA",0);
	cvNamedWindow("ImageB",0);
	cvNamedWindow("LKpyr_OpticalFlow",0);
	cvShowImage("ImageA",imgA);
	cvShowImage("ImageB",imgB);
	cvShowImage("LKpyr_OpticalFlow",imgC);
	cvShowImage("LKpyr_OpticalFlow2",imgD);
	waitKey(0);
	return 0;
	#if 0	
	Mat img = imread(argv[1],-1);
	if(img.empty())
		return -1;
	namedWindow("Example1",WINDOW_AUTOSIZE);
	imshow("Example1",img);
	waitKey(0);
	destroyWindow("Example1");

	
	return 0;
#endif

}
