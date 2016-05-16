#include "image_process.h"

using namespace cv;
using namespace std;

float Color_Detection(IplImage* src,IplImage* dst,double &xpositon,double& yposition)
{
	int sum=0,xsum=0,ysum=0;
	float percent=0;
	cvSmooth(src,src,CV_MEDIAN,5,5);
	IplImage *src_float = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F, 3);
	cvConvertScale(src, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	int step = hsv_img->widthStep/sizeof(float);
	int channels = hsv_img->nChannels;
	float * datafloat = (float *)hsv_img->imageData;
	for(int i = 0; i < hsv_img->height; i++)
	{
		for(int j = 0; j < hsv_img->width; j++)
		{
			if(datafloat[i*step + j*channels + 2]>10&&(datafloat[i*step + j*channels + 1]>0.2)&&(datafloat[i*step + j*channels]>130&&datafloat[i*step + j*channels]<310))
			{
				src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
				src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
				src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=0;
			}
			else
			{
				src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
				src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
				src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=255;
				sum++;
				xsum+=(j+1);
				ysum+=(i+1);
			}
		}
	}

	percent=1.0*sum/(hsv_img->height*hsv_img->width);
	if(sum>0)
	{
		xpositon=1.0*xsum/sum;
		yposition=1.0*ysum/sum;
	}
	else
	{
		xpositon=0;
		yposition=0;
	}
	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
	return percent;
}

float Color_Detection_Pro(IplImage* src,double &xpositon,double& yposition,double& xpositionblue,double& ypositionblue,int mode)
{
	int scale =10;
	int sum=0,xsum=0,ysum=0;
	int sumblue=0,xbluesum=0,ybluesum=0;
	float percent=0;
	CvSize size=cvGetSize(src);
	CvSize newsize;
	newsize.width=size.width/scale;
	newsize.height=size.height/scale;
	cvSmooth(src,src,CV_MEDIAN,5,5);
	IplImage *src_float = cvCreateImage(size,IPL_DEPTH_32F, 3);
	cvConvertScale(src, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(size, IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	if(mode==3)
	{
		int step = hsv_img->widthStep/sizeof(float);
		int channels = hsv_img->nChannels;
		float * datafloat = (float *)hsv_img->imageData;
		for(int i = 0; i < hsv_img->height; i++)
		{
			for(int j = 0; j < hsv_img->width; j++)
			{
				if(datafloat[i*step + j*channels + 2]>40&&(datafloat[i*step + j*channels + 1]>0.3)&&((datafloat[i*step + j*channels]>=0&&datafloat[i*step + j*channels]<10)||(datafloat[i*step + j*channels]>340&&datafloat[i*step + j*channels]<=360)))
				{
					src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
					sum++;
					xsum+=(j+1);
					ysum+=(i+1);
				}
				else
				{
					src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
				}
			}
		}
		percent=1.0*sum/(hsv_img->height*hsv_img->width);
		if(sum>0)
		{
			xpositon=1.0*xsum/sum;
			yposition=1.0*ysum/sum;
		}
		else
		{
			xpositon=0;
			yposition=0;
		}
	}
	else
	{
		IplImage *src_blue = cvCreateImage(size,IPL_DEPTH_8U, 1);
		IplImage *src_blue_resize = cvCreateImage(newsize,IPL_DEPTH_8U, 1);
		IplImage *src_blue_canny = cvCreateImage(newsize,IPL_DEPTH_8U, 1);
		int step = hsv_img->widthStep/sizeof(float);
		int channels = hsv_img->nChannels;
		float * datafloat = (float *)hsv_img->imageData;
		for(int i = 0; i < hsv_img->height; i++)
		{
			for(int j = 0; j < hsv_img->width; j++)
			{
				if(datafloat[i*step + j*channels + 2]>40&&(datafloat[i*step + j*channels + 1]>0.5)&&(datafloat[i*step + j*channels]>40&&datafloat[i*step + j*channels]<80))
				{
					src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
					sum++;
					xsum+=(j+1);
					ysum+=(i+1);
				}
				else
				{
					src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
					src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
				}
				if(datafloat[i*step + j*channels + 2]>20&&(datafloat[i*step + j*channels + 1]>0.3)&&(datafloat[i*step + j*channels]>40&&datafloat[i*step + j*channels]<80))
				{
					src_blue->imageData[i*(src_blue->widthStep)+j*(src_blue->nChannels)]=255;
					sumblue++;
					xbluesum+=(j+1);
					ybluesum+=(i+1);
				}
				else
				{
					src_blue->imageData[i*(src_blue->widthStep)+j*(src_blue->nChannels)]=0;
				}
			}
		}
		cvShowImage("blue image",src_blue);
		 //detect lines in blue mode
		cvResize(src_blue,src_blue_resize);
		cvCanny(src_blue_resize,src_blue_canny,200,220,3);
		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* lines = 0;
		lines = cvHoughLines2(src_blue_canny, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, 30, 10);
		if(lines->total>0)
		{

			CvPoint* line = (CvPoint*)cvGetSeqElem(lines, 0);
			if((1.0*abs(line[0].y-line[1].y)/(abs(line[0].x-line[1].x)+0.1))>newsize.height/10)
			{
				xpositionblue=(line[0].x+line[1].x)*scale/2;
				ypositionblue=(line[0].x+line[1].x)*scale/2;
				cvLine(src_blue_canny, line[0], line[1], CV_RGB(255, 255, 255), 3, CV_AA, 0);
			}
			else
			{
				xpositionblue=0;
				ypositionblue=0;
			}

		}
		cvShowImage("line Detector",src_blue_canny);
		percent=1.0*sum/(hsv_img->height*hsv_img->width);
		if(sum>0)
		{
			xpositon=1.0*xsum/sum;
			yposition=1.0*ysum/sum;
		}
		else
		{
			xpositon=0;
			yposition=0;
		}
		cvReleaseImage(&src_blue);
		cvReleaseImage(&src_blue_resize);
		cvReleaseImage(&src_blue_canny);
		cvReleaseMemStorage(&storage);
	}
	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
	return percent;
}

//Input: 8比特、单通道(二值)图像
void edge_extracting(IplImage* src, IplImage* dst)
{
	cvCanny(src,dst,200,220,3);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, 30, 10);
	for (int i=0;i<lines->total;i++)  
    {  
        CvPoint *line = (CvPoint *)cvGetSeqElem(lines,i);  
        cvLine(dst,line[0],line[1],CV_RGB(255,255,255),3,CV_AA,0);  
    }  

	cvShowImage("line Detector",dst);
	cvReleaseMemStorage(&storage);
}

//Input: 8比特、单通道(二值)图像
float find_center(IplImage* src, double &x, double &y)
{
	int sum=0,xsum=0,ysum=0;
	float percent=0;
	unsigned char* data=(unsigned char *)src->imageData;  
	int step = src->widthStep/sizeof(unsigned char);  

	for(int i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			if(data[i*step + j] > 100)
			{
				sum++;
				xsum+=(j+1);
				ysum+=(i+1);
			}
		}
	}
	if(sum > 0)
	{
		x = xsum/sum;
		y = ysum/sum;
		percent = 1.0 * sum / (src->height * src->width);
	}
	
	return percent;
}