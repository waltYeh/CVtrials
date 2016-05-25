#ifndef   __IMAGE_PROCESS_H__ 
#define   __IMAGE_PROCESS_H__ 

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

float Color_Detection(IplImage* src,IplImage* dst,double &xpositon,double& yposition);
void Number_Detection(IplImage* src,IplImage* dst);
void edge_extracting(IplImage* src, IplImage* dst);
float find_center(IplImage* src, double &x, double &y);

#endif
