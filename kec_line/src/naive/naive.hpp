#ifndef __KECNAIVE
#define __KECNAIVE

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#define RANGE       5.0
#define RANGE_MIN   0.46
#define SCAN_RES    180
#define PPM         100.0
#define ORIGIN_X    191
#define ORIGIN_Y    479

/*
//pixel
int test_pixel(IplImage* in, int x, int y);
void set_pixel(IplImage* in, int x, int y, int value);

int bersenham(IplImage* in, int i0, int j0, int i1, int j1);
*/

using namespace cv;

//segmentation
int avgValue(Mat* in);
void optimumThreshold(Mat* in, Mat* out, int t);
void segment(Mat* in, Mat* out);
void sexysegment(Mat* in, Mat* out);

//raycast
float ray(Mat* in, int i, int j, double theta);
void raycast(Mat* in, sensor_msgs::LaserScan& scan);

#endif
