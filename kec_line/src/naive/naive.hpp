#ifndef __KECNAIVE
#define __KECNAIVE

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#define RANGE   5.0
#define PPM     75

/*
int avgValue(IplImage* in);
void optimumThreshold(IplImage* in, IplImage* out, int t);
void lineDetect(IplImage* in, IplImage* out);

//pixel
int test_pixel(IplImage* in, int x, int y);
void set_pixel(IplImage* in, int x, int y, int value);

//raycast
void ray(IplImage* in, int i, int j, double theta, int* ii, int* jj);
void raycast(IplImage* in, IplImage* out);
int bersenham(IplImage* in, int i0, int j0, int i1, int j1);
*/

using namespace cv;

int avgValue(Mat* in);
void optimumThreshold(Mat* in, Mat* out, int t);
void segment(Mat* in, Mat* out);

#endif
