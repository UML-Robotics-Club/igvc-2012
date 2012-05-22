#include "naive.hpp"
#include <stdint.h>
#include <iostream>
#include <vector>

using namespace cv;

int avgValue(Mat* in){
    //std::cout << "avg val start" <<std::endl;
    Scalar temp = mean(*in);
    //std::cout << "avg val done" <<std::endl;
    return temp[0];
}

void optimumThreshold(Mat* in, Mat* out, int t){
    int i, j;
    double m1 = 0;
    double m2 = 0;
    double c1 = 0;
    double c2 = 0;
    double tp = 0;

    //find optimum threshold
    while(abs(t-tp) > 10){
        for(i=0; i<in->rows; i++){
            for(j=0; j<in->cols; j++){
                if(in->at<uint8_t>(i,j) > t){
                    m1 += in->at<uint8_t>(i,j);
                    c1++;
                }else{
                    m2 += in->at<uint8_t>(i,j);
                    c2++;
                }
            }
        }

        tp = t;
        t = ((m1/c1) + (m2/c2))/2;

        m1=m2=c1=c2=0;
    }

    //threshold
    threshold(*in, *out, t, 255, CV_THRESH_BINARY);
}

void segment(Mat* in, Mat* out){
    
    Mat hsv(in->cols, in->rows, CV_8UC3);
    Mat mid(in->cols, in->rows, CV_8UC3);
    Mat mixed(in->cols, in->rows, CV_8UC1);

    const int hue = 0;
    const int sat = 1;
    const int val = 2;
    const int b   = 0;
    const int g   = 1;
    const int r   = 2;

    //convert to hsv and split
    cvtColor(*in, hsv, CV_BGR2HSV, 0);
    //std::vector<Mat> hsvVec = split(hsv);
    std::vector<Mat> hsvVec;
    split(hsv, hsvVec);

    if(avgValue(&hsvVec[sat]) < 80){
        equalizeHist(hsvVec[val], hsvVec[val]);
        equalizeHist(hsvVec[sat], hsvVec[sat]);

        merge(hsvVec, hsv);
        cvtColor(hsv, mid, CV_HSV2BGR, 0);
        //std::vector<Mat> bgr = split(mid);
        std::vector<Mat> bgr; 
        split(mid, bgr);
        
        //do the thing
        addWeighted(bgr[b], 1.0, bgr[g], -1.0, 0, mixed);
        addWeighted(mixed, 1.0, bgr[r], -1.0, 0, mixed);
    }else{
        std::vector<Mat> bgr;
        split(*in, bgr);

        addWeighted(bgr[b], 2.0, bgr[g], -1.0, 0, mixed);
        addWeighted(mixed, 2.0, bgr[r], -1.0, 0, mixed);
    }
    
    //find optimum threshold
    optimumThreshold(&mixed, out, 127);
}

/*
int test_pixel(IplImage* in, int x, int y){
    if(x < 0 || x >= in->width){
        //printf("%d, %d, x\n", x, y);
        return 0;
    }
    if(y < 0 || y >= in->height){
        //printf("%d, %d, y\n", x, y);
        return 0;
    }else
        return ((unsigned char*) &(in->imageData[y*in->width]))[x];
}

void set_pixel(IplImage* in, int x, int y, int value){
    ((unsigned char*) &(in->imageData[y*in->width]))[x] = value;
}


//cast rays from the center of the bottom out. the first
//white pixel the ray hits turns into a white pixel in the
//output image
void raycast(IplImage* in, IplImage* out){
    double theta;
    int i = (in->width-1) / 2;
    int j = (in->height-1);
    int x, y;

    //for(theta = (3.0/2.0) * M_PI; theta > M_PI / 2.0; theta -= M_PI / 180){
    for(theta = 2.0 * M_PI; theta > M_PI ; theta -= M_PI / 180){
        ray(in, i, j, theta, &x, &y);
        if(x != 0 && y != 0)
            set_pixel(out, x, y, 255);
    }
}


//ii and jj will contain the coordinates of a hit, 0, 0 if no hits
//in range
void ray(IplImage* in, int i, int j, double theta, int* ii, int* jj){
    double x = cos(theta); 
    double y = sin(theta);
    int h;
    int xx, yy;

    *ii = *jj = 0;

    for(h = 1; h < RANGE * PPM; h++){
        xx = floor(i+(x*h));
        yy = floor(j+(y*h));
        if(test_pixel(in, xx, yy)){
            *ii = xx;
            *jj = yy;
            break;
        }
    }
}

//value of pixel at end of bersenham's march
int bersenham(IplImage* in, int i0, int j0, int i1, int j1){
    char steep;
    int t;
    int dx;
    int dy;
    double err = 0;
    double derr;
    int ystep;
    int x, y;

    steep = abs(j1 - j0) > abs(i1 - i0);
    if(steep != 0){
        t = i0;
        i0 = j0;
        j0 = t;
        
        t = i1;
        i1 = j1;
        j1 = t;
    }    

    if(i0 > i1){
        t = i0;
        i0 = i1;
        i1 = t;

        t = j0;
        j0 = j1;
        j1 = t;
    }

    dx = i1 - i0;
    dy = abs(j1 - j0);
    derr = dy / dx;
    
    y = j0;
    ystep = (j0 < j1) ? 1 : -1;

    for(x=i0; x<i1; x++){
        //if steep:
        //set_pixel(in, x, y, 255);
        //else:
        //set_pixel(in, y, x, 255);
        err += derr;
        if(err >= 0.5){
            y += ystep;
            err -= 1.0;
        }
    }

    return test_pixel(in, x, y);
}
*/






















