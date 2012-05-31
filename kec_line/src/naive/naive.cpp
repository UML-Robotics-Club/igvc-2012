#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include "naive.hpp"

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

void raycast(Mat* in, sensor_msgs::LaserScan& scan){
    ros::Time scan_time = ros::Time::now();
    const int res = SCAN_RES;

    scan.header.stamp = scan_time;
    scan.header.frame_id = "lase_link";
    scan.angle_min = 0; //assume scanning from 0->180
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / res;
    scan.time_increment = (1 / 60) / res;
    scan.range_min = RANGE_MIN;
    scan.range_max = RANGE;

    scan.set_ranges_size(res);
    scan.set_intensities_size(res);

    //build dataset
    double theta;
    //int i = (in->cols-1) / 2; //scan from center of bottom
    //int j = (in->rows-1);
    int i = ORIGIN_X;
    int j = ORIGIN_Y;
    int k = 0;

    for(theta = 2.0 * M_PI; theta > M_PI; theta -= M_PI / res){
        scan.ranges[k] = ray(in, i, j, theta);
        scan.ranges[k] += (scan.ranges[k] < 5.0 - RANGE_MIN) ? RANGE_MIN : 0;
        k++;
    }   

}

float ray(Mat* in, int i, int j, double theta){
    double x = cos(theta);
    double y = sin(theta);
    int h;
    int xx, yy;

    for(h = 1; h < RANGE * PPM; h++){
        xx = floor(i+(x*h));
        yy = floor(j+(y*h));
    
        if(in->at<uint8_t>(xx,yy) != 0)
            return (float) h / PPM;
    }

    //else return max range
    return RANGE;
}
