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
    scan.header.frame_id = "vision_laser_link";
    scan.angle_min = (M_PI * 60.0/180.0) - (M_PI/2.0); //assume scanning from 0->180
    scan.angle_max = (M_PI * 115.0/180.0) - (M_PI/2.0);
    scan.angle_increment = (M_PI / 180);
    scan.time_increment = (1 / 60) / res;
    scan.range_min = RANGE_MIN;
    scan.range_max = RANGE;

    scan.set_ranges_size(115-60);
    scan.set_intensities_size(115-60);

    //build dataset
    double theta;
    //int i = (in->cols-1) / 2; //scan from center of bottom
    //int j = (in->rows-1);
    int i = ORIGIN_X;
    int j = ORIGIN_Y;
    int k = 0;

    //for(theta = M_PI * (60.0/180.0); theta < M_PI * (115.0/180.0); theta += scan.angle_increment){
    for(theta = M_PI * (60.0/180.0); theta < M_PI * (115.0/180.0); theta += scan.angle_increment){
        scan.ranges[k] = ray(in, i, j, theta);
        //scan.ranges[k] = (scan.ranges[k] < RANGE_MIN) ? RANGE_MIN : scan.ranges[k];
        k++;
    }   

}

float ray(Mat* in, int i, int j, double theta){
    double x = cos(theta);
    double y = sin(theta);
    int h;
    signed int xx, yy;

    for(h = 1; h < RANGE * PPM; h++){
        xx = floor(i+(x*h));
        yy = floor(j-(y*h));
    
        if(xx > 0 && xx < in->cols && yy > 0 && yy < in->rows)
            if(in->at<uint8_t>(yy,xx) != 0)
                return (float) h / PPM;
    }

    //else return max range
    return RANGE;
}

void sexysegment(Mat* in, Mat* out){
    Mat mid(in->rows, in->cols, CV_8UC3);
    Mat gray(in->rows, in->cols, CV_8UC1);
    Mat temp(in->rows, in->cols, CV_8UC1);

    pyrMeanShiftFiltering(*in, mid, 2, 30, 2);
    //cvtColor(mid, gray, CV_BGR2GRAY);
    std::vector<Mat> bgr; 
    split(mid, bgr);
    
    addWeighted(bgr[0], 2.0, bgr[1], -1.0, 0, gray);
    addWeighted(gray, 2.0, bgr[2], -1.0, 0, gray);

    /*
    for(int j=0; j<in->rows/3; j++){
        for(int i=0; i<in->cols; i++){
            gray.at<uint8_t>(j,i) = 0;
        }
    }
    */

    

    Mat mystruct = getStructuringElement(MORPH_RECT, Size(37,37));//15
    morphologyEx(gray, temp, MORPH_TOPHAT, mystruct, Point(-1,-1), 1);
    adaptiveThreshold(temp, gray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 43, -5 );

    mystruct = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(gray, gray, MORPH_OPEN, mystruct, Point(-1,-1), 1);

    //remove blobs
    std::vector<std::vector<Point> > v;
    std::vector<std::vector<Point> > vb;
    
    findContours(gray, v, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


    for(unsigned int i=0; i<v.size(); i++){
        RotatedRect r = minAreaRect(v[i]);

        if( (r.size.width / r.size.height > 1.5 ||
            r.size.height / r.size.width > 1.5) &&
            (r.size.height > 0 ||
             r.size.width > 0)){ //50
            vb.push_back(v[i]);
        }
    }

    *out = Mat::zeros(out->size(), out->type());
    //bw out
    //drawContours(*out, vb, -1, Scalar(255,255,255), -1);
            

    //for(int j=0; j<gray.rows; j++){
        //for(int i=0; i<gray.cols; i++){
            //gray.at<uint8_t>(j,i) = 0;
        //}
    //}
    //*out = in->clone();
    //drawContours(gray, vb, -1, Scalar(255,255,255), -1);
    

    
    gray = Mat::zeros(gray.size(), gray.type());
    drawContours(gray, vb, -1, Scalar(255,255,255), -1);
    //hough
    std::vector<Vec4i> lines;
    HoughLinesP(gray, lines, 1, CV_PI/180, 60, 60, 10);
    //HoughLinesP(gray, lines, 1, CV_PI/180, 120, 100, 5);

    for(size_t i=0; i<lines.size(); i++){
        line(*out, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255,255,255), 3, 8);
    }

    /*
    color transform
    *out = in->clone();
    drawContours(*out, vb, -1, Scalar(255,0,0), -1);
      
    for(size_t i=0; i<lines.size(); i++)
        line(*out, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8);
*/
}



















