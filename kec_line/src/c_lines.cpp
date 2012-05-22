#include "ros/ros.h"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "naive/naive.hpp"

namespace enc = sensor_msgs::image_encodings;

class DetectLines {
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    image_transport::Subscriber it_sub;

public:
    DetectLines() : it(n){
        it_sub = it.subscribe("cam", 1, &DetectLines::got_frame, this);
        cv::namedWindow("in");
        cv::namedWindow("out");
    }

    ~DetectLines(){
        cv::destroyWindow("in");
        cv::destroyWindow("out");
    }

    void got_frame(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr frame;

        //bridge
        try{
            frame = cv_bridge::toCvCopy(msg, enc::BGR8);
        }catch(cv_bridge::Exception &e){
            //ROS_ERROR("kec_line exception: %s", e.what());
            std::cout << "kec_line exception: " << e.what() << std::endl;
            return;
        }

        cv::Mat out(frame->image.rows, frame->image.cols, CV_8UC1);

        //lineDetect(frame->image, &iplout);
        segment(&frame->image, &out);

        //show image
        cv::imshow("in", frame->image);
        cv::imshow("out", out);
        cv::waitKey(1);
        out.release();
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "c_lines");
    DetectLines oop;
    ros::spin();
    return 0;
}
